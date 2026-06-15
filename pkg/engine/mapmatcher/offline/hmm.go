// Package offline provides offline map matching logic
package offline

import (
	"math"
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type HMM struct {
	graph *da.Graph
	re    *routing.CRPRoutingEngine[int32]
	rt    *spatialindex.Rtree
}

type hmmObservation struct {
	observationId int
	gps           *da.GPSPoint
	candidates    []*ma.Candidate
}

func newHMMObservation(observationId int,
	gps *da.GPSPoint,
	candidates []*ma.Candidate) hmmObservation {
	return hmmObservation{observationId: observationId, gps: gps, candidates: candidates}
}

func NewHiddenMarkovModelMapMatching(graph *da.Graph, re *routing.CRPRoutingEngine[int32],
	rt *spatialindex.Rtree) *HMM {
	return &HMM{
		graph: graph,
		re:    re,
		rt:    rt,
	}
}

// default gps accuracy radius
func (h *HMM) MapMatch(gpsTraj []*da.GPSPoint) []*da.MatchedGPSPoint {
	gpsRadiusesM := make([]float64, len(gpsTraj))
	for i := range gpsRadiusesM {
		gpsRadiusesM[i] = defaultGPSAccuracyRadiusM
	}
	matchedPoints, _ := h.MapMatchWithGPSRadiuses(gpsTraj, gpsRadiusesM)
	return matchedPoints
}

/*
implementation of:
Newson, P. & Krumm, J., (2009). "Hidden Markov map matching through noise and
sparseness". Proceedings of the 17th ACM SIGSPATIAL International Conference
on Advances in Geographic Information Systems (GIS ‘09), pp.336–343.
*/
func (h *HMM) MapMatchWithGPSRadiuses(gpsTraj []*da.GPSPoint, gpsRadiusesM []float64) ([]*da.MatchedGPSPoint, []da.Coordinate) {
	return h.mapMatchWithGPSRadiuses(gpsTraj, gpsRadiusesM)
}

func (h *HMM) mapMatchWithGPSRadiuses(gpsTraj []*da.GPSPoint, gpsRadiusesM []float64) ([]*da.MatchedGPSPoint, []da.Coordinate) {
	if len(gpsTraj) == 0 {
		return []*da.MatchedGPSPoint{}, []da.Coordinate{}
	}
	if len(gpsRadiusesM) != len(gpsTraj) {
		return []*da.MatchedGPSPoint{}, []da.Coordinate{}
	}

	candidates, numberOfStates := h.projectAllGpsWithRadiuses(gpsTraj, gpsRadiusesM)
	if numberOfStates == 0 {
		return []*da.MatchedGPSPoint{}, []da.Coordinate{}
	}

	allowUTurns := h.buildAllowUturnSlice(gpsTraj)

	numberOfObservations := len(gpsTraj)
	viterbiDecoder := ma.NewViterbi(numberOfObservations)

	stateData := make([]*ma.Candidate, numberOfStates)
	path := make([]*ma.State, 0, numberOfObservations)

	var (
		prevObs      *da.GPSPoint
		prevCands    []*ma.Candidate
		initialized  bool
		prevObsStack []hmmObservation
	)

	resetTrip := func() {
		viterbiDecoder = ma.NewViterbi(numberOfObservations)
		prevObsStack = prevObsStack[:0]
		prevObs = nil
		prevCands = nil
		initialized = false
	}

	finishTrip := func() {
		if initialized && len(viterbiDecoder.GetForwardProb()) > 0 {
			path = append(path, viterbiDecoder.RetrieveMostLikelyStateSequence()...)
		}
		resetTrip()
	}

	pushAcceptedObsToTrip := func(observationId int, gps *da.GPSPoint, gpsCands []*ma.Candidate) {
		frame := newHMMObservation(
			observationId,
			gps,
			gpsCands,
		)
		prevObsStack = append(prevObsStack, frame)
		prevObs = gps
		prevCands = gpsCands
	}

	for t := 0; t < numberOfObservations; t++ {
		emissionProbMatrix := make(map[int]float64)
		states := make([]int, 0)
		gps := gpsTraj[t]
		allowUTurn := allowUTurns[t]

		gpsCands := candidates[t]
		if len(gpsCands) == 0 {
			continue
		}

		transitionProbMatrix := make(map[ma.Transition]float64)

		for j := 0; j < len(gpsCands); j++ {
			cand := gpsCands[j]
			candProjection := cand.GetProjectedCoord()

			gpsToCandDist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(
				candProjection.GetLat(), candProjection.GetLon(),
				gps.Lat(), gps.Lon(),
			))

			emissionProb := computeEmissionLogProb(gpsToCandDist, gpsRadiusesM[t])

			emissionProbMatrix[cand.GetStateId()] = emissionProb

			states = append(states, cand.GetStateId())

			stateData[cand.GetStateId()] = cand
		}

		if !initialized {
			viterbiDecoder.InitForwardProb(t, states, emissionProbMatrix)
			if viterbiDecoder.IsHMMBroken() {
				resetTrip()
				continue
			}
			initialized = true
			pushAcceptedObsToTrip(t, gps, gpsCands)
			continue

		} else {
			measurementDist := util.KilometerToMeter(
				geo.CalculateGreatCircleDistance(prevObs.Lat(), prevObs.Lon(), gps.Lat(), gps.Lon()),
			)

			deltaTime := gps.Time().Sub(prevObs.Time()).Seconds()

			// Compute transition probabilities
			for j := 0; j < len(prevCands); j++ {
				for k := 0; k < len(gpsCands); k++ {

					prevState := prevCands[j]
					currState := gpsCands[k]

					param := newCalcTransitionProbParam(measurementDist, deltaTime, prevState, currState, allowUTurn)
					transition := h.calcTransitionProb(param)
					if util.Eq(transition.getTransitionProb(), ma.InvalidLogProb) {
						continue
					}
					transitionProbMatrix[transition.getTransition()] = transition.getTransitionProb()
				}
			}

			viterbiDecoder.ForwardStep(t, states, emissionProbMatrix, transitionProbMatrix)
		}

		if viterbiDecoder.IsHMMBroken() {
			// https://www.microsoft.com/en-us/research/wp-content/uploads/2016/12/map-matching-ACM-GIS-camera-ready.pdf
			if len(prevObsStack) == 0 {
				resetTrip()
				continue
			}

			lastAccepted := prevObsStack[len(prevObsStack)-1]
			breakDuration := gps.Time().Sub(lastAccepted.gps.Time()).Seconds()
			if util.Gt(breakDuration, maxSecondsToHMMBreak) {

				// if the break exceeds 180 seconds long, split the data into separate trips
				finishTrip() // split trip
				viterbiDecoder.InitForwardProb(t, states, emissionProbMatrix)
				if viterbiDecoder.IsHMMBroken() {
					resetTrip()
					continue
				}
				initialized = true
				pushAcceptedObsToTrip(t, gps, gpsCands)
				continue
			}

			// when break is detected in timesteps t and t+1, remove prevObs and and curObs
			prevObsStack = prevObsStack[:len(prevObsStack)-1]
			if len(prevObsStack) == 0 {
				resetTrip()
				continue
			}

			stepBackFrame := prevObsStack[len(prevObsStack)-1]
			viterbiDecoder.StepBack(stepBackFrame.observationId)

			prevObs = stepBackFrame.gps
			prevCands = stepBackFrame.candidates
		} else {
			pushAcceptedObsToTrip(t, gps, gpsCands)
		}
	}

	path = append(path, viterbiDecoder.RetrieveMostLikelyStateSequence()...)

	mapMatchingResult := make([]*da.MatchedGPSPoint, 0, len(path))

	var (
		prevMatchedCand  *ma.Candidate
		prevGps          *da.GPSPoint
		matchedRoutePath []da.Coordinate
	)

	for i, p := range path {
		s := stateData[p.GetStateId()]
		gps := gpsTraj[p.GetObservationId()]

		stateEdgeId := s.EdgeId()
		e := h.graph.GetOutEdge(stateEdgeId)
		tail := h.graph.GetVertex(h.graph.GetTailOfOutedge(e.GetEdgeId()))
		head := h.graph.GetVertex(e.GetHead())
		eInitialBearing := geo.BearingTo(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
		streetName := h.graph.GetStreetName(e.GetEdgeId())
		matchedSegment := da.NewMatchedGPSPoint(gps, stateEdgeId, s.GetProjectedCoord(), eInitialBearing,
			uint32(p.GetObservationId()), streetName)
		mapMatchingResult = append(mapMatchingResult, matchedSegment)

		if i == 0 {
			prevMatchedCand = s
			prevGps = gps
			continue
		}

		allowUTurn := allowUTurns[p.GetObservationId()]

		sp := h.newPhantomNodeFromCandidate(prevMatchedCand)
		tp := h.newPhantomNodeFromCandidate(s)
		_, sameSegment := h.handleSameSourceDestinationSegment(sp, tp)

		if !sameSegment {
			route, ok := h.handleDestinationSegmentNextToSourceSegment(sp, tp, allowUTurn)
			if !ok {
				deltaTime := gps.Time().Sub(prevGps.Time()).Seconds()
				route, _ = h.shortestPathDistance(sp, tp, deltaTime, true)
			}

			matchedRoutePath = appendRoutePath(matchedRoutePath, route.path)

			prevMatchedCand = s
			prevGps = gps
		}
	}

	return mapMatchingResult, matchedRoutePath
}

// buildAllowUturnSlice. allowUTurns slice. allowUTurns[i] true jika gpsTraj[i] kemungkinan jadi titik u-turn
func (h *HMM) buildAllowUturnSlice(gpsTraj []*da.GPSPoint) []bool {
	n := len(gpsTraj)
	allowUTurns := make([]bool, n)

	for i := 2; i < n; i++ {
		doublePrev := gpsTraj[i-2]
		prev := gpsTraj[i-1]
		cur := gpsTraj[i]

		doubleDist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(doublePrev.Lat(), doublePrev.Lon(), prev.Lat(), prev.Lon()))
		prevDist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(prev.Lat(), prev.Lon(), cur.Lat(), cur.Lon()))

		uTurnPossible := false

		prevBearing := geo.ComputeInitialBearing(doublePrev.Lat(), doublePrev.Lon(), prev.Lat(), prev.Lon())
		delta := geo.ComputeRelativeBearing(prev.Lat(), prev.Lon(), cur.Lat(), cur.Lon(), prevBearing)
		absDelta := math.Abs(delta)
		deltaDegree := util.RadiansToDegree(absDelta)
		if util.Gt(deltaDegree, uTurnMinTurnAngle) && util.Gt(doubleDist, uTurnMinDistMeters) && util.Gt(prevDist, uTurnMinDistMeters) {
			uTurnPossible = true
		}

		allowUTurns[i] = uTurnPossible
	}

	return allowUTurns
}

func (h *HMM) projectAllGpsWithRadiuses(gpsTraj []*da.GPSPoint, gpsRadiusesM []float64) ([][]*ma.Candidate, int) {
	allCandidates := make([][]*ma.Candidate, 0, len(gpsTraj))
	stateId := 0
	for i, gps := range gpsTraj {
		searchRadiusM := gpsRadiusesM[i] * candidateSearchRadiusMultiplier
		searchRadiusKM := util.MeterToKilometer(searchRadiusM)

		nearbyArcs := h.rt.SearchWithinRadius(gps.Lat(), gps.Lon(), searchRadiusKM, 3)

		candidates := make([]*ma.Candidate, 0, len(nearbyArcs))

		for _, arcEndpoint := range nearbyArcs {
			eLength := h.re.GetSegmentLength(arcEndpoint, true)
			cand := ma.NewCandidate(arcEndpoint, 0, eLength)
			candidates = append(candidates, cand)
		}

		h.projectAllCandidates(gps, candidates)
		candidates = filterAndSortCandidates(candidates, searchRadiusM)

		for _, cand := range candidates {
			cand.SetStateId(stateId)
			stateId++
		}

		allCandidates = append(allCandidates, candidates)
	}

	return allCandidates, stateId
}

func filterAndSortCandidates(candidates []*ma.Candidate, maxDistanceM float64) []*ma.Candidate {
	filtered := candidates[:0]
	for _, cand := range candidates {
		if util.Le(cand.GetDist(), maxDistanceM) {
			filtered = append(filtered, cand)
		}
	}

	sort.Slice(filtered, func(i, j int) bool {
		return filtered[i].GetDist() < filtered[j].GetDist()
	})

	n := len(filtered)

	filtered = filtered[:min(n, spatialindex.FILTERED_CANDIDATES_MAP_MATCHING)]

	return filtered
}

type calcTransitionProbParam struct {
	measurementDist   float64
	deltaTime         float64
	prevCand, curCand *ma.Candidate
	allowUTurn        bool
}

func newCalcTransitionProbParam(measurementDist, deltaTime float64, prevCand, curCand *ma.Candidate, allowUTurn bool) calcTransitionProbParam {
	return calcTransitionProbParam{measurementDist: measurementDist, deltaTime: deltaTime, prevCand: prevCand, curCand: curCand, allowUTurn: allowUTurn}
}
func (ct *calcTransitionProbParam) getMeasurementDist() float64 {
	return ct.measurementDist
}

func (ct *calcTransitionProbParam) getDeltaTime() float64 {
	return ct.deltaTime
}

func (ct *calcTransitionProbParam) getPrevCand() *ma.Candidate {
	return ct.prevCand
}

func (ct *calcTransitionProbParam) getCurCand() *ma.Candidate {
	return ct.curCand
}

func (ct *calcTransitionProbParam) getAllowUTurn() bool {
	return ct.allowUTurn
}

type transitionWithProb struct {
	transition     ma.Transition
	transitionProb float64
}

func newTransitionWithProb(tr ma.Transition, transitionProb float64) transitionWithProb {
	return transitionWithProb{transition: tr, transitionProb: transitionProb}
}

func (tr *transitionWithProb) getTransition() ma.Transition {
	return tr.transition
}

func (tr *transitionWithProb) getTransitionProb() float64 {
	return tr.transitionProb
}

func computeEmissionLogProb(obsStateDist, gpsRadiusM float64) float64 {
	if util.Eq(gpsRadiusM, 0) {
		// gpsRadiusM not set
		gpsRadiusM = sigmaZ
	}
	sigma := gpsRadiusM
	return -0.5*(math.Log(2.0*math.Pi)+(obsStateDist/sigma)*(obsStateDist/sigma)) - math.Log(sigma)
}

func computeTransitionLogProb(routeDistance, measurementDistance float64) float64 {
	obsStateDiff := math.Abs(routeDistance - measurementDistance)

	return -math.Log(beta) - obsStateDiff/beta
}

func (h *HMM) calcTransitionProb(param calcTransitionProbParam) transitionWithProb {
	prevCand := param.getPrevCand()
	curCand := param.getCurCand()
	tr := ma.NewTransition(prevCand.GetStateId(), curCand.GetStateId())
	invalidTransition := newTransitionWithProb(tr, ma.InvalidLogProb)

	deltaTime := param.getDeltaTime()

	maxDist := MaxSpeedMS * deltaTime

	sp := h.newPhantomNodeFromCandidate(prevCand)
	tp := h.newPhantomNodeFromCandidate(curCand)

	allowUTurn := param.getAllowUTurn()

	route, ok := h.handleSameSourceDestinationSegment(sp, tp)
	if util.Eq(route.distance, pkg.INF_WEIGHT) {
		return invalidTransition
	}
	if !ok {
		route, ok = h.handleDestinationSegmentNextToSourceSegment(sp, tp, allowUTurn)
	}
	if !ok {
		route, ok = h.shortestPathDistance(sp, tp, deltaTime, false)
	}

	if !ok {
		return invalidTransition
	}

	distDiff := math.Abs(route.distance - param.getMeasurementDist())
	if util.Ge(distDiff, maxDist) || util.Ge(distDiff, maxTransitionDist) {
		// if the difference between routeDistance and measurementDistance exceeds 2000 meters or more, we assign probablity of zero
		return invalidTransition
	}

	return newTransitionWithProb(tr, computeTransitionLogProb(route.distance, param.getMeasurementDist()))
}

type transitionRoute struct {
	distance   float64
	travelTime float64
	path       da.Coordinates
}

func (h *HMM) newPhantomNodeFromCandidate(cand *ma.Candidate) da.PhantomNode {
	edgeId := cand.EdgeId()
	inEdgeId := h.graph.GetEntryIdOfOutEdge(edgeId)

	forwardDistance := cand.GetDistanceFromHead()
	reverseDistance := cand.GetDistanceFromTail()

	forwardTravelTime := cand.GetTravelTimeFromHead()
	reverseTravelTime := cand.GetTravelTimeFromTail()

	forwardGeometry, reverseGeometry := h.candidatePhantomGeometries(cand)

	return da.NewPhantomNode(
		cand.GetProjectedCoord(),
		forwardTravelTime,
		reverseTravelTime,
		edgeId,
		inEdgeId,
		forwardDistance,
		reverseDistance,
		forwardGeometry,
		reverseGeometry,
	)
}

func (h *HMM) candidatePhantomGeometries(cand *ma.Candidate) ([]da.Coordinate, []da.Coordinate) {
	eGeometry := h.graph.GetEdgeGeometry(cand.EdgeId())
	if len(eGeometry) < 2 {
		return []da.Coordinate{}, []da.Coordinate{}
	}

	lastIndex := h.projectedSegmentIndex(cand.GetProjectedCoord(), eGeometry)
	forwardGeometry := append([]da.Coordinate{}, eGeometry[lastIndex+1:]...)
	reverseGeometry := append([]da.Coordinate{}, eGeometry[:lastIndex+1]...)

	return forwardGeometry, reverseGeometry
}

func (h *HMM) projectedSegmentIndex(projectedCoord da.Coordinate, eGeometry []da.Coordinate) int {
	lastIndex := 0
	minDist := pkg.INF_WEIGHT

	for i := 0; i < len(eGeometry)-1; i++ {
		projectedPoint := geo.ProjectPointOnSegment(eGeometry[i], eGeometry[i+1], projectedCoord)
		dist := geo.CalculateEuclideanDistMercatorProj(
			projectedPoint.GetLat(), projectedPoint.GetLon(),
			projectedCoord.GetLat(), projectedCoord.GetLon(),
		)
		if util.Lt(dist, minDist) {
			minDist = dist
			lastIndex = i
		}
	}

	return lastIndex
}

func (h *HMM) handleSameSourceDestinationSegment(sp, tp da.PhantomNode) (transitionRoute, bool) {
	if sp.GetOutEdgeId() != tp.GetOutEdgeId() {
		return transitionRoute{}, false
	}

	distance := tp.GetReverseDistance() - sp.GetReverseDistance()

	if util.Lt(distance, -sigmaZ) {
		// backward movement
		return transitionRoute{}, false
	}

	distance = math.Abs(distance)

	travelTime := h.re.GetWeightFromLength(sp.GetOutEdgeId(), true, distance)

	return transitionRoute{distance: distance, travelTime: travelTime}, true
}

func (h *HMM) isReversedEdge(e1, e2 da.Index) bool {
	e1Tail := h.graph.GetTailOfOutedge(e1)
	e1Head := h.graph.GetHeadOfOutEdge(e1)

	e2Tail := h.graph.GetTailOfOutedge(e2)
	e2Head := h.graph.GetHeadOfOutEdge(e2)

	return e1Tail == e2Head && e1Head == e2Tail
}

func (h *HMM) handleDestinationSegmentNextToSourceSegment(sp, tp da.PhantomNode, allowUTurn bool) (transitionRoute, bool) {
	sourceHead := h.graph.GetHeadOfOutEdge(sp.GetOutEdgeId())
	destinationTail := h.graph.GetTailOfOutedge(tp.GetOutEdgeId())
	if sourceHead != destinationTail {
		return transitionRoute{}, false
	}

	path := make([]da.Coordinate, 0, len(sp.GetForwardGeometry())+len(tp.GetReverseGeometry())+2)
	path = appendRoutePath(path, []da.Coordinate{sp.GetSnappedCoord()})
	path = appendRoutePath(path, sp.GetForwardGeometry())
	path = appendRoutePath(path, tp.GetReverseGeometry())
	path = appendRoutePath(path, []da.Coordinate{tp.GetSnappedCoord()})

	nextSegmentDur := sp.GetForwardDistance() + tp.GetReverseDistance()
	nextSegmentDist := sp.GetForwardDistance() + tp.GetReverseDistance()

	if !allowUTurn {
		return transitionRoute{
			distance:   nextSegmentDist,
			travelTime: nextSegmentDur,
			path:       path,
		}, true
	}

	// allowUTurn=true
	minSegmentDur := pkg.INF_WEIGHT
	minSegmentDist := pkg.INF_WEIGHT

	sOutEdgeId := sp.GetOutEdgeId()
	tOutEdgeId := tp.GetOutEdgeId()
	if h.isReversedEdge(sOutEdgeId, tOutEdgeId) {
		/*
			buat handle case:
			oi = gps measurement ke-i (sorted by timestamp)
			cif = candidate dari oi yang forward road segment (kalau osm way dari road segmentnya oneway=false )
			cir =  ---------------------- reverse road segment

					o1       	  o2                    o3
					    	o4

			e1Tail--c1f-----c4r---c2f-------------------c3f------> e1Head

			e2Head<-c1r-----c4f----c2r-----------------c3r------- e2Tail

			e1 dan e2 adalah satu osm way yang oneway=false (or two-way).

			o4 dibelakang o3 yang mungkin adalah u-turn dari o2->o3->o4...

			atau untuk phantom node yang ,mungkin dilewatin driver nya: c1f->c2f->c3r->c4f
			note: c3r bukan c4f, karena driver langsung u-turn dari c2f-c3r->c4f (phantomnode c3f dan c3r didalam osm way oneway=false)

			ketika  c2f->c3r: kita bisa set routeDistance dari c2f->c3r dengan = min(e1.length - c2f.ReverseDist - c3r.ReverseDist, nextSegmentDist)

			allowUTurn == True ketika current gps observation (yang kandidate phantomnode nya tp) di titik u-turn (misal o3 di example diatas)
			ketika c3r-c4r allowUTurn = false karena o4 bukan titik u-turn (udah return duluan pakai logic diatas)

			ketika c2r->c3f (karena c2r dan c3f saling reverse dan o3 titik u-turn): karena e1.length - c2r.ReverseDist - c3f.ReverseDist < 0 ,kita set routeDist dari c2r->c3f = INF_WEIGHT
			dan ambil minimum dari min(routeDist dari c2r->c3f, nextSegmentDist)

		*/

		sameSegmentDur := pkg.INF_WEIGHT
		sameSegmentDist := pkg.INF_WEIGHT

		eLength := h.re.GetSegmentLength(sOutEdgeId, true)

		sameSegmentRevDist := eLength - sp.GetReverseDistance() - tp.GetReverseDistance()
		if util.Gt(sameSegmentRevDist, 0) {
			sameSegmentDist = sameSegmentRevDist
			eDuration := h.re.GetWeightSeconds(sOutEdgeId, true)
			sameSegmentDur = eDuration - sp.GetReverseTravelTime() - tp.GetReverseTravelTime()
		}

		minSegmentDist = min(sameSegmentDist, nextSegmentDist)
		minSegmentDur = min(sameSegmentDur, nextSegmentDur)
	}

	return transitionRoute{
		distance:   minSegmentDist,
		travelTime: minSegmentDur,
		path:       path,
	}, true
}

func (h *HMM) shortestPathDistance(sp, tp da.PhantomNode, deltaTimeSeconds float64, addPath bool) (transitionRoute, bool) {
	if h.re == nil {
		return transitionRoute{}, false
	}

	longerDist := MaxSpeedMS * deltaTimeSeconds
	spCoord := sp.GetSnappedCoord()
	tpCoord := tp.GetSnappedCoord()

	dist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(spCoord.GetLat(), spCoord.GetLon(), tpCoord.GetLat(), tpCoord.GetLon()))
	longerDist += dist
	longerDist *= multiplyDistWith

	longerDur := longerDist / MaxSpeedMS

	crpQuery := routing.NewCRPBidirectionalSearch(h.re, 1.0)
	defer crpQuery.Done()
	crpQuery.SetMaxSearchRadiusSecs(longerDur)
	weight, edgePath, found := crpQuery.ShortestPathSearch(sp, tp)
	if !found {
		return transitionRoute{}, false
	}
	travelTime := h.re.GetCostFunction().WeightToSeconds(weight)

	if !addPath {
		routeDistance := sp.GetForwardDistance() + tp.GetReverseDistance()
		for _, edgeId := range edgePath {
			routeDistance += h.re.GetSegmentLength(edgeId, true)
		}

		return transitionRoute{distance: routeDistance, travelTime: travelTime}, true
	}

	routeDistance := sp.GetForwardDistance() + tp.GetReverseDistance()

	finalPath, totalDistance := h.re.GetEdgePath(edgePath, 0)
	routeDistance += totalDistance

	path := make([]da.Coordinate, 0, len(sp.GetForwardGeometry())+len(*finalPath)+len(tp.GetReverseGeometry())+2)
	path = appendRoutePath(path, []da.Coordinate{sp.GetSnappedCoord()})
	path = appendRoutePath(path, sp.GetForwardGeometry())
	path = appendRoutePath(path, *finalPath)
	path = appendRoutePath(path, tp.GetReverseGeometry())
	path = appendRoutePath(path, []da.Coordinate{tp.GetSnappedCoord()})

	h.re.PutCoordsToPool(finalPath)

	return transitionRoute{distance: routeDistance, travelTime: travelTime, path: path}, true
}

func appendRoutePath(path []da.Coordinate, segment []da.Coordinate) []da.Coordinate {
	for _, coord := range segment {
		if len(path) == 0 || !da.IsSameCoordinate(path[len(path)-1], coord) {
			path = append(path, coord)
		}
	}
	return path
}

func (h *HMM) projectAllCandidates(gps *da.GPSPoint, candidates []*ma.Candidate) {
	for _, cand := range candidates {

		gpsCoord := da.NewCoordinate(gps.Lat(), gps.Lon())

		eGeometry := h.graph.GetEdgeGeometry(cand.EdgeId())

		candEdgeBearing := 0.0

		var (
			minDist, minDistr  = math.MaxFloat64, math.MaxFloat64
			bestProjectedPoint da.Coordinate
		)

		cumLength := 0.0

		for i := 0; i < len(eGeometry)-1; i++ {
			tail := eGeometry[i]
			head := eGeometry[i+1]
			projectedPoint := geo.ProjectPointOnSegment(
				tail,
				head,
				gpsCoord,
			)

			dist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(
				projectedPoint.GetLat(), projectedPoint.GetLon(),
				gpsCoord.GetLat(), gpsCoord.GetLon(),
			))

			tailToProjectedDist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(
				tail.GetLat(), tail.GetLon(),
				projectedPoint.GetLat(), projectedPoint.GetLon(),
			))

			distr := cumLength + tailToProjectedDist

			if dist < minDist {
				minDist = dist
				minDistr = distr
				bestProjectedPoint = projectedPoint
				eInitialBearing := geo.BearingTo(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
				candEdgeBearing = eInitialBearing
			}

			cumLength += util.KilometerToMeter(geo.CalculateGreatCircleDistance(
				tail.GetLat(), tail.GetLon(),
				head.GetLat(), head.GetLon(),
			))

		}

		cand.SetProjectedCoord(bestProjectedPoint.GetLat(), bestProjectedPoint.GetLon())
		cand.SetDist(minDist)
		cand.SetDistr(minDistr)
		cand.SetDistanceFromTail(minDistr)

		edgeLength := h.re.GetSegmentLength(cand.EdgeId(), true)
		distanceFromHead := max(edgeLength-minDistr, 0)
		cand.SetDistanceFromHead(distanceFromHead)

		cand.SetTravelTimeFromTail(h.re.GetWeightFromLength(cand.EdgeId(), true, minDistr))
		cand.SetTravelTimeFromHead(h.re.GetWeightFromLength(cand.EdgeId(), true, distanceFromHead))

		cand.SetEdgeBearing(candEdgeBearing)
	}
}
