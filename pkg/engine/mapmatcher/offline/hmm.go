package offline

import (
	"math"
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type HMM struct {
	graph  *da.Graph
	re     *engine.Engine
	rt     *spatialindex.Rtree
	logger *zap.Logger

	lm *landmark.Landmark
}

func NewHiddenMarkovModelMapMatching(graph *da.Graph, re *engine.Engine,
	rt *spatialindex.Rtree,

	lm *landmark.Landmark) *HMM {
	return &HMM{
		graph: graph,
		re:    re,
		rt:    rt,
		lm:    lm,
	}
}

/*
masih salah
todo: benerin
implementation of:
Newson, P. & Krumm, J., (2009). "Hidden Markov map matching through noise and
sparseness". Proceedings of the 17th ACM SIGSPATIAL International Conference
on Advances in Geographic Information Systems (GIS ‘09), pp.336–343.
*/
func (h *HMM) MapMatch(gpsTraj []*da.GPSPoint) []*da.MatchedGPSPoint {

	candidates, numberOfStates := h.projectAllGps(gpsTraj)

	viterbiDecoder := newViterbi()

	stateData := make([]*ma.Candidate, numberOfStates)

	prevObs := gpsTraj[0]
	prevCands := candidates[0]
	for i := 0; i < len(gpsTraj); i++ {
		emissionProbMatrix := make(map[int]float64)
		states := make([]int, 0)
		gps := gpsTraj[i]

		transitionProbMatrix := make(map[transition]float64)

		for j := 0; j < len(candidates[i]); j++ {
			cand := candidates[i][j]
			candProjection := cand.GetProjectedCoord()

			gpsToCandDist := util.KilometerToMeter(geo.CalculateHaversineDistance(
				candProjection.GetLat(), candProjection.GetLon(),
				gps.Lat(), gps.Lon(),
			))

			emissionProb := computeEmissionLogProb(gpsToCandDist)

			emissionProbMatrix[cand.GetStateId()] = emissionProb

			states = append(states, cand.GetStateId())

			stateData[cand.GetStateId()] = cand
		}

		gpsCands := candidates[i]

		if i == 0 {
			viterbiDecoder.initForwardProb(i, states, emissionProbMatrix)

		} else {
			measurementDist := util.KilometerToMeter(
				geo.CalculateHaversineDistance(prevObs.Lat(), prevObs.Lon(), gps.Lat(), gps.Lon()),
			)

			workers := concurrent.NewWorkerPool[calcTransitionProbParam, transitionWithProb](1,
				len(gpsCands)*len(prevCands))

			for j := 0; j < len(prevCands); j++ {
				for k := 0; k < len(gpsCands); k++ {
					prevState := prevCands[j]
					currState := gpsCands[k]
					workers.AddJob(newCalcTransitionProbParam(
						measurementDist, prevState, currState,
					))
				}
			}

			workers.Close()
			workers.Start(h.calcTransitionProb)

			workers.Wait()

			for transition := range workers.CollectResults() {
				if da.Eq(transition.transitionProb, INVALIDLOGPROB) {
					continue
				}
				transitionProbMatrix[transition.transition] = transition.transitionProb
			}

			viterbiDecoder.forwardStep(i, states, emissionProbMatrix, transitionProbMatrix)

			if viterbiDecoder.isHMMBroken() {
				break
			}
		}

		prevObs = gps
		prevCands = gpsCands
	}

	path := viterbiDecoder.retrieveMostLikelyStateSequence()

	mapMatchingResult := make([]*da.MatchedGPSPoint, 0, len(path))
	for _, p := range path {
		s := stateData[p.getStateId()]
		gps := gpsTraj[p.getObservationId()]

		e := h.graph.GetOutEdge(s.EdgeId())
		tail := h.graph.GetVertex(h.graph.GetTailOfOutedge(e.GetEdgeId()))
		head := h.graph.GetVertex(e.GetHead())
		eInitialBearing := geo.BearingTo(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
		matchedSegment := da.NewMatchedGPSPoint(gps, s.EdgeId(), s.GetProjectedCoord(), eInitialBearing)
		mapMatchingResult = append(mapMatchingResult, matchedSegment)
	}

	return mapMatchingResult
}

type calcTransitionProbParam struct {
	measurementDist   float64
	prevCand, curCand *ma.Candidate
}

func newCalcTransitionProbParam(measurementDist float64, prevCand, curCand *ma.Candidate) calcTransitionProbParam {
	return calcTransitionProbParam{measurementDist: measurementDist, prevCand: prevCand, curCand: curCand}
}
func (ct *calcTransitionProbParam) getMeasurementDist() float64 {
	return ct.measurementDist
}

func (ct *calcTransitionProbParam) getPrevCand() *ma.Candidate {
	return ct.prevCand
}

func (ct *calcTransitionProbParam) getCurCand() *ma.Candidate {
	return ct.curCand
}

type transitionWithProb struct {
	transition     transition
	transitionProb float64
}

func newTransitionWithProb(tr transition, transitionProb float64) transitionWithProb {
	return transitionWithProb{transition: tr, transitionProb: transitionProb}
}

func (tr *transitionWithProb) getTransition() transition {
	return tr.transition
}

func (tr *transitionWithProb) getTransitionProb() float64 {
	return tr.transitionProb
}

const (
	sigmaZ                    = 4.07
	beta                      = 0.0009
	maxTransitionDist         = 1000.0 // 1km
	INVALIDLOGPROB    float64 = -1e34
)

func computeEmissionLogProb(obsStateDist float64) float64 {
	return math.Log(1.0/(math.Sqrt(2*math.Pi)*sigmaZ)) + (-0.5 * math.Pow(obsStateDist/sigmaZ, 2))
}

func computeTransitionLogProb(routeDistance, measurementDistance float64) float64 {
	obsStateDiff := math.Abs(measurementDistance - routeDistance)
	return math.Log(1.0/beta) - (obsStateDiff / beta)
}

func (h *HMM) calcTransitionProb(param calcTransitionProbParam) transitionWithProb {
	msDist := param.getMeasurementDist()
	prevCand, curCand := param.getPrevCand(), param.getCurCand()

	handleRelaxOutEdge := func(e *da.OutEdge) float64 {
		// misal splitted outEdge nya (u,v) -> (u, p1) , (p1, p2) , (p2, v)
		// u -> p1 -> p2 -> v

		if e.GetEdgeId() == prevCand.EdgeId() {
			return prevCand.GetTravelTimeFromTail()
		}
		return 0
	}

	handleRelaxInEdge := func(e *da.InEdge) float64 {
		// misal splitted inEdge nya (u,v) -> (u, p1) , (p1, p2) , (p2, v)
		// u -> p1 -> p2 -> v

		if e.GetEdgeId() == curCand.EdgeId() {
			return curCand.GetTravelTimeFromHead()
		}
		return 0
	}

	as := prevCand.EdgeId()
	at := curCand.EdgeId()

	tr := newTransition(prevCand.GetStateId(), curCand.GetStateId())
	if as == at {
		routeDist := curCand.GetDistanceFromTail() - prevCand.GetDistanceFromTail()
		if routeDist < 0 {
			return newTransitionWithProb(tr, INVALIDLOGPROB)
		}
		return newTransitionWithProb(tr, computeTransitionLogProb(routeDist,
			msDist))
	}

	neighbor := false

	prevOutEdge := h.graph.GetOutEdge(as)
	prevHead := prevOutEdge.GetHead()
	h.graph.ForOutEdgesOf(prevHead, da.Index(prevOutEdge.GetEntryPoint()), func(e *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {
		if e.GetEdgeId() == at {
			neighbor = true
		}
	})

	if neighbor {
		routeDist := prevCand.GetDistanceFromHead() + curCand.GetDistanceFromTail()

		return newTransitionWithProb(tr, computeTransitionLogProb(routeDist,
			msDist))
	}

	crpQuery := routing.NewCRPALTBidirectionalSearch(h.re.GetRoutingEngine(), 1.0, h.lm)
	crpQuery.OnMapMatching(handleRelaxOutEdge, handleRelaxInEdge)

	_, routeDist, _, _, found := crpQuery.ShortestPathSearch(as, at)

	if !found || math.Abs(routeDist-msDist) >= maxTransitionDist {
		return newTransitionWithProb(tr, INVALIDLOGPROB)
	}

	//  misal splitted outEdge dari as: -> (s, p1) , (p1, p2) , (p2, v)
	//  misal splitted outEdge dari at: -> (w, p3) , (p3, p4) , (p4, t)
	// routeDist adalah distance (sum of edge lengths, bukan travel time) dari shortest path (dihitung pakai priority travel time) dari s ke t
	// projectionPoint dari prevCand ada di p1, projectionPoint dari curCand ada di p4
	// untuk dapetin routeDist dari p1 ke p4:
	// karena routeDist hasil crp query hanya memberikan route distance dari v ke w
	// kita harus tambahkan routeDist dengan:
	routeDist += prevCand.GetDistanceFromHead()
	routeDist += curCand.GetDistanceFromTail()
	// kok negatif distancefromhead nya

	return newTransitionWithProb(tr, computeTransitionLogProb(routeDist,
		msDist))
}

func (h *HMM) projectAllGps(gpsTraj []*da.GPSPoint) ([][]*ma.Candidate, int) {
	allCandidates := make([][]*ma.Candidate, 0, len(gpsTraj))
	stateId := 0
	for _, gps := range gpsTraj {
		nearbyArcs := h.rt.SearchWithinRadius(gps.Lat(), gps.Lon(), 0.07)

		candidates := make([]*ma.Candidate, 0, len(nearbyArcs))

		sumLength := 0.0
		for _, arcEndpoint := range nearbyArcs {
			eLength := h.graph.GetOutEdge(arcEndpoint.GetExitId()).GetSimplifiedLength()
			sumLength += eLength
		}

		for _, arcEndpoint := range nearbyArcs {

			eLength := h.graph.GetOutEdge(arcEndpoint.GetExitId()).GetSimplifiedLength()

			cand := ma.NewCandidate(arcEndpoint.GetId(), eLength/sumLength, eLength)
			cand.SetStateId(stateId)
			candidates = append(candidates, cand)
			stateId++
		}

		h.projectAllCandidates(gps, candidates)

		allCandidates = append(allCandidates, candidates)
	}

	numberOfStates := h.splitEdges(gpsTraj, allCandidates)

	return allCandidates, numberOfStates
}

// splitEdges.
// split edge (u,v) menjadi (u, projectedPoint_1),  (projectedPoint_2,projectedPoint_3), ...., (projectedPoint_n , v)
// projectedPoint_i adalah proyeksi gps ke edge (u,v)
// turn cost dari edge lain ke edge (u,v) cuma diset ke new edge (u, projectedPoint_1)
// caranya dengan set exitPoint dari edge  (u, projectedPoint_1) sama dengan exitPoint dari edge (u, v)
// dan set entryPoint dari (projectedPoint_n , v) ke entryPoint dari edge (u, v)
// untuk turn costs dari semua edges diatara edge pertama dan terakhir bernilai 0
// split edge ini digunakan di computeTransitionProbability dimana untuk menghitung transition probability
// kita membutuhkan great circle distance dari projected point (ke nearby edges) dari suatu gps point ke projected point (ke nearby edges) dari gps point lainnya
func (h *HMM) splitEdges(gpsTraj []*da.GPSPoint, candidates [][]*ma.Candidate) int {
	flattenCandidates := make([]*ma.Candidate, 0, len(candidates)*2)

	for i := 0; i < len(gpsTraj); i++ {
		for j := 0; j < len(candidates[i]); j++ {
			flattenCandidates = append(flattenCandidates, candidates[i][j])
		}
	}

	numberOfStates := len(flattenCandidates)

	edgeCandidatesMap := make(map[da.Index][]*ma.Candidate)
	for i := 0; i < len(flattenCandidates); i++ {
		cand := flattenCandidates[i]
		if _, ok := edgeCandidatesMap[cand.EdgeId()]; !ok {
			edgeCandidatesMap[cand.EdgeId()] = make([]*ma.Candidate, 0)
		}
		edgeCandidatesMap[cand.EdgeId()] = append(edgeCandidatesMap[cand.EdgeId()], cand)
	}

	removedOutEdgeIds := make([]da.Index, 0, len(edgeCandidatesMap))
	removedInEdgeIds := make([]da.Index, 0, len(edgeCandidatesMap))

	for eId, eCands := range edgeCandidatesMap {
		removedOutEdgeIds = append(removedOutEdgeIds, eId)
		inEdge := h.graph.GetInEdgeOfOutEdge(eId)
		removedInEdgeIds = append(removedInEdgeIds, inEdge.GetEdgeId())

		e := h.graph.GetOutEdge(eId)

		tail := h.graph.GetTailOfOutedge(eId)

		tailVertex := h.graph.GetVertex(tail)

		sort.Slice(eCands, func(i, j int) bool {
			a := eCands[i]
			b := eCands[j]
			distA := geo.CalculateHaversineDistance(tailVertex.GetLat(), tailVertex.GetLon(),
				a.GetProjectedCoord().GetLat(), a.GetProjectedCoord().GetLon())
			distB := geo.CalculateHaversineDistance(tailVertex.GetLat(), tailVertex.GetLon(),
				b.GetProjectedCoord().GetLat(), b.GetProjectedCoord().GetLon())

			if distA < distB {
				return true
			} else {
				return false
			}
		})

		eSpeed := h.re.GetRoutingEngine().GetMaxSpeed(e)

		// add virtual edges: (u, projectedPoint_1),  (projectedPoint_2,projectedPoint_3), ....,
		for i := 0; i < len(eCands); i++ {
			snap := eCands[i]
			distFromTail := snap.GetDistr()
			snap.SetDistanceFromTail(distFromTail)
			snap.SetTravelTimeFromTail(distFromTail / eSpeed)
			snap.SetDistanceFromHead(e.GetSimplifiedLength() - distFromTail)
			snap.SetTravelTimeFromHead((e.GetSimplifiedLength() - distFromTail) / eSpeed)
		}
	}

	return numberOfStates
}

func (h *HMM) projectAllCandidates(gps *da.GPSPoint, candidates []*ma.Candidate) {
	for _, cand := range candidates {

		gpsCoord := da.NewCoordinate(gps.Lat(), gps.Lon())

		eGeometry := h.graph.GetEdgeGeometry(cand.EdgeId())

		var (
			minDist, minDistr  float64 = math.MaxFloat64, math.MaxFloat64
			bestProjectedPoint geo.Coordinate
		)

		cumLength := 0.0

		for i := 0; i < len(eGeometry)-1; i++ {
			tail := eGeometry[i]
			head := eGeometry[i+1]
			projectedPoint := geo.ProjectPointToLineCoord(
				tail.ToGeoCoordinate(),
				head.ToGeoCoordinate(),
				gpsCoord.ToGeoCoordinate(),
			)

			dist := util.KilometerToMeter(geo.CalculateHaversineDistance(
				projectedPoint.GetLat(), projectedPoint.GetLon(),
				gpsCoord.GetLat(), gpsCoord.GetLon(),
			))

			tailToProjectedDist := util.KilometerToMeter(geo.CalculateHaversineDistance(
				tail.GetLat(), tail.GetLon(),
				projectedPoint.GetLat(), projectedPoint.GetLon(),
			))

			distr := cumLength + tailToProjectedDist

			if dist < minDist {
				minDist = dist
				minDistr = distr
				bestProjectedPoint = projectedPoint
			}

			cumLength += util.KilometerToMeter(geo.CalculateHaversineDistance(
				tail.GetLat(), tail.GetLon(),
				head.GetLat(), head.GetLon(),
			))

		}

		cand.SetProjectedCoord(bestProjectedPoint.GetLat(), bestProjectedPoint.GetLon())
		cand.SetDist(minDist)
		cand.SetDistr(minDistr)
	}
}
