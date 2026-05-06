package online

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
implementation of:
[1] Taguchi, S., Koide, S. and Yoshimura, T. (2019) “Online Map Matching With Route
Prediction,” IEEE Transactions on Intelligent Transportation Systems, 20(1), pp.
338–347. Available at: https://doi.org/10.1109/TITS.2018.2812147.

ini buat golang webassembly (client-side real-time map matching). terinspirasi dari: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
lihat ./cmd/online_mapmatch_wasm/main.go
*/

type OnlineMapMatchMHTWasm struct {
	graph             *da.MapMatchingGraph
	rt                *spatialindex.Rtree
	initialSpeedMean  float64 // \overline{v}
	initialSpeedStd   float64 // \sigma_{v}
	posteriorThresold float64 // L_u
	gpsStd            float64
	accelerationStd   float64
	lp                float64 // L_p
	lc                float64 // L_c
	N                 *da.SparseMatrix[int]
}

func NewOnlineMapMatchMHTWasm(graph *da.MapMatchingGraph, rt *spatialindex.Rtree, initialSpeedMean, initialSpeedStd float64,
	posteriorThresold, gpsStd, lp, lc, accelerationStd float64,
	N *da.SparseMatrix[int]) *OnlineMapMatchMHTWasm {
	return &OnlineMapMatchMHTWasm{
		graph:             graph,
		rt:                rt,
		initialSpeedMean:  initialSpeedMean,
		initialSpeedStd:   initialSpeedStd,
		posteriorThresold: posteriorThresold,
		gpsStd:            gpsStd,
		lp:                lp,
		accelerationStd:   accelerationStd,
		lc:                lc,
		N:                 N,
	}
}

// OnlineMapMatch. perform online map matching using Multiple Hypothesis Technique
// speed in meter/s, arc length in meter, k is current time step (1-based)
// Algorithm 1 in ref[1]
// O( b^{d_p}), b=max outDegree of any vertex in the graph, d_p=maxVelocity*sampling interval/avgSegmentLength [1]
func (om *OnlineMapMatchMHTWasm) OnlineMapMatch(gps *da.GPSPoint, k int,
	candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64) {

	if k == 1 || len(candidates) == 0 {
		nearbyArcs := om.rt.SearchWithinRadius(gps.Lat(), gps.Lon(), om.lc, 3) // O(M)
		sumLength := 0.0
		startOfTheRoute := len(candidates) > 0

		var initialCandidate *ma.Candidate
		if startOfTheRoute {
			initialCandidate = candidates[0]
		}

		candidates = make([]*ma.Candidate, 0, len(nearbyArcs))

		for _, edgeId := range nearbyArcs {
			if !startOfTheRoute || (startOfTheRoute && edgeId == initialCandidate.EdgeId()) {
				arc := om.graph.GetOutEdge(edgeId)
				eLength := arc.GetLength()
				sumLength += eLength
			}
		}

		for _, edgeId := range nearbyArcs {
			if !startOfTheRoute || (startOfTheRoute && edgeId == initialCandidate.EdgeId()) {
				// biar candidates nya cuma first edgeId dari rute yang dipilih user (see https://github.com/lintang-b-s/navigatorx-crp-fe/blob/main/app/page.tsx).
				arc := om.graph.GetOutEdge(edgeId)
				eLength := arc.GetLength()

				candidates = append(candidates, ma.NewCandidate(edgeId, eLength/sumLength, eLength))
			}
		}

		om.projectAllCandidates(gps, candidates)

		matchedPoint, newcandidates, _ := om.filterLog(gps, candidates)
		return matchedPoint, newcandidates, om.initialSpeedMean, om.initialSpeedStd
	} else {
		var (
			speedMean, speedStd float64
		)
		if k == 2 {
			speedMean = om.initialSpeedMean
			speedStd = om.initialSpeedStd
		} else {
			speedMean = speedMeanK
			speedStd = speedStdK
		}

		if gps.GetDeadReckoning() {
			predDist := convertMeterToKilometer(speedMean * gps.DeltaTime()) // speed in m/s, deltatime in s
			predGpsLat, predGpsLon := geo.GetDestinationPoint(gps.Lat(), gps.Lon(), lastBearing, predDist)
			gps.SetCoord(predGpsLat, predGpsLon)
		}

		newCandidates := make([]*ma.Candidate, 0, len(candidates))

		for _, cand := range candidates {
			tau := make([]da.Index, 0, 5)
			tau = append(tau, cand.EdgeId())
			ptau := 1.0
			hpre := 1.0
			newCandidates = om.recur(newCandidates, cand.Weight(), tau, ptau, speedMean, hpre, speedStd, gps.DeltaTime())
		}
		speedMeanK, speedStdK = om.kalmanFilter(speedMean, speedStd, gps.Speed(), gps.DeltaTime())

		om.projectAllCandidates(gps, newCandidates)

		matchedPoint, newCandidatesFiltered, reset := om.filterLog(gps, newCandidates)

		if gps.GetDeadReckoning() {
			matchedPoint.SetPredictedGpsCoord(da.NewCoordinate(matchedPoint.GetMatchedCoord().GetLat(),
				matchedPoint.GetMatchedCoord().GetLon()))
		}

		if reset {
			return matchedPoint, make([]*ma.Candidate, 0), om.initialSpeedMean, om.initialSpeedStd
		}

		return matchedPoint, newCandidatesFiltered, speedMeanK, speedStdK
	}
}

// recur. prediction step of multiple hypothesis technique (compute prior)
// Algorithm 2 in ref[1]
func (om *OnlineMapMatchMHTWasm) recur(newCands []*ma.Candidate, w float64, tau []da.Index, ptau float64,
	speedMean, hpre, speedStd, deltaTime float64) []*ma.Candidate {
	hnew := om.computeHProb(tau, speedMean, speedStd, deltaTime)
	if w*hnew*ptau > om.lp {
		eNext := make([]da.Index, 0, 5)
		lastEdgeId := tau[len(tau)-1]
		lastEdge := om.graph.GetOutEdge(lastEdgeId)
		head := lastEdge.GetHead()

		om.graph.ForOutEdgesOf(head, func(eId, head da.Index, length float64, geometry []da.Coordinate) {
			eNext = append(eNext, eId)
		})

		for _, nextEdgeId := range eNext {
			tauPrime := make([]da.Index, len(tau))
			copy(tauPrime, tau)
			tauPrime = append(tauPrime, nextEdgeId)
			nj := len(eNext)
			ptauPrime := ptau * om.computEdgeTransitionProb(lastEdgeId, nextEdgeId, nj)
			hprePrime := hnew
			newCands = om.recur(newCands, w, tauPrime, ptauPrime, speedMean, hprePrime, speedStd, deltaTime)
		}
	}
	wprime := w * ptau * (hpre - hnew)
	var cnew *ma.Candidate
	for _, cand := range newCands {
		if cand.EdgeId() == tau[len(tau)-1] { // tau[len(tau)-1]=r_{k+1}
			cnew = cand
		}
	}
	if cnew == nil {
		lastTauEdge := om.graph.GetOutEdge(tau[len(tau)-1])
		newCands = append(newCands, ma.NewCandidate(tau[len(tau)-1], wprime,
			lastTauEdge.GetLength()))
	} else {
		cnew.SetWeight(cnew.Weight() + wprime)
	}
	return newCands
}

// filterLog. filter step of multiple hypothesis technique (compute posterior & pick most probable road segment)
//
//	normalization use log-sum-exp trick to avoid numerical underflow/overflow (https://gregorygundersen.com/blog/2020/02/09/log-sum-exp/)
func (om *OnlineMapMatchMHTWasm) filterLog(gps *da.GPSPoint, candidates []*ma.Candidate) (*da.MatchedGPSPoint, []*ma.Candidate, bool) {
	logAllCandWeights := make([]float64, 0, len(candidates))

	for _, cand := range candidates {
		obsLogLikelihood := om.computeObservationLogLikelihood(cand)
		logAllCandWeights = append(logAllCandWeights, math.Log(cand.Weight())+obsLogLikelihood)
	}

	allCandsWeightLSE := logSumExp(logAllCandWeights)
	sumPosterior := 0.0 // should \approx 1

	for i, cand := range candidates {
		obsLogLikelihood := om.computeObservationLogLikelihood(cand)
		posterior := (obsLogLikelihood + math.Log(cand.Weight())) - (allCandsWeightLSE)
		candidates[i] = ma.NewCandidate(cand.EdgeId(), math.Exp(posterior), cand.Length())
		candidates[i].SetProjectedCoord(cand.GetProjectedCoord().GetLat(), cand.GetProjectedCoord().GetLon())
		candidates[i].SetEdgeBearing(cand.GetEdgeBearing())
		sumPosterior += math.Exp(posterior)
	}

	// filter candidate yang memiliki weight < posteriorThreshold
	filteredCands := make([]*ma.Candidate, 0, len(candidates))
	for _, cand := range candidates {
		if math.IsNaN(cand.Weight()) {
			continue
		}
		if cand.Weight() > om.posteriorThresold {
			filteredCands = append(filteredCands, cand)
		}
	}

	// argmax posterior
	var matchedSegment *da.MatchedGPSPoint
	maxWeight := -1.0
	for _, cand := range filteredCands {
		if util.Gt(cand.Weight(), maxWeight) {
			projectedPointCoord := cand.GetProjectedCoord()
			eInitialBearing := cand.GetEdgeBearing()
			e := om.graph.GetOutEdge(cand.EdgeId())
			originalEdgeId := e.GetOriginalEdgeId()
			matchedSegment = da.NewMatchedGPSPoint(gps, originalEdgeId, projectedPointCoord, eInitialBearing)
			maxWeight = cand.Weight()
		}
	}

	if matchedSegment == nil {
		gpsPoint := da.NewGPSPoint(gps.Lat(), gps.Lon(), gps.Time(), gps.Speed(), gps.DeltaTime(),
			gps.GetDeadReckoning())
		invalidMatchedCoord := da.NewCoordinate(INVALID_LAT, INVALID_LON)
		matchedSegment = da.NewMatchedGPSPoint(gpsPoint, da.INVALID_EDGE_ID, invalidMatchedCoord, 0.0)
	}

	return matchedSegment, filteredCands, om.needToReset(gps, matchedSegment)
}

func (om *OnlineMapMatchMHTWasm) needToReset(gps *da.GPSPoint, matchedSegment *da.MatchedGPSPoint) bool {
	gpsLat, gpsLon := gps.Lat(), gps.Lon()
	matchCoord := matchedSegment.GetMatchedCoord()
	dist := convertKilometerToMeter(geo.CalculateGreatCircleDistance(
		gpsLat, gpsLon,
		matchCoord.Lat, matchCoord.Lon,
	))
	return dist >= DISTANCE_RESET_THRESHOLD
}

// logarithm of equation 21 ref[1]
func (om *OnlineMapMatchMHTWasm) computeObservationLogLikelihood(cand *ma.Candidate) float64 {

	xi := func(x float64) float64 {
		return (1 / (1 + math.Exp(-(math.Pi*(x-cand.GetDistr()))/(math.Sqrt(3)*om.gpsStd))))
	}

	zeroMeanGaussianLog := -(cand.GetDist() * cand.GetDist() / (2 * om.gpsStd * om.gpsStd))

	left := math.Log((1 / cand.Length())) + zeroMeanGaussianLog
	right := math.Log(xi(cand.Length()) - xi(0))
	return left + right
}

// equation 23 & 24 in ref[1]
func (om *OnlineMapMatchMHTWasm) kalmanFilter(speedMeanKprev, speedStdKprev, gpsSpeed, deltaTime float64) (float64, float64) {
	speedMeanK := speedMeanKprev
	speedStdK := math.Sqrt(speedStdKprev*speedStdKprev + om.accelerationStd*om.accelerationStd*deltaTime*deltaTime)
	numerator := om.initialSpeedStd*om.initialSpeedStd*speedMeanK + speedStdK*speedStdK*gpsSpeed
	denominator := om.initialSpeedStd*om.initialSpeedStd + speedStdK*speedStdK
	speedMean := numerator / denominator

	speedStdK = math.Sqrt(1 / (1/(om.initialSpeedStd*om.initialSpeedStd) + 1/(speedStdK*speedStdK)))
	return speedMean, speedStdK
}

// equation 3 in ref[1]
func (om *OnlineMapMatchMHTWasm) computEdgeTransitionProb(eFromId, eToId da.Index, nj int) float64 {

	eFrom := om.graph.GetOutEdge(eFromId)
	eFromOriginalId := eFrom.GetOriginalEdgeId()
	eTo := om.graph.GetOutEdge(eToId)
	eToOriginalId := eTo.GetOriginalEdgeId()

	branch := make([]da.Index, 0, 4)
	e := om.graph.GetOutEdge(eFromId)
	head := e.GetHead()
	om.graph.ForOutEdgesOf(head, func(eId, head da.Index, length float64, geometry []da.Coordinate) {
		e := om.graph.GetOutEdge(eId)
		branch = append(branch, e.GetOriginalEdgeId())
	})
	sumNej := 0.0
	for _, jOriginal := range branch {
		trans := float64(om.N.Get(int(eFromOriginalId), int(jOriginal)))
		if trans == 0 {
			trans = 1
		}
		sumNej += trans
	}
	return (1.0 + float64(om.N.Get(int(eFromOriginalId), int(eToOriginalId)))) / (sumNej + float64(nj))
}

// equation 20 in ref[1]
func (om *OnlineMapMatchMHTWasm) computeHProb(tau []da.Index, speedMean, speedStd, deltaTime float64) float64 {
	tauLength := 0.0
	for _, edgeId := range tau {
		e := om.graph.GetOutEdge(edgeId)
		tauLength += e.GetLength()
	}
	firstEdge := om.graph.GetOutEdge(tau[0])

	s := (math.Sqrt(3) * speedStd * deltaTime) / math.Pi
	out := (1.0 / firstEdge.GetLength())

	f := func(x float64) float64 {
		numerator := speedMean*deltaTime - (tauLength - x)
		denominator := (math.Sqrt(3) * speedStd * deltaTime) / math.Pi
		expo := math.Exp(numerator / denominator)
		log := math.Log(expo + 1.0)
		return s * log
	}

	return out * (f(firstEdge.GetLength()) - f(0))
}

func (om *OnlineMapMatchMHTWasm) projectAllCandidates(gps *da.GPSPoint, candidates []*ma.Candidate) {
	for _, cand := range candidates {

		gpsCoord := da.NewCoordinate(gps.Lat(), gps.Lon())
		candEdgeBearing := 0.0
		eGeometry := om.graph.GetEdgeGeometry(cand.EdgeId())

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
				projectedPoint.Lat, projectedPoint.Lon,
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
		cand.SetEdgeBearing(candEdgeBearing)
	}
}
