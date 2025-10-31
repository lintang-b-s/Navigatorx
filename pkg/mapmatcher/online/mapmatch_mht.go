package online

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
)

/*
implementation of:
Taguchi, S., Koide, S. and Yoshimura, T. (2019) “Online Map Matching With Route
Prediction,” IEEE Transactions on Intelligent Transportation Systems, 20(1), pp.
338–347. Available at: https://doi.org/10.1109/TITS.2018.2812147.


udah bisa dites pakai dataset "https://www.microsoft.com/en-us/research/publication/hidden-markov-map-matching-noise-sparseness/"
see demo: "https://drive.google.com/file/d/1VpwW7O_3FP7bgTiXwWHQq2fTSwS0F-E9/view?usp=sharing"
*/

type OnlineMapMatchMHT struct {
	graph             *datastructure.Graph
	rt                *spatialindex.Rtree
	initialSpeedMean  float64 // \overline{v}
	initialSpeedStd   float64 // \sigma_{v}
	posteriorThresold float64 // L_u
	gpsStd            float64
	samplingInterval  float64
	accelerationStd   float64
	lp                float64 // L_p
	lc                float64 // L_c
}

func NewOnlineMapMatchMHT(graph *datastructure.Graph, rt *spatialindex.Rtree, initialSpeedMean, initialSpeedStd float64,
	posteriorThresold, gpsStd, samplingInterval, lp, lc, accelerationStd float64) *OnlineMapMatchMHT {
	return &OnlineMapMatchMHT{
		graph:             graph,
		rt:                rt,
		initialSpeedMean:  initialSpeedMean,
		initialSpeedStd:   initialSpeedStd,
		posteriorThresold: posteriorThresold,
		gpsStd:            gpsStd,
		samplingInterval:  samplingInterval,
		lp:                lp,
		accelerationStd:   accelerationStd,
		lc:                lc,
	}
}

// OnlineMapMatch. perform online map matching using Multiple Hypothesis Technique
// speed in meter/minute, arc length in meter, k is current time step (1-based)
func (om *OnlineMapMatchMHT) OnlineMapMatch(gps *datastructure.GPSPoint, k int,
	candidates []*Candidate, speedMeanK, speedStdK float64) (*datastructure.MatchedGPSPoint, []*Candidate, float64, float64) {

	if k == 1 || len(candidates) == 0 {
		nearbyArcs := om.rt.SearchWithinRadius(gps.Lat(), gps.Lon(), om.lc)
		candidates = make([]*Candidate, len(nearbyArcs))
		for i, arcEndpoint := range nearbyArcs {
			candidates[i] = NewCandidate(arcEndpoint.GetId(), arcEndpoint.GetLength(), arcEndpoint.GetLength())
		}

		matchedPoint, newcandidates, _ := om.filterLog(gps, candidates)
		return matchedPoint, newcandidates, om.initialSpeedMean, om.initialSpeedStd
	} else {
		var speedMean, speedStd float64
		if k == 2 {
			speedMean = om.initialSpeedMean
			speedStd = om.initialSpeedStd
		} else {
			speedMean = speedMeanK
			speedStd = speedStdK
		}

		newCandidates := make([]*Candidate, 0, len(candidates))

		for _, cand := range candidates {
			tau := make([]datastructure.Index, 0)
			tau = append(tau, cand.EdgeId())
			ptau := 1.0
			hpre := 1.0
			newCandidates = om.recur(newCandidates, cand.Weight(), tau, ptau, speedMean, hpre, speedStd)
		}
		speedMeanK, speedStdK = om.kalmanFilter(speedMean, speedStd, gps.Speed())

		matchedPoint, newCandidatesFiltered, reset := om.filterLog(gps, newCandidates)
		if reset {
			return matchedPoint, make([]*Candidate, 0), om.initialSpeedMean, om.initialSpeedStd
		}
		return matchedPoint, newCandidatesFiltered, speedMeanK, speedStdK
	}
}

// recur. prediction step of multiple hypothesis technique (compute prior)
func (om *OnlineMapMatchMHT) recur(newCands []*Candidate, w float64, tau []datastructure.Index, ptau float64,
	speedMean, hpre, speedStd float64) []*Candidate {
	hnew := om.computeHProb(tau, speedMean, speedStd)
	if w*hnew*ptau > om.lp {
		eNext := make([]datastructure.Index, 0)
		lastEdgeId := tau[len(tau)-1]
		head := om.graph.GetOutEdge(lastEdgeId).GetHead()

		om.graph.ForOutEdgesOfWithId(head, func(outArc *datastructure.OutEdge, Id datastructure.Index) {
			eNext = append(eNext, Id)
		})

		for _, nextEdgeId := range eNext {
			tauPrime := make([]datastructure.Index, len(tau))
			copy(tauPrime, tau)
			tauPrime = append(tauPrime, nextEdgeId)
			nj := len(eNext)
			ptauPrime := ptau * om.computEdgeTransitionProb(lastEdgeId, nextEdgeId, nj)
			hprePrime := hnew
			newCands = om.recur(newCands, w, tauPrime, ptauPrime, speedMean, hprePrime, speedStd)
		}
	}
	wprime := w * ptau * (hpre - hnew)
	var cnew *Candidate
	for _, cand := range newCands {
		if cand.EdgeId() == tau[len(tau)-1] { // tau[len(tau)-1]=r_{k+1}
			cnew = cand
		}
	}
	if cnew == nil {
		newCands = append(newCands, NewCandidate(tau[len(tau)-1], wprime,
			om.graph.GetOutEdge(tau[len(tau)-1]).GetLength()))
	} else {
		cnew.SetWeight(cnew.Weight() + wprime)
	}
	return newCands
}

// filterLog. filter step of multiple hypothesis technique (compute posterior & pick most probable road segment)
//
//	normalization use log-sum-exp trick to avoid numerical underflow/overflow (https://gregorygundersen.com/blog/2020/02/09/log-sum-exp/)
func (om *OnlineMapMatchMHT) filterLog(gps *datastructure.GPSPoint, candidates []*Candidate) (*datastructure.MatchedGPSPoint, []*Candidate, bool) {
	logAllCandWeights := make([]float64, 0, len(candidates))

	for _, cand := range candidates {
		obsLogLikelihood := om.computeObservationLogLikelihood(gps, cand)

		logAllCandWeights = append(logAllCandWeights, math.Log(cand.Weight())+obsLogLikelihood)
	}

	allCandsWeightLSE := logSumExp(logAllCandWeights)
	sumPosterior := 0.0 // should \approx 1

	for i, cand := range candidates {
		obsLogLikelihood := om.computeObservationLogLikelihood(gps, cand)
		posterior := (obsLogLikelihood + math.Log(cand.Weight())) - (allCandsWeightLSE)
		candidates[i] = NewCandidate(cand.EdgeId(), math.Exp(posterior), cand.Length())
		sumPosterior += math.Exp(posterior)
	}

	// filter candidate yang memiliki weight < posteriorThreshold
	filteredCands := make([]*Candidate, 0, len(candidates))
	for _, cand := range candidates {
		if math.IsNaN(cand.Weight()) {
			continue
		}
		if cand.Weight() > om.posteriorThresold {
			filteredCands = append(filteredCands, cand)
		}
	}

	// argmax posterior
	var matchedSegment *datastructure.MatchedGPSPoint
	maxWeight := -1.0
	for _, cand := range filteredCands {
		if cand.Weight() > maxWeight {
			candE := om.graph.GetOutEdge(cand.EdgeId())
			head := om.graph.GetVertex(candE.GetHead())
			tail := om.graph.GetVertex(om.graph.GetTailOfOutedge(cand.EdgeId()))
			projectedPoint := geo.ProjectPointToLineCoord(
				datastructure.NewCoordinate(head.GetLat(), head.GetLon()).ToGeoCoordinate(),
				datastructure.NewCoordinate(tail.GetLat(), tail.GetLon()).ToGeoCoordinate(),
				datastructure.NewCoordinate(gps.Lat(), gps.Lon()).ToGeoCoordinate(),
			)
			projectedPointCoord := datastructure.NewCoordinate(projectedPoint.Lat, projectedPoint.Lon)
			matchedSegment = datastructure.NewMatchedGPSPoint(gps, cand.EdgeId(), projectedPointCoord)
			maxWeight = cand.Weight()
		}
	}

	return matchedSegment, filteredCands, om.needToReset(gps, matchedSegment)
}

func (om *OnlineMapMatchMHT) needToReset(gps *datastructure.GPSPoint, matchedSegment *datastructure.MatchedGPSPoint) bool {
	gpsLat, gpsLon := gps.Lat(), gps.Lon()
	matchCoord := matchedSegment.GetMatchedCoord()
	dist := convertKilometerToMeter(geo.CalculateEuclidianDistanceEquirectangularProj(
		gpsLat, gpsLon,
		matchCoord.Lat, matchCoord.Lon,
	))
	if dist >= DISTANCE_RESET_THRESHOLD {
		return true
	}
	return false
}

// https://gregorygundersen.com/blog/2020/02/09/log-sum-exp/
func logSumExp(ps []float64) float64 {
	if len(ps) == 0 {
		return math.Inf(-1)
	}
	maxP := ps[0]
	for _, p := range ps {
		if p > maxP {
			maxP = p
		}
	}
	sumExp := 0.0
	for _, p := range ps {
		sumExp += math.Exp(p - maxP)
	}
	return maxP + math.Log(sumExp)
}

func (om *OnlineMapMatchMHT) computeObservationLogLikelihood(gps *datastructure.GPSPoint, cand *Candidate) float64 {
	e := om.graph.GetOutEdge(cand.edgeId)
	headId := e.GetHead()
	tailId := om.graph.GetTailOfOutedge(cand.edgeId)
	head := om.graph.GetVertex(headId)
	tail := om.graph.GetVertex(tailId)
	headPoint := datastructure.NewCoordinate(head.GetLat(), head.GetLon()).ToGeoCoordinate()
	tailPoint := datastructure.NewCoordinate(tail.GetLat(), tail.GetLon()).ToGeoCoordinate()
	gpsPoint := datastructure.NewCoordinate(gps.Lat(), gps.Lon()).ToGeoCoordinate()
	projectedPoint := geo.ProjectPointToLineCoord(headPoint, tailPoint,
		gpsPoint)

	dist := convertKilometerToMeter(geo.CalculateEuclidianDistanceEquirectangularProj(
		projectedPoint.Lat, projectedPoint.Lon,
		gpsPoint.Lat, gpsPoint.Lon,
	))

	distr := convertKilometerToMeter(geo.CalculateEuclidianDistanceEquirectangularProj(
		tailPoint.Lat, tailPoint.Lon,
		projectedPoint.Lat, projectedPoint.Lon,
	))

	f := func(x float64) float64 {
		return (1 / (1 + math.Exp(-(math.Pi*(x-distr))/(math.Sqrt(3)*om.gpsStd))))
	}

	gaussianLog := -(math.Pow(dist, 2) / (2 * math.Pow(om.gpsStd, 2)))

	left := math.Log((1 / cand.Length())) + gaussianLog
	right := math.Log(f(cand.Length()) - f(0))
	return left + right
}

func (om *OnlineMapMatchMHT) kalmanFilter(speedMeanKprev, speedStdKprev, gpsSpeed float64) (float64, float64) {
	speedMeanK := speedMeanKprev
	speedStdK := math.Sqrt(speedStdKprev*speedStdKprev + math.Pow(om.accelerationStd, 2)*math.Pow(om.samplingInterval, 2))
	numerator := math.Pow(om.initialSpeedStd, 2)*speedMeanK + math.Pow(speedStdK, 2)*gpsSpeed
	denominator := math.Pow(om.initialSpeedStd, 2) + math.Pow(speedStdK, 2)
	speedMean := numerator / denominator

	speedStdK = math.Sqrt(1 / (1/math.Pow(om.initialSpeedStd, 2) + 1/math.Pow(speedStdK, 2)))
	return speedMean, speedStdK
}

func (om *OnlineMapMatchMHT) computEdgeTransitionProb(eFrom, eTo datastructure.Index, nj int) float64 {
	return 1.0 / float64(nj)
}

func (om *OnlineMapMatchMHT) computeHProb(tau []datastructure.Index, speedMean, speedStd float64) float64 {
	tauLength := 0.0
	for _, edgeId := range tau {
		e := om.graph.GetOutEdge(edgeId)
		tauLength += e.GetLength()
	}
	firstEdge := om.graph.GetOutEdge(tau[0])

	s := (math.Sqrt(3) * speedStd * om.samplingInterval) / math.Pi
	out := (1.0 / firstEdge.GetLength())

	f := func(x float64) float64 {
		numerator := speedMean*om.samplingInterval - (tauLength - x)
		denominator := (math.Sqrt(3) * speedStd * om.samplingInterval) / math.Pi
		expo := math.Exp(numerator / denominator)
		log := math.Log(expo + 1.0)
		return s * log
	}

	return out * (f(firstEdge.GetLength()) - f(0))
}

func (om *OnlineMapMatchMHT) computeObservationLikelihood(gps *datastructure.GPSPoint, cand *Candidate) float64 {
	e := om.graph.GetOutEdge(cand.edgeId)
	headId := e.GetHead()
	tailId := om.graph.GetTailOfOutedge(cand.edgeId)
	head := om.graph.GetVertex(headId)
	tail := om.graph.GetVertex(tailId)
	headPoint := datastructure.NewCoordinate(head.GetLat(), head.GetLon()).ToGeoCoordinate()
	tailPoint := datastructure.NewCoordinate(tail.GetLat(), tail.GetLon()).ToGeoCoordinate()
	gpsPoint := datastructure.NewCoordinate(gps.Lat(), gps.Lon()).ToGeoCoordinate()
	projectedPoint := geo.ProjectPointToLineCoord(headPoint, tailPoint,
		gpsPoint)

	dist := convertKilometerToMeter(geo.CalculateHaversineDistance(
		projectedPoint.Lat, projectedPoint.Lon,
		gpsPoint.Lat, gpsPoint.Lon,
	))

	distr := convertKilometerToMeter(geo.CalculateHaversineDistance(
		tailPoint.Lat, tailPoint.Lon,
		projectedPoint.Lat, projectedPoint.Lon,
	))

	f := func(x float64) float64 {
		return (1 / (1 + math.Exp(-(math.Pi*(x-distr))/(math.Sqrt(3)*om.gpsStd))))
	}

	expo := math.Exp(-(math.Pow(dist, 2) / (2 * math.Pow(om.gpsStd, 2)))) // can be NaN if dist is so big...

	left := (1 / cand.Length()) * expo
	right := f(cand.Length()) - f(0)
	return left * right
}

func (om *OnlineMapMatchMHT) computeObservationLikelihoodNewson(gps *datastructure.GPSPoint, cand *Candidate) float64 {
	e := om.graph.GetOutEdge(cand.edgeId)
	headId := e.GetHead()
	tailId := om.graph.GetTailOfOutedge(cand.edgeId)
	head := om.graph.GetVertex(headId)
	tail := om.graph.GetVertex(tailId)
	headPoint := datastructure.NewCoordinate(head.GetLat(), head.GetLon()).ToGeoCoordinate()
	tailPoint := datastructure.NewCoordinate(tail.GetLat(), tail.GetLon()).ToGeoCoordinate()
	gpsPoint := datastructure.NewCoordinate(gps.Lat(), gps.Lon()).ToGeoCoordinate()
	projectedPoint := geo.ProjectPointToLineCoord(headPoint, tailPoint,
		gpsPoint)

	dist := convertKilometerToMeter(geo.CalculateHaversineDistance(
		projectedPoint.Lat, projectedPoint.Lon,
		gpsPoint.Lat, gpsPoint.Lon,
	))

	prob := (1.0 / (math.Sqrt(2*math.Pi) * om.gpsStd)) * math.Exp(-0.5*math.Pow(dist/om.gpsStd, 2))

	return prob
}
