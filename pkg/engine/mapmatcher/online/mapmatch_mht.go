package online

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
)

/*
implementation of:
Taguchi, S., Koide, S. and Yoshimura, T. (2019) “Online Map Matching With Route
Prediction,” IEEE Transactions on Intelligent Transportation Systems, 20(1), pp.
338–347. Available at: https://doi.org/10.1109/TITS.2018.2812147.


udah bisa dites pakai dataset "https://www.microsoft.com/en-us/research/publication/hidden-markov-map-matching-noise-sparseness/"
see demo: "https://drive.google.com/file/d/1zfeQRon9ve5R2_PCEdQj1XL-mDmHvuD-/view?usp=sharing"
datset: "https://drive.google.com/file/d/1vdKt78d2QoNej3jc0OR0ajSOFBXrGUYt/view?usp=sharing"
*/

type OnlineMapMatchMHT struct {
	graph                   *da.Graph
	rt                      *spatialindex.Rtree
	initialSpeedMean        float64 // \overline{v}
	initialSpeedStd         float64 // \sigma_{v}
	posteriorThresold       float64 // L_u
	gpsStd                  float64
	defaultSamplingInterval float64
	accelerationStd         float64
	lp                      float64 // L_p
	lc                      float64 // L_c
	N                       *da.SparseMatrix[int]
}

func NewOnlineMapMatchMHT(graph *da.Graph, rt *spatialindex.Rtree, initialSpeedMean, initialSpeedStd float64,
	posteriorThresold, gpsStd, defaultSamplingInterval, lp, lc, accelerationStd float64,
	N *da.SparseMatrix[int]) *OnlineMapMatchMHT {
	return &OnlineMapMatchMHT{
		graph:                   graph,
		rt:                      rt,
		initialSpeedMean:        initialSpeedMean,
		initialSpeedStd:         initialSpeedStd,
		posteriorThresold:       posteriorThresold,
		gpsStd:                  gpsStd,
		defaultSamplingInterval: defaultSamplingInterval,
		lp:                      lp,
		accelerationStd:         accelerationStd,
		lc:                      lc,
		N:                       N,
	}
}

// OnlineMapMatch. perform online map matching using Multiple Hypothesis Technique
// speed in meter/minute, arc length in meter, k is current time step (1-based)
func (om *OnlineMapMatchMHT) OnlineMapMatch(gps *da.GPSPoint, k int,
	candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64) {

	if k == 1 || len(candidates) == 0 {
		nearbyArcs := om.rt.SearchWithinRadius(gps.Lat(), gps.Lon(), om.lc)
		candidates = make([]*ma.Candidate, 0, len(nearbyArcs))
		for _, arcEndpoint := range nearbyArcs {
			if arcEndpoint.GetLength() == 0 { // skip dummy arc
				continue
			}
			candidates = append(candidates, ma.NewCandidate(arcEndpoint.GetId(), arcEndpoint.GetLength(), arcEndpoint.GetLength()))
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
			predDist := convertMeterToKilometer(speedMean * gps.DeltaTime()) // speed in m/min, deltatime in min
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
func (om *OnlineMapMatchMHT) recur(newCands []*ma.Candidate, w float64, tau []da.Index, ptau float64,
	speedMean, hpre, speedStd, deltaTime float64) []*ma.Candidate {
	hnew := om.computeHProb(tau, speedMean, speedStd, deltaTime)
	if w*hnew*ptau > om.lp {
		eNext := make([]da.Index, 0, 5)
		lastEdgeId := tau[len(tau)-1]
		head := om.graph.GetOutEdge(lastEdgeId).GetHead()

		om.graph.ForOutEdgesOfWithId(head, func(outArc *da.OutEdge, Id da.Index) {

			eNext = append(eNext, Id)
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
		newCands = append(newCands, ma.NewCandidate(tau[len(tau)-1], wprime,
			om.graph.GetOutEdge(tau[len(tau)-1]).GetLength()))
	} else {
		cnew.SetWeight(cnew.Weight() + wprime)
	}
	return newCands
}

// filterLog. filter step of multiple hypothesis technique (compute posterior & pick most probable road segment)
//
//	normalization use log-sum-exp trick to avoid numerical underflow/overflow (https://gregorygundersen.com/blog/2020/02/09/log-sum-exp/)
func (om *OnlineMapMatchMHT) filterLog(gps *da.GPSPoint, candidates []*ma.Candidate) (*da.MatchedGPSPoint, []*ma.Candidate, bool) {
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
		if cand.Weight() > maxWeight {

			projectedPointCoord := cand.GetProjectedCoord()
			e := om.graph.GetOutEdge(cand.EdgeId())
			tail := om.graph.GetVertex(om.graph.GetTailOfOutedge(e.GetEdgeId()))
			head := om.graph.GetVertex(e.GetHead())
			eInitialBearing := geo.BearingTo(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
			matchedSegment = da.NewMatchedGPSPoint(gps, cand.EdgeId(), projectedPointCoord, eInitialBearing)
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

func (om *OnlineMapMatchMHT) needToReset(gps *da.GPSPoint, matchedSegment *da.MatchedGPSPoint) bool {
	gpsLat, gpsLon := gps.Lat(), gps.Lon()
	matchCoord := matchedSegment.GetMatchedCoord()
	dist := convertKilometerToMeter(geo.CalculateHaversineDistance(
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

func (om *OnlineMapMatchMHT) computeObservationLogLikelihood(cand *ma.Candidate) float64 {

	f := func(x float64) float64 {
		return (1 / (1 + math.Exp(-(math.Pi*(x-cand.GetDistr()))/(math.Sqrt(3)*om.gpsStd))))
	}

	gaussianLog := -(math.Pow(cand.GetDist(), 2) / (2 * math.Pow(om.gpsStd, 2)))

	left := math.Log((1 / cand.Length())) + gaussianLog
	right := math.Log(f(cand.Length()) - f(0))
	return left + right
}

func (om *OnlineMapMatchMHT) kalmanFilter(speedMeanKprev, speedStdKprev, gpsSpeed, deltaTime float64) (float64, float64) {
	speedMeanK := speedMeanKprev
	speedStdK := math.Sqrt(speedStdKprev*speedStdKprev + math.Pow(om.accelerationStd, 2)*math.Pow(deltaTime, 2))
	numerator := math.Pow(om.initialSpeedStd, 2)*speedMeanK + math.Pow(speedStdK, 2)*gpsSpeed
	denominator := math.Pow(om.initialSpeedStd, 2) + math.Pow(speedStdK, 2)
	speedMean := numerator / denominator

	speedStdK = math.Sqrt(1 / (1/math.Pow(om.initialSpeedStd, 2) + 1/math.Pow(speedStdK, 2)))
	return speedMean, speedStdK
}

func (om *OnlineMapMatchMHT) computEdgeTransitionProb(eFrom, eTo da.Index, nj int) float64 {
	branch := make([]da.Index, 0, 4)
	e := om.graph.GetOutEdge(eFrom)
	head := e.GetHead()
	om.graph.ForOutEdgesOfWithId(head, func(e *da.OutEdge, id da.Index) {

		branch = append(branch, id)
	})
	sumNej := 0.0
	for _, j := range branch {
		sumNej += float64(om.N.Get(int(eFrom), int(j)))
	}
	return (1.0 + float64(om.N.Get(int(eFrom), int(eTo)))) / (sumNej + float64(nj))
}

func (om *OnlineMapMatchMHT) computeHProb(tau []da.Index, speedMean, speedStd, deltaTime float64) float64 {
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

func (om *OnlineMapMatchMHT) computeObservationLikelihood(gps *da.GPSPoint, cand *ma.Candidate) float64 {
	e := om.graph.GetOutEdge(cand.EdgeId())
	headId := e.GetHead()
	tailId := om.graph.GetTailOfOutedge(cand.EdgeId())
	head := om.graph.GetVertex(headId)
	tail := om.graph.GetVertex(tailId)
	headPoint := da.NewCoordinate(head.GetLat(), head.GetLon()).ToGeoCoordinate()
	tailPoint := da.NewCoordinate(tail.GetLat(), tail.GetLon()).ToGeoCoordinate()
	gpsPoint := da.NewCoordinate(gps.Lat(), gps.Lon()).ToGeoCoordinate()
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

func (om *OnlineMapMatchMHT) computeObservationLikelihoodNewson(gps *da.GPSPoint, cand *ma.Candidate) float64 {
	e := om.graph.GetOutEdge(cand.EdgeId())
	headId := e.GetHead()
	tailId := om.graph.GetTailOfOutedge(cand.EdgeId())
	head := om.graph.GetVertex(headId)
	tail := om.graph.GetVertex(tailId)
	headPoint := da.NewCoordinate(head.GetLat(), head.GetLon()).ToGeoCoordinate()
	tailPoint := da.NewCoordinate(tail.GetLat(), tail.GetLon()).ToGeoCoordinate()
	gpsPoint := da.NewCoordinate(gps.Lat(), gps.Lon()).ToGeoCoordinate()
	projectedPoint := geo.ProjectPointToLineCoord(headPoint, tailPoint,
		gpsPoint)

	dist := convertKilometerToMeter(geo.CalculateHaversineDistance(
		projectedPoint.Lat, projectedPoint.Lon,
		gpsPoint.Lat, gpsPoint.Lon,
	))

	prob := (1.0 / (math.Sqrt(2*math.Pi) * om.gpsStd)) * math.Exp(-0.5*math.Pow(dist/om.gpsStd, 2))

	return prob
}

func (om *OnlineMapMatchMHT) projectAllCandidates(gps *da.GPSPoint, candidates []*ma.Candidate) {
	for _, cand := range candidates {

		gpsCoord := da.NewCoordinate(gps.Lat(), gps.Lon())

		eGeometry := om.graph.GetEdgeGeometry(cand.EdgeId())

		var (
			minDist, minDistr  float64 = math.MaxFloat64, math.MaxFloat64
			bestProjectedPoint geo.Coordinate
		)

		for i := 0; i < len(eGeometry)-1; i++ {
			tail := eGeometry[i]
			head := eGeometry[i+1]
			projectedPoint := geo.ProjectPointToLineCoord(
				tail.ToGeoCoordinate(),
				head.ToGeoCoordinate(),
				gpsCoord.ToGeoCoordinate(),
			)
			dist := convertKilometerToMeter(geo.CalculateHaversineDistance(
				projectedPoint.Lat, projectedPoint.Lon,
				gpsCoord.GetLat(), gpsCoord.GetLon(),
			))

			distr := convertKilometerToMeter(geo.CalculateHaversineDistance(
				tail.GetLat(), tail.GetLon(),
				projectedPoint.GetLat(), projectedPoint.GetLon(),
			))

			if dist < minDist {
				minDist = dist
				minDistr = distr
				bestProjectedPoint = projectedPoint
			}

		}

		cand.SetProjectedCoord(bestProjectedPoint.GetLat(), bestProjectedPoint.GetLon())
		cand.SetDist(minDist)
		cand.SetDistr(minDistr)
	}
}
