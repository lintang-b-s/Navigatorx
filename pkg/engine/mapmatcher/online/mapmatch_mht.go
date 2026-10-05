// Package online provides online map matching implementations using MHT.
package online

/*
implementation of:
[1] Taguchi, S., Koide, S. and Yoshimura, T. (2019) “Online Map Matching With Route
Prediction,” IEEE Transactions on Intelligent Transportation Systems, 20(1), pp.
338–347. Available at: https://doi.org/10.1109/TITS.2018.2812147.

evaluation/tests suite di: tests/mapmatching/ (cari yang ada nama Online di test funtions nya)


metode yang dipropose di ref[1] implement bayes filter
dimana goal nya adalah compute posterior probability distribution (pmf or pdf) over the current state x_t
given the history of measurements z_{1:t}.
di kasus ini state nya cuma road segment yang ditempati vehicle pada time step t
given gps measurements z_{1:t}
bayes filter terdiri dari prediction step dan measurement update (atau filter/correction) step.
prediction step computes the prior probability distribution p(x_t | z_{1:z_{t-1}}) yang memberikan estimasi road segmen yang
mungkin ditempati vehicle di time step t, sebelum incorporate gps measurement time step t.
measurement update step menyempurnakan estimasi ini dengan incorporate gps measurement pada time step t.

integral di prediction step dan normalization di measurement step bisa computationally intractable for continuous, high-dimensional state spaces.
sehingga biasanya untuk practical state estimation algorithm mengandalkan aproksimasi dari state posterior prob. distribution.
untuk approximasi posterior prob. dist., biasanya ada dua tipe filter: parametric filter dan non-parametric filter.
parametric filter contohnya: kalman filter, extended kalman filter, dan unscented kalman filter. ketiganya
assuming state posterior  prob. dist. terdistribusi gaussian.
non-parametric filter contohnya: particle filter dan multiple hypothesis technique..
di non-parametric filter kita gak assuming distribusi dari posterior.
baca https://porabook.com/book/localization-filtering/ dan https://porabook.com/book/approximate-filters/ untuk penjelasan bayes filter.


This explanation is taken from ref[1]:
In the MHT, the probability density function
is divided into the probability of each candidate. When using
the MHT, the prediction and filtering calculations simplify
because they are performed by calculating each candidate. The
idea that the complex probability distribution is represented
by a finite number of candidates is similar to the particle filter
approach

However, the MHT differs from a particle filter
in that it does not use random sampling to predict states. In the
MHT, each candidate produces new candidates for all states
that can be transited and then candidates that represent the
same states are integrated by summing their probabilities.

For a system based on discrete transitions between a finite number
of states, the MHT is more suitable than the particle filter
because its calculation cost is an order of magnitude lower
and it does not cause degeneracy. In the MHT, the number
of candidates becomes large during prediction; therefore,
we remove candidates that have a probability below a threshold
value L_u after filtering.


*/

// type OnlineMapMatchMHT struct {
// 	graph             *da.Graph
// 	rn                *da.RoadNetworkDataContainer
// 	getSegmentLength  func(da.Index) float64
// 	rt                *spatialindex.Rtree
// 	initialSpeedMean  float64 // \overline{v}
// 	initialSpeedStd   float64 // \sigma_{v}
// 	posteriorThresold float64 // L_u
// 	gpsStd            float64
// 	accelerationStd   float64
// 	lp                float64 // L_p
// 	lc                float64 // L_c
// 	N                 *da.SparseMatrix
// 	tipe              TIPE_MHT
// }

// func NewOnlineMapMatchMHT(graph *da.Graph, rt *spatialindex.Rtree, initialSpeedMean, initialSpeedStd float64,
// 	posteriorThresold, gpsStd, lp, lc, accelerationStd float64,
// 	N *da.SparseMatrix, getSegmentLength func(da.Index) float64) *OnlineMapMatchMHT {
// 	return &OnlineMapMatchMHT{
// 		graph:             graph,
// 		getSegmentLength:  getSegmentLength,
// 		rt:                rt,
// 		initialSpeedMean:  initialSpeedMean,
// 		initialSpeedStd:   initialSpeedStd,
// 		posteriorThresold: posteriorThresold,
// 		gpsStd:            gpsStd,
// 		lp:                lp,
// 		accelerationStd:   accelerationStd,
// 		lc:                lc,
// 		N:                 N,
// 		tipe:              MHT_TIPE_TWO,
// 	}
// }

// // OnlineMapMatch. perform online map matching using Multiple Hypothesis Technique
// // speed in meter/s, arc length in meter, k is current time step (1-based)
// // Algorithm 1 in ref[1]
// // O(b^{d_p}), b=max outDegree of any vertex in the graph, d_p=maxVelocity*sampling interval/avgSegmentLength [1]
// func (om *OnlineMapMatchMHT) OnlineMapMatch(prevGps, gps *da.GPSPoint, k int,
// 	candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64) {

// 	if k == 1 || len(candidates) == 0 {
// 		nearbyArcs := om.rt.SearchWithinRadius(gps.Lat(), gps.Lon(), om.lc, 3) // O(M)
// 		sumLength := 0.0
// 		startOfTheRoute := len(candidates) > 0

// 		var initialCandidate *ma.Candidate
// 		if startOfTheRoute {
// 			initialCandidate = candidates[0]
// 		}

// 		candidates = make([]*ma.Candidate, 0, len(nearbyArcs))

// 		for _, segmentId := range nearbyArcs {
// 			if !startOfTheRoute || (startOfTheRoute && segmentId == initialCandidate.GetSegmentId()) {
// 				eLength := om.getSegmentLength(segmentId)
// 				sumLength += eLength
// 			}
// 		}

// 		for _, eId := range nearbyArcs {
// 			if !startOfTheRoute || (startOfTheRoute && eId == initialCandidate.GetSegmentId()) {
// 				// biar candidates nya cuma first segmentId dari rute yang dipilih user (see https://github.com/lintang-b-s/navigatorx-crp-fe/blob/main/app/page.tsx).
// 				eLength := om.getSegmentLength(eId)
// 				c := ma.NewCandidate(eId, eLength/sumLength, eLength)

// 				candidates = append(candidates, c)
// 			}
// 		}

// 		om.projectAllCandidates(gps, candidates)

// 		matchedPoint, newcandidates, _ := om.filterLog(gps, candidates)

// 		return matchedPoint, newcandidates, om.initialSpeedMean, om.initialSpeedStd
// 	} else {
// 		var (
// 			speedMean, speedStd float64
// 		)
// 		if k == 2 {
// 			speedMean = om.initialSpeedMean
// 			speedStd = om.initialSpeedStd
// 		} else {
// 			speedMean = speedMeanK
// 			speedStd = speedStdK
// 		}

// 		om.projectAllCandidates(prevGps, candidates)

// 		newCandidates := make([]*ma.Candidate, 0, len(candidates))

// 		for _, cand := range candidates {
// 			tau := make([]da.Index, 0, 5)
// 			tau = append(tau, cand.GetSegmentId())
// 			ptau := 1.0
// 			hpre := 1.0
// 			newCandidates = om.recur(newCandidates, cand.Weight(), tau, ptau, speedMean, hpre, speedStd, gps.DeltaTime(), prevGps, gps, cand)
// 		}
// 		speedMeanK, speedStdK = om.kalmanFilter(speedMean, speedStd, gps.Speed(), gps.DeltaTime())

// 		om.projectAllCandidates(gps, newCandidates)

// 		matchedPoint, newCandidatesFiltered, reset := om.filterLog(gps, newCandidates)

// 		if reset {
// 			return matchedPoint, make([]*ma.Candidate, 0), om.initialSpeedMean, om.initialSpeedStd
// 		}

// 		return matchedPoint, newCandidatesFiltered, speedMeanK, speedStdK
// 	}
// }

// // recur. prediction step of multiple hypothesis technique (compute prior)
// // Algorithm 2 in ref[1]
// // route prediction buat compute prior probability dari next road segment candidates r_{k+1}
// func (om *OnlineMapMatchMHT) recur(newCands []*ma.Candidate, w float64, tau []da.Index, ptau float64,
// 	speedMean, hpre, speedStd, deltaTime float64, prevGps, gps *da.GPSPoint, prevCand *ma.Candidate) []*ma.Candidate {
// 	hnew := om.computeHProb(tau, speedMean, speedStd, deltaTime)

// 	if w*hnew*ptau > om.lp {
// 		eNext := make([]da.Index, 0, 5)
// 		lSegId := tau[len(tau)-1]

// 		om.graph.ForOutEdgesOf(lSegId, func(_, nSegId da.Index, _ da.Index) {
// 			eNext = append(eNext, nSegId)
// 		})

// 		for _, nextSegment := range eNext {
// 			// iterate all road segment connected to last road segment (atau last state) dari current markov chain path
// 			tauPrime := make([]da.Index, len(tau))
// 			copy(tauPrime, tau)
// 			tauPrime = append(tauPrime, nextSegment)
// 			nj := len(eNext)

// 			// compute new markov chain path probability that enter this next state (road segment nextSegment)
// 			ptauPrime := ptau * om.computEdgeTransitionProb(lSegId, nextSegment, nj)
// 			hprePrime := hnew
// 			newCands = om.recur(newCands, w, tauPrime, ptauPrime, speedMean, hprePrime, speedStd, deltaTime, prevGps, gps, prevCand)
// 		}
// 	}

// 	// compute route prediction probability that defined in eq [1] or eq [17] in ref 1
// 	// route prediction probability in eq [1] in ref [1]
// 	// route prediction probability p(r_{k+1}|r_k)  defined as marginalized joint pmf of r_{k+1},tau|r_k. using total probability theorem we can get eq [1] in ref 1
// 	// (hpre-hnew) is for computing p(r_{k+1}| r_{k}, tau) defined in eq [18] in ref 1
// 	// ptau is markov chain path tau probability that start di current road segment r_k ending at r_{k+1}
// 	// r_k adalah rkCand di fungsi OnlineMapMatch
// 	// r_{k+1} adalah  tau[len(tau)-1] atau last state dari current markov chain path tau

// 	routePredProb := 0.0
// 	if om.tipe == MHT_TIPE_ONE {
// 		routePredProb = ptau * (hpre - hnew) // route prediction probability
// 	} else {
// 		routePredProb = ptau * om.computeSegmentTransitionProb(tau, prevGps, gps, prevCand)
// 	}

// 	// calculate prior probability in eq [12] ref 1, prior probability p(r_{k+1}|g_{1:k}) calculated as marginalized joint pmf p(r_{k+1}, r_{k}|g_{1:k}), this joint pmf can be written as conditional probability like in eq [12] row 2 by total probability theorem.
// 	wprime := w * routePredProb // compute prior probability. w adalah posterior probability dari candidate (road segment) r_k di previous time step
// 	var cnew *ma.Candidate
// 	for _, cand := range newCands {
// 		if cand.GetSegmentId() == tau[len(tau)-1] { // tau[len(tau)-1]=r_{k+1}
// 			cnew = cand
// 		}
// 	}
// 	if cnew == nil {

// 		newCands = append(newCands, ma.NewCandidate(tau[len(tau)-1], wprime,
// 			om.getSegmentLength(tau[len(tau)-1])))
// 	} else {
// 		cnew.SetWeight(cnew.Weight() + wprime)
// 	}
// 	return newCands
// }

// // filterLog. filter step of multiple hypothesis technique (compute posterior & pick most probable road segment)
// //
// //	normalization use log-sum-exp trick to avoid numerical underflow/overflow (https://gregorygundersen.com/blog/2020/02/09/log-sum-exp/)
// func (om *OnlineMapMatchMHT) filterLog(gps *da.GPSPoint, candidates []*ma.Candidate) (*da.MatchedGPSPoint, []*ma.Candidate, bool) {
// 	logDenominator := make([]float64, 0, len(candidates))

// 	for _, cand := range candidates {
// 		obsLogLikelihood := om.computeEmissionLogProb(cand)
// 		logDenominator = append(logDenominator, math.Log(cand.Weight())+obsLogLikelihood)
// 	}

// 	logDenominatorLSE := logSumExp(logDenominator) // log-sum-exp trick
// 	sumPosterior := 0.0                            // should approx 1

// 	for i, cand := range candidates {
// 		// computing posterior for each road segment candidate, each road segment candidate is mutually exclusive/disjoint
// 		obsLogLikelihood := om.computeEmissionLogProb(cand)
// 		logNumerator := (obsLogLikelihood + math.Log(cand.Weight())) // cand.Weight is the prior probability dari road segment cand hasil dari fungsi recur()
// 		posterior := logNumerator - (logDenominatorLSE)              // log of equation (11) in ref[1] using log-sum-exp trick.

// 		posteriorProb := math.Exp(posterior) // posterior back to [0,1]. hasil rewriting eq (3) di https://gregorygundersen.com/blog/2020/02/09/log-sum-exp/
// 		candidates[i].SetWeight(posteriorProb)
// 		sumPosterior += posteriorProb // karena  value posteriorProb [0,1], sum over all candidate yg saling mutually exlusive must approx to 1
// 	}

// 	// filter candidate yang memiliki weight < posteriorThreshold
// 	filteredCands := make([]*ma.Candidate, 0, len(candidates))
// 	for _, cand := range candidates {
// 		if math.IsNaN(cand.Weight()) {
// 			continue
// 		}
// 		if cand.Weight() > om.posteriorThresold {
// 			filteredCands = append(filteredCands, cand)
// 		}
// 	}

// 	// argmax posterior
// 	var matchedSegment *da.MatchedGPSPoint
// 	maxWeight := -1.0
// 	for _, cand := range filteredCands {
// 		if util.Gt(cand.Weight(), maxWeight) {
// 			projectedPointCoord := cand.GetProjectedCoord()
// 			eInitialBearing := cand.GetEdgeBearing()
// 			matchedSegment = da.NewMatchedGPSPoint(gps, cand.GetSegmentId(), projectedPointCoord, eInitialBearing, 0) // kita gak pake obsId online map matching
// 			maxWeight = cand.Weight()
// 		}
// 	}
// 	if matchedSegment == nil {
// 		gpsPoint := da.NewGPSPoint(gps.Lat(), gps.Lon(), gps.Time(), gps.Speed(), gps.DeltaTime())
// 		invalidMatchedCoord := da.NewCoordinate(INVALID_LAT, INVALID_LON)
// 		matchedSegment = da.NewMatchedGPSPoint(gpsPoint, da.INVALID_SEGMENT_ID, invalidMatchedCoord, 0.0, 0)
// 	}

// 	return matchedSegment, filteredCands, om.needToReset(gps, matchedSegment)
// }

// func (om *OnlineMapMatchMHT) needToReset(gps *da.GPSPoint, matchedSegment *da.MatchedGPSPoint) bool {
// 	gpsLat, gpsLon := gps.Lat(), gps.Lon()
// 	matchCoord := matchedSegment.GetMatchedCoord()
// 	dist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(
// 		gpsLat, gpsLon,
// 		matchCoord.GetLat(), matchCoord.GetLon(),
// 	))
// 	return dist >= DISTANCE_RESET_THRESHOLD
// }

// func (om *OnlineMapMatchMHT) computeSegmentTransitionProb(tau []da.Index, prevGps, curGps *da.GPSPoint, prevCand *ma.Candidate) float64 {
// 	tauLength := 0.0
// 	lid := max(0, len(tau)-1)
// 	for _, segmentId := range tau[:lid] {
// 		tauLength += om.getSegmentLength(segmentId)
// 	}

// 	mDist := geo.CalculateGreatCircleDistance(prevGps.Lat(), prevGps.Lon(), curGps.Lat(), curGps.Lon())
// 	mDist = util.KilometerToMeter(mDist)

// 	leId := tau[lid]
// 	nextSegPoint, _, ldistr, _ := om.projectGpsToRoadSegment(curGps.GetCoordinate(), leId)
// 	_ = nextSegPoint
// 	routeDist := tauLength + ldistr - prevCand.GetDistr()
// 	d := (mDist - routeDist)
// 	dAbs := math.Abs(d)
// 	pr := (1 / beta) * math.Exp(-dAbs/beta)

// 	return pr
// }

// // computeEmissionLogProb. logarithm of equation [1]  in https://www.microsoft.com/en-us/research/wp-content/uploads/2016/12/map-matching-ACM-GIS-camera-ready.pdf
// // give the likelihood that a gps measurement/observation resulted from a given road segment candidate
// // that hmm newson paper model absolute distance from gps point to candidate road segment as zero-mean gaussian distribution
// func (om *OnlineMapMatchMHT) computeEmissionLogProb(cand *ma.Candidate) float64 {
// 	obsStateDist := cand.GetDist()
// 	sigma := om.gpsStd
// 	emsLogProb := -0.5*(math.Log(2.0*math.Pi)+(obsStateDist/sigma)*(obsStateDist/sigma)) - math.Log(sigma)
// 	return emsLogProb
// }

// // // logarithm of equation 21 ref[1]
// // func (om *OnlineMapMatchMHT) computeObservationLogLikelihood(cand *ma.Candidate) float64 {

// // 	xi := func(x float64) float64 {
// // 		return (1 / (1 + math.Exp(-(math.Pi*(x-cand.GetDistr()))/(math.Sqrt(3)*om.gpsStd))))
// // 	}

// // 	zeroMeanGaussianLog := -(cand.GetDist() * cand.GetDist() / (2 * om.gpsStd * om.gpsStd))

// // 	left := -math.Log(cand.Length()) + zeroMeanGaussianLog
// // 	right := math.Log(xi(cand.Length()) - xi(0))
// // 	return left + right
// // }

// // equation 23 & 24 in ref[1]
// // linear kalman filter buat estimate vehicle velocity at time step k.
// // velocity constant model, state nya cuma velocity at time k.
// // karena velocity constant model, transition model nya: velocity di time step k sama dengan velocity dengan time step k-1.
// // measurement cuma speed dari gps data (bisa didapet dari Android FusedLocationProvider API/ expo location API https://docs.expo.dev/versions/latest/sdk/location/)
// // ref for kalman filter: https://porabook.com/book/approximate-filters/#S2
// func (om *OnlineMapMatchMHT) kalmanFilter(speedMeanKprev, speedStdKprev, gpsSpeed, deltaTime float64) (float64, float64) {
// 	// prediction step dari kalman filter
// 	speedMeanK := speedMeanKprev // transition model. calculate prediction state.
// 	speedStdK := math.Sqrt(speedStdKprev*speedStdKprev + om.accelerationStd*om.accelerationStd*deltaTime*deltaTime)

// 	// correction/measurement update step dari kalman filter
// 	numerator := om.initialSpeedStd*om.initialSpeedStd*speedMeanK + speedStdK*speedStdK*gpsSpeed
// 	denominator := om.initialSpeedStd*om.initialSpeedStd + speedStdK*speedStdK
// 	speedMean := numerator / denominator
// 	speedStdK = math.Sqrt(1 / (1/(om.initialSpeedStd*om.initialSpeedStd) + 1/(speedStdK*speedStdK)))
// 	return speedMean, speedStdK
// }

// // equation 3 in ref[1]
// func (om *OnlineMapMatchMHT) computEdgeTransitionProb(u, v da.Index, nj int) float64 {
// 	branch := make([]da.Index, 0, 4)

// 	om.graph.ForOutEdgesOf(u, func(_, nSegId da.Index, _ da.Index) {
// 		branch = append(branch, nSegId)
// 	})
// 	sumNej := 0.0
// 	for _, j := range branch {
// 		trans := float64(om.N.Get(int(u), int(j)))
// 		sumNej += trans
// 	}

// 	return (1.0 + float64(om.N.Get(int(u), int(v)))) / (sumNej + float64(nj))
// }

// // equation 20 in ref[1]
// //
// //	h(τ_{1:n}, \hat{v}_k ,σ_v,k ,t_k ) denotes the probability of the vehicle traveling further than e_n.
// //
// // ini dipakai untuk compute route prediction probability
// func (om *OnlineMapMatchMHT) computeHProb(tau []da.Index, speedMean, speedStd, deltaTime float64) float64 {
// 	tauLength := 0.0
// 	for _, eId := range tau {
// 		tauLength += om.getSegmentLength(eId)
// 	}
// 	fEdgeLength := om.getSegmentLength(tau[0])

// 	s := (math.Sqrt(3) * speedStd * deltaTime) / math.Pi
// 	out := (1.0 / fEdgeLength)

// 	f := func(x float64) float64 {
// 		numerator := speedMean*deltaTime - (tauLength - x)
// 		denominator := (math.Sqrt(3) * speedStd * deltaTime) / math.Pi
// 		expo := math.Exp(numerator / denominator)
// 		log := math.Log(expo + 1.0)
// 		return s * log
// 	}

// 	return out * (f(fEdgeLength) - f(0))
// }

// func (om *OnlineMapMatchMHT) projectAllCandidates(gps *da.GPSPoint, candidates []*ma.Candidate) {
// 	for _, cand := range candidates {
// 		gpsCoord := da.NewCoordinate(gps.Lat(), gps.Lon())
// 		bp, minDist, minDistr, segmentBearing := om.projectGpsToRoadSegment(gpsCoord, cand.GetSegmentId())
// 		cand.SetProjectedCoord(bp.GetLat(), bp.GetLon())
// 		cand.SetDist(minDist)
// 		cand.SetDistr(minDistr)
// 		cand.SetEdgeBearing(segmentBearing)
// 	}
// }

// func (om *OnlineMapMatchMHT) projectGpsToRoadSegment(gpsCoord da.Coordinate, eId da.Index) (da.Coordinate, float64, float64, float64) {
// 	eGeometry := om.rn.GetSegmentGeometry(eId)
// 	var (
// 		minDist, minDistr = math.MaxFloat64, math.MaxFloat64
// 		bp                da.Coordinate
// 	)
// 	segmentBearing := 0.0
// 	cumLength := 0.0

// 	for i := 0; i < len(eGeometry)-1; i++ {
// 		tail := eGeometry[i]
// 		head := eGeometry[i+1]
// 		projectedPoint := geo.ProjectPointOnSegment(
// 			tail,
// 			head,
// 			gpsCoord,
// 		)
// 		dist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(
// 			projectedPoint.GetLat(), projectedPoint.GetLon(),
// 			gpsCoord.GetLat(), gpsCoord.GetLon(),
// 		))

// 		tailToProjectedDist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(
// 			tail.GetLat(), tail.GetLon(),
// 			projectedPoint.GetLat(), projectedPoint.GetLon(),
// 		))

// 		distr := cumLength + tailToProjectedDist

// 		if dist < minDist {
// 			minDist = dist
// 			minDistr = distr
// 			bp = projectedPoint
// 			eInitialBearing := geo.BearingTo(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
// 			segmentBearing = eInitialBearing
// 		}

// 		cumLength += util.KilometerToMeter(geo.CalculateGreatCircleDistance(
// 			tail.GetLat(), tail.GetLon(),
// 			head.GetLat(), head.GetLon(),
// 		))
// 	}

// 	return bp, minDist, minDistr, segmentBearing
// }
