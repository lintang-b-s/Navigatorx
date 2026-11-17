// Package landmark provides landmark selection and ALT (A*, Landmarks, and Triangle inequality) preprocessing & query phase.
package landmark

import (
	"errors"
	"fmt"
	"math"
	"sync"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/lintang-b-s/radixheap"
	"go.uber.org/zap"
)

// Landmark represents ALT landmark precomputed distances.
type Landmark[W util.Numeric] struct {
	lw        []W // shortest path costs from landmarks to vertices
	vlw       []W // shortest path costs from vertices to landmarks
	landmarks []da.Index
	n         da.Index
	k         da.Index
	inf       W
}

// NewLandmark creates and returns ALT landmark precomputed distances.
func NewLandmark[W util.Numeric]() *Landmark[W] {
	lm := &Landmark[W]{}
	lm.lw = []W{}
	lm.vlw = []W{}
	landmarks := make([]da.Index, 0)
	lm.landmarks = landmarks
	lm.inf = util.Infinity[W]()

	return lm
}

// todo: pas run ./bin/customizer --region=jateng_jabar peak memory ~7.4GB, coba cek pprof heap && optimize...
// DONE sekarang setelah pakai radixheap cuma 4.5 GB

// SelectLandmarks selects landmark vertices for the ALT (A*, Landmarks, and Triangle Inequality) Algorithm.
// [1] Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
// this is an implementation of planar landmark selection described in section 7 paper [1]
// O((n+m)logn). n,m=number of vertices, number of edges
func (lm *Landmark[W]) SelectLandmarks(k int, graph *da.Graph, cf *met.TimeFunction[W], heapPool *sync.Pool) []da.Vertex {

	landmarks := make([]da.Vertex, 0, k)
	ivs := graph.GetVertices()

	minLon := math.MaxFloat64
	maxLon := math.Inf(-1)
	minLat := math.MaxFloat64
	maxLat := math.Inf(-1)
	for _, v := range ivs {
		maxLon = max(maxLon, v.GetLon())
		maxLat = max(maxLat, v.GetLat())
		minLon = min(minLon, v.GetLon())
		minLat = min(minLat, v.GetLat())
	}

	centerLat := (maxLat + minLat) / 2.0
	centerLon := (maxLon + minLon) / 2.0
	n := graph.NumberOfVertices()

	// first, find a vertex mid closest to the center of the embedding.
	midLandmark := da.NewEmptyVertex()
	minMidDist := math.MaxFloat64
	for _, v := range ivs {
		// O(n)
		dist := geo.CalculateGreatCircleDistance(v.GetLat(), v.GetLon(),
			centerLat, centerLon)
		if dist < minMidDist {
			minMidDist = dist
			midLandmark = v
		}
	}

	type cv struct {
		v       da.Vertex
		bearing float64
		dist    W
	}

	cvs := make([]cv, len(ivs))

	midv := midLandmark.GetId()
	cost := make([]W, n)
	inf := lm.inf // duration/travel time
	for v := range n {
		cost[v] = inf
	}
	dijk := NewDijkstra(graph, cf, false)
	dijk.ShortestPath(midv, heapPool, cost) // O((n+m)logn)

	const speed float64 = 55.5556 // 200 km/h
	for i, v := range ivs {
		b := geo.BearingTo(midLandmark.GetLat(), midLandmark.GetLon(), v.GetLat(), v.GetLon())
		vId := v.GetId()
		var d W
		if cost[vId] == inf {
			dgc := geo.CalculateGreatCircleDistance(midLandmark.GetLat(), midLandmark.GetLon(), v.GetLat(), v.GetLon())
			d = W(util.KilometerToMeter(dgc))
		} else {
			d = W(float64(cost[vId]) * speed)
		}
		cvs[i] = cv{v, b, d}
	}

	// Divide the embedding into k pie-slice sectors centered at vertex mid.
	// each sector containing approximately the same number of vertices.
	pieSize := n / k
	kth := pieSize
	for i := 1; i <= k; i++ { // O(nk)
		var start int
		nth := pieSize * i
		start = nth - pieSize

		if i < k {
			// use fast k-th order statistics algorithm, RandomizedSelect with expected runtime O(n).
			// after this RandomizedSelect(), because of the partitioning & random pivot selection of randomized quicksort,
			// all vertices cvs[start:nth] bearings will be less or equal than the vertex cvs[nth] bearing
			util.RandomizedSelect(cvs, start, n-1, kth, func(left, right int) bool {
				return cvs[left].bearing < cvs[right].bearing
			}) // expected runtime O(n)
		}

		// For each sector, pick a vertex farthest away from the center (vertex mid).
		pie := cvs[start:nth]
		farthestId := 0
		maxDist := -lm.inf
		for j, vv := range pie { // O(n/k)
			midToVDist := vv.dist
			if midToVDist > maxDist {
				maxDist = midToVDist
				farthestId = j
			}
		}

		farthestV := pie[farthestId]
		landmarks = append(landmarks, farthestV.v)
	}

	return landmarks
}

type queryParam struct {
	i da.Index
	s da.Index
}

func newQueryparam(i, s da.Index) queryParam {
	return queryParam{i, s}
}

// PreprocessALT computes ALT (A*, Landmarks, and Triangle Inequality ) Preprocessing Algorithm
// [1] Goldberg, A.V. and Harrelson,  (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
// preprocessing phase of A*, landmark, and triangle inequality (ALT) described in [1]
// time complexity of ALT preprocessing:
// O((n+m)logn * k), n=number of vertices, m=number of edges,k=number of landmarks
func (lm *Landmark[W]) PreprocessALT(k int, cf *met.TimeFunction[W], graph *da.Graph, logger *zap.Logger) error {
	if k > 64 {
		return errors.New("too much landmarks!, the maximum number of landmarks is 64. ")
	}
	n := da.Index(graph.NumberOfVertices())

	if n < da.Index(k) {
		lm.k = 0
		return nil
	}

	isRoadNetwork := graph.IsRoadNetworkGraph()
	heapPool := &sync.Pool{
		New: func() any {
			if isRoadNetwork {
				return radixheap.NewRadixHeap[uint32, da.Index]()
			}
			return da.NewQueryHeap[da.Index, W](n, n, da.ArrayStorageMode)
		},
	}

	logger.Sugar().Infof("number of landmarks: %v, selecting best landmarks....", k)
	lm.lw = make([]W, da.Index(k)*n)
	lm.landmarks = make([]da.Index, k)
	lm.vlw = make([]W, n*da.Index(k))
	landmarksVertices := lm.SelectLandmarks(k, graph, cf, heapPool)
	lm.k = da.Index(k)
	lm.n = n

	inf := util.Infinity[W]()
	for l := da.Index(0); l < da.Index(k); l++ {
		for v := da.Index(0); v < n; v++ {
			lm.lw[l*n+v] = inf
			lm.vlw[l*n+v] = inf
		}
	}

	dijkstraInChan := make(chan queryParam, landmarkChanSize)
	dijkstraRevInChan := make(chan queryParam, landmarkChanSize)

	wg := &sync.WaitGroup{}

	calcDijkstra := func(dijkstraInChan chan queryParam, graph *da.Graph, heapPool *sync.Pool, cf *met.TimeFunction[W], lw []W) {
		for qp := range dijkstraInChan {
			s := qp.s
			i := qp.i

			dijk := NewDijkstra(graph, cf, false) // O((m+n)logn). at most n items in pq , decrease-key operation at most m times, extractMin/insert operation at most n times
			dijk.ShortestPath(s, heapPool, lw[i*n:(i+1)*n])
			wg.Done()
		}
	}

	calcDijkstraRev := func(dijkstraRevInChan chan queryParam, graph *da.Graph, heapPool *sync.Pool, cf *met.TimeFunction[W], vlw []W) {
		for qp := range dijkstraRevInChan {
			s := qp.s
			i := qp.i

			dijk := NewDijkstra(graph, cf, true) // O((m+n)logn). at most n items in pq , decrease-key operation at most m times, extractMin/insert operation at most n times
			dijk.ShortestPath(s, heapPool, vlw[i*n:(i+1)*n])
			wg.Done()
		}
	}

	logger.Sugar().Infof("computing landmarks distances....")

	for i := 0; i < WORKERS; i++ {
		go calcDijkstra(dijkstraInChan, graph, heapPool, cf, lm.lw)
		go calcDijkstraRev(dijkstraRevInChan, graph, heapPool, cf, lm.vlw)
	}

	for i := da.Index(0); i < da.Index(k); i++ {
		landmark := landmarksVertices[i]
		s := landmark.GetId()
		lm.landmarks[i] = s
		wg.Add(2)
		dijkstraInChan <- newQueryparam(i, s)
		dijkstraRevInChan <- newQueryparam(i, s)
	}

	close(dijkstraInChan)
	close(dijkstraRevInChan)

	wg.Wait()

	logger.Info("done computing landmarks....")
	return nil
}

func (lm *Landmark[W]) matrix(r, c da.Index, lw bool) [][]W {
	w := make([][]W, r)
	for i := da.Index(0); i < r; i++ {
		w[i] = make([]W, c)
		for j := da.Index(0); j < c; j++ {
			if lw {
				w[i][j] = lm.lw[i*lm.n+j]
			} else {
				w[i][j] = lm.vlw[j*lm.n+i]
			}
		}
	}
	return w
}

/*
ComputeTighestLowerBound computes ALT heuristic h(u)  (tighest lower bound of shortest path distance from u to t dist(u,t))
[1] Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
[2] Bast, H. et al. (2016) “Route Planning in Transportation Networks,” in L.
Kliemann and P. Sanders (eds.) Algorithm Engineering: Selected Results and
Surveys. Cham: Springer International Publishing, pp. 19–80. Available at:
https://doi.org/10.1007/978-3-319-49487-6_2.
[3] Goldberg, A. and Harrelson, C. (2004) “Computing the Shortest Path: A* Search Meets Graph Theory.” Available at: https://www.microsoft.com/en-us/research/publication/computing-the-shortest-path-a-search-meets-graph-theory/ (Accessed: February 9, 2026).

implementation of computing tighest lower bound of A*, landmarks, and triangle inequality, section 6 Computing Lower Bounds in [1] or section 2.2 ALT in [2]
misal L adalah set of landmarks, dist(v,w) adalah shortest path cost dari vertex v ke vertex w
ComputeTighestLowerBound compute h(u)=max_{l in L}{dist(l,t)-dist(l,u), dist(u,l)-dist(t,l)} potential/heuristics function  h(u) has property consistent/feasible and admissible [3].

activeLandmarks berisi list index dari active query landmark (list index dari lm.landmarks)

O(1)
*/
func (lm *Landmark[W]) ComputeTighestLowerBound(u, t da.Index, activeLandmarks [2]da.Index) W {
	if lm.k == 0 {
		return 0
	}
	l := activeLandmarks[0]
	lbOne := lm.ComputeLowerBound(l, u, t)
	l = activeLandmarks[1]
	lbTwo := lm.ComputeLowerBound(l, u, t)

	return max(lbOne, lbTwo)
}

// ComputeLowerBound compute h(u)=max(dist(l,t)-dist(l,u), dist(u,l)-dist(t,l))
// [1] Goldberg, A. and Harrelson, C. (2004) “Computing the Shortest Path: A* Search Meets Graph Theory.” Available at: https://www.microsoft.com/en-us/research/publication/computing-the-shortest-path-a-search-meets-graph-theory/ (Accessed: February 9, 2026).
// [2] Cormen, T.H. et al. (2009) Introduction to Algorithms. 3th ed. Cambridge, MA, USA: MIT Press
// potential/heuristics function  h(u) has property consistent/feasible and admissible [1].
// consistent means that the modified edge weight l'(u,v) = l(u,v) + h(v) - h(u) >= 0 for all edge (u,v) in E.
// which means that running A* with consistent heuristic equivalent to running dijkstra with this nonnegative modified edge weight l'(u,v).
// by lemma reweighting doesnt changes the shortest path like in Johnson Algorithm, in lemma 25.1 ref[2],
// the shortest path cost returned by A* with consistent heuristic equivalent to shortest path cost using nonnegative edge weight l(u,v).
// admissible means that this heuristics function h(u) is an lower bound of shortest path distance from u to t dist(u,t). h(u) <= dist(u,t).
func (lm *Landmark[W]) ComputeLowerBound(l, u, t da.Index) W {
	uToLandmark := lm.vlw[l*lm.n+u]
	landmarkToU := lm.lw[l*lm.n+u]

	tToLandmark := lm.vlw[l*lm.n+t]
	landmarkToT := lm.lw[l*lm.n+t]

	lbOne := uToLandmark - tToLandmark
	lbTwo := landmarkToT - landmarkToU
	return max(lbOne, lbTwo)
}

/*
SelectBestQueryLandmarks selects best query landmarks.

https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf

Use only an active subset:  (page 22)
– prefer landmarks that give the best (tighest) lower bound on shortest path distance from s to t dist(s, t).
O(k)
*/
func (lm *Landmark[W]) SelectBestQueryLandmarks(s, t da.Index) [2]da.Index {

	k := len(lm.landmarks)
	if k == 0 {
		return [2]da.Index{0, 0}
	}

	// best landmark that gives tighest lower bound on dist(s,t)
	lmOne := da.Index(0)
	lbOne := lm.ComputeLowerBound(0, s, t)

	if k == 1 {
		return [2]da.Index{lmOne, 0}
	}

	// second (2nd) best landmark that gives second (2nd) tighest lower bound on dist(s,t)
	lmTwo := da.Index(1)
	lbTwo := lm.ComputeLowerBound(1, s, t)

	if lbTwo > lbOne {
		lmOne, lmTwo = lmTwo, lmOne
		lbOne, lbTwo = lbTwo, lbOne
	}

	for i := 2; i < k; i++ {
		l := da.Index(i)
		lb := lm.ComputeLowerBound(l, s, t)
		if lb > lbOne {
			// this landmark l gives tigher lower bound on dist(s,t) than the current best landmark.
			// update the best landmark
			lmOne = l
			lbOne = lb
			// update the second (2nd) best landmark
			lmTwo = lmOne
			lbTwo = lbOne
		} else if lb > lbTwo {
			// this landmark gives tigher lower bound on dist(s,t) than current second (2nd) best landmark, but it is not better than the best landmark.
			lmTwo = l
			lbTwo = lb
		}
	}

	return [2]da.Index{lmOne, lmTwo}
}

// ComputeTighestConsistentLowerBound computes heuristic function for bidirectional ALT (A* Seach, Landmarks, and Triangle Inequality) Algorithm.
/*
[1] Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
[2] https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf
[3] Ikeda, T. et al. (1994) ‘A fast algorithm for finding better routes by AI search techniques’, in Proceedings of VNIS’94 - 1994 Vehicle Navigation and Information Systems Conference, pp. 291–296. Available at: https://doi.org/10.1109/VNIS.1994.396824.
[4] https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf

implementation of consistent/feasible potential function in 5.2 Consistent Approach ref [1]
calc pi_f(v)=max(h_f(v), h_r(t)-h_r(v)+beta) untuk forward search and pi_r(v)=-pi_f(v) untuk backward search
kita disini pakai beta=h_f(s)
h_f(u) adalah consistent lower bound dari sp cost dari u ke t
h_r(u) adalah consistent lower bound dari sp cost dari s ke u
*/
func (lm *Landmark[W]) ComputeTighestConsistentLowerBound(u, s, t da.Index, activeLandmarks [2]da.Index, pirt W, forw bool) W {
	pifu := lm.ComputeTighestLowerBound(u, t, activeLandmarks) // consistent lower bound dari dist(u,t)
	piru := lm.ComputeTighestLowerBound(s, u, activeLandmarks) // consistent lower bound dari dist(s,u)

	beta := pirt // h_f(s)
	pfu := max(pifu, pirt-piru+beta)
	pru := -pfu

	if forw {
		return pfu
	}
	return pru
}

// GetLandmarkVIds returns landmark vertex ids.
func (lm *Landmark[W]) GetLandmarkVIds() []da.Index {
	return lm.landmarks
}

// GetLandmarkVWeights returns precomputed landmarks to vertices weights.
func (lm *Landmark[W]) GetLandmarkVWeights() [][]W {
	return lm.matrix(lm.k, lm.n, true)
}

// GetVerticesLandmarkWeights returns vertices to landmarks weights.
func (lm *Landmark[W]) GetVerticesLandmarkWeights() [][]W {
	return lm.matrix(lm.n, lm.k, false)
}

// UpdateLandmarks updates landmarks.
func (lm *Landmark[W]) UpdateLandmarks(landmarkFilePath string) error {
	newLandmark, err := ReadLandmark[W](landmarkFilePath)
	if err != nil {
		return fmt.Errorf("UpdateLandmarks: failed to read new precalculated landmark distances: %v: %w", err, err)
	}

	lmLm := newLandmark.landmarks
	lmLw := newLandmark.lw
	lmVlw := newLandmark.vlw
	lm.landmarks = lmLm
	lm.lw = lmLw
	lm.vlw = lmVlw
	lm.n = newLandmark.n
	lm.k = newLandmark.k

	return nil
}
