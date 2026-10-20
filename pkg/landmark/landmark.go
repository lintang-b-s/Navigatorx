// Package landmark provides landmark selection and ALT (A*, Landmarks, and Triangle inequality) preprocessing & query phase.
package landmark

import (
	"cmp"
	"errors"
	"fmt"
	"math"
	"slices"
	"sync"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Landmark[W util.Numeric] struct {
	lw        []W
	vlw       []W
	landmarks []da.Index
	n         da.Index
	k         da.Index
}

func NewLandmark[W util.Numeric]() *Landmark[W] {
	lm := &Landmark[W]{}
	lm.lw = []W{}
	lm.vlw = []W{}
	landmarks := make([]da.Index, 0)
	lm.landmarks = landmarks

	return lm
}

/*
[1] Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.

this is an implementation of planar landmark selection described in section 7 paper [1] versi 2

O(V*logV)
*/
func (lm *Landmark[W]) SelectLandmarksTwo(k int, graph *da.Graph) []da.Vertex {

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

	midLandmark := da.NewEmptyVertex()
	minMidDist := math.MaxFloat64
	for _, v := range ivs {
		// O(V)
		dist := geo.CalculateGreatCircleDistance(v.GetLat(), v.GetLon(),
			centerLat, centerLon)
		if dist < minMidDist {
			minMidDist = dist
			midLandmark = v
		}
	}

	vsCopy := make([]da.Vertex, n)
	copy(vsCopy, ivs)

	// mirip algoritma graham explore buat bikin convex hull
	// graham scan: sort Points by their polar angles around a p0 (bottomost point or rightmost & bottomost point if tie)
	// ini: sort Points by their initial bearing angles around a p0 (center point/coordinate)
	// karena geographic coordinate, sort by initial bearing angle (sudut clockwise antara garis yang menghubungkan titik pivot ke other point dan garis meridian)

	slices.SortFunc(vsCopy, func(a, b da.Vertex) int {
		ab := geo.BearingTo(midLandmark.GetLat(), midLandmark.GetLon(), a.GetLat(), a.GetLon())
		bb := geo.BearingTo(midLandmark.GetLat(), midLandmark.GetLon(), b.GetLat(), b.GetLon())
		return cmp.Compare(ab, bb)
	})

	pieSize := n / k

	for i := 0; i < k; i++ {
		// O(V)
		pie := vsCopy[i*pieSize : (i+1)*pieSize]
		if i == k-1 {
			pie = vsCopy[i*pieSize:]
		}

		terjauhId := 0
		maxDist := math.Inf(-1)
		for j, v := range pie {
			midToVDist := geo.CalculateGreatCircleDistance(midLandmark.GetLat(), midLandmark.GetLon(), v.GetLat(), v.GetLon())

			if midToVDist > maxDist {
				maxDist = midToVDist
				terjauhId = j
			}
		}

		farthestV := pie[terjauhId]
		landmarks = append(landmarks, farthestV)
	}

	return landmarks
}

type queryParam struct {
	il  int
	sid da.Index
}

func newQueryparam(il int, sid da.Index) queryParam {
	return queryParam{il, sid}
}

func (qp *queryParam) getIndex() int {
	return qp.il
}

func (qp *queryParam) getSid() da.Index {
	return qp.sid
}

type queryRet[W util.Numeric] struct {
	il  int
	sps []W
}

func newQueryRet[W util.Numeric](il int, sps []W) queryRet[W] {
	return queryRet[W]{il, sps}
}

func (qr *queryRet[W]) getIndex() int {
	return qr.il
}

func (qr *queryRet[W]) getSpCosts() []W {
	return qr.sps
}

// todo: reduce memory usage PreprocessALT()
// currently (7 oktober 2026), pakai diy_solo_semarang.osm.pbf PreprocessALT() peak htop RES/RSS ~ 2GB, hampir 20x ukuran file osmnya
// kayake bisa dibatesi jumlah goroutine worker nya... & coba cek pprof di PreprocessALT()

/*
[1] Goldberg, A.V. and Harrelson,  (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.

preprocessing phase of A*, landmark, and triangle inequality (ALT) described in [1]

time complexity of ALT preprocessing:

O((n+m)logn * k), m=number of edges,k=number of landmarks
*/
func (lm *Landmark[W]) PreprocessALT(k int, cf *met.TimeFunction[W], graph *da.Graph, logger *zap.Logger) error {
	if k > 64 {
		return errors.New("too much landmarks!, the maximum number of landmarks is 64. ")
	}
	n := da.Index(graph.NumberOfVertices())

	if n < da.Index(k) {
		return nil
	}

	logger.Sugar().Infof("computing landmarks.... number of landmarks: %v", k)
	lw := make([]W, da.Index(k)*n)
	landmarks := make([]da.Index, k)
	vlw := make([]W, n*da.Index(k))
	landmarksVertices := lm.SelectLandmarksTwo(k, graph)
	lm.k = da.Index(k)
	lm.n = n

	maxSearchSize := graph.NumberOfVertices()

	heapPool := sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.QueryKey, W](da.Index(maxSearchSize), da.ARRAY_STORAGE)
		},
	}

	dijkstraInChan := make(chan queryParam, landmarkChanSize)
	dijkstraRevInChan := make(chan queryParam, landmarkChanSize)

	dijkstraOutChan := make(chan queryRet[W], landmarkChanSize)
	dijkstraRevOutChan := make(chan queryRet[W], landmarkChanSize)

	wg := sync.WaitGroup{}

	calcDijkstra := func() {
		for qp := range dijkstraInChan {
			sid := qp.getSid()
			il := qp.getIndex()

			crpQuery := NewDijkstra(graph, cf, false) // O((m+n)logn). at most n items in pq , decrease/insert key operation at most m times, extractMin operation at most n times
			sps := crpQuery.ShortestPath(sid, &heapPool)
			dijkstraOutChan <- newQueryRet(il, sps)
		}
	}

	calcDijkstraRev := func() {
		for qp := range dijkstraRevInChan {
			sid := qp.getSid()
			il := qp.getIndex()
			crpQuery := NewDijkstra(graph, cf, true) // O((m+n)logn). at most n items in pq , decrease/insert key operation at most m times, extractMin operation at most n times

			sps := crpQuery.ShortestPath(sid, &heapPool)
			dijkstraRevOutChan <- newQueryRet(il, sps)
		}
	}

	go func() {
		for res := range dijkstraRevOutChan {
			il := res.getIndex()
			sps := res.getSpCosts()
			for v := 0; v < int(n); v++ {
				vlw[v*k+il] = sps[v]
			}
			wg.Done()
		}
	}()

	go func() {
		for res := range dijkstraOutChan {
			il := res.getIndex()
			sps := res.getSpCosts()
			copy(lw[il*int(n):(il+1)*int(n)], sps)
			wg.Done()
		}
	}()

	for i := 0; i < WORKERS; i++ {
		go calcDijkstra()
		go calcDijkstraRev()
	}

	for i := 0; i < k; i++ {
		landmark := landmarksVertices[i]
		sid := landmark.GetID()
		landmarks[i] = sid
		wg.Add(2)
		dijkstraInChan <- newQueryparam(i, sid)
		dijkstraRevInChan <- newQueryparam(i, sid)
	}

	close(dijkstraInChan)
	close(dijkstraRevInChan)

	wg.Wait()
	close(dijkstraOutChan)
	close(dijkstraRevOutChan)

	lm.landmarks = landmarks
	lm.lw = lw
	lm.vlw = vlw

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
				w[i][j] = lm.vlw[i*lm.k+j]
			}
		}
	}
	return w
}

/*
[1] Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
[2] Bast, H. et al. (2016) “Route Planning in Transportation Networks,” in L.
Kliemann and P. Sanders (eds.) Algorithm Engineering: Selected Results and
Surveys. Cham: Springer International Publishing, pp. 19–80. Available at:
https://doi.org/10.1007/978-3-319-49487-6_2.
[3] Goldberg, A. and Harrelson, C. (2004) “Computing the Shortest Path: A* Search Meets Graph Theory.” Available at: https://www.microsoft.com/en-us/research/publication/computing-the-shortest-path-a-search-meets-graph-theory/ (Accessed: February 9, 2026).

implementation of computing tighest lower bound of A*, landmarks, and triangle inequality, section 6 Computing Lower Bounds in [1] or section 2.2 ALT in [2]
misal L adalah set of landmarks, dist(v,w) adalah shortest path cost dari vertex v ke vertex w
ComputeTighestLowerBound compute h(u)=max_{l in L}{dist(l,t)-dist(l,u), dist(u,l)-dist(t,l)}
fungsi potential/heuristik  h(u) meiliki sifat konsisten/feasible [3]

activeLandmarks berisi list index dari active query landmark (list index dari lm.landmarks)
*/
//go:inline
func (lm *Landmark[W]) ComputeTighestLowerBound(u, t da.Index, activeLandmarks []da.Index) W {
	// O(k), k = number of active landmarks
	tighestLowerBound := -util.Infinity[W]()

	for i := 0; i < len(activeLandmarks); i++ {
		lid := activeLandmarks[i]
		lb := lm.ComputeLowerBound(lid, u, t)
		tighestLowerBound = max(tighestLowerBound, lb)
	}

	return tighestLowerBound
}

// ComputeLowerBound compute max(dist(l,t)-dist(l,u), dist(u,l)-dist(t,l))
//
//go:inline
func (lm *Landmark[W]) ComputeLowerBound(lid, u, t da.Index) W {
	uToLandmark := lm.vlw[u*lm.k+lid]
	tToLandmark := lm.vlw[t*lm.k+lid]

	landmarkToT := lm.lw[lid*lm.n+t]
	landmarkToU := lm.lw[lid*lm.n+u]

	lbOne := uToLandmark - tToLandmark
	lbTwo := landmarkToT - landmarkToU
	return max(lbOne, lbTwo)
}

type activeLandmark[W util.Numeric] struct {
	i  da.Index
	lb W
}

func newActiveLandmark[W util.Numeric](i da.Index, lb W) activeLandmark[W] {
	return activeLandmark[W]{i, lb}
}

const (
	activeLandmarkSize = 2
)

/*
https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf

Use only an active subset:  (page 22)
– prefer landmarks that give the best lower bound on dist(s, t).
*/
func (lm *Landmark[W]) SelectBestQueryLandmarks(s, t da.Index) []da.Index {

	bestLandmarks := make([]da.Index, 0, activeLandmarkSize)

	lowerBounds := make([]activeLandmark[W], len(lm.landmarks))
	for i := 0; i < len(lm.landmarks); i++ {
		lb := lm.ComputeLowerBound(da.Index(i), s, t)
		lowerBounds[i] = newActiveLandmark(da.Index(i), lb)
	}

	// O(k* logk), k = number of landmarks
	slices.SortFunc(lowerBounds, func(a activeLandmark[W], b activeLandmark[W]) int {
		return cmp.Compare(b.lb, a.lb)
	})

	lbs := lowerBounds[:min(len(lowerBounds), activeLandmarkSize)]
	for _, v := range lbs {
		bestLandmarks = append(bestLandmarks, v.i)
	}

	return bestLandmarks
}

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
//go:inline
func (lm *Landmark[W]) ComputeTighestConsistentLowerBound(u, s, t da.Index, activeLandmarks []da.Index, pirt W, forw bool) W {
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

func (lm *Landmark[W]) GetLandmarkVIds() []da.Index {
	return lm.landmarks
}

func (lm *Landmark[W]) GetLandmarkVWeights() [][]W {
	return lm.matrix(lm.k, lm.n, true)
}

func (lm *Landmark[W]) GetVerticesLandmarkWeights() [][]W {
	return lm.matrix(lm.n, lm.k, false)
}

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
