// Package landmark provides landmark selection and ALT (A*, Landmarks, and Triangle inequality) preprocessing & query phase.
package landmark

import (
	"bufio"
	"errors"
	"fmt"
	"math"
	"sort"
	"sync"
	"sync/atomic"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Landmark[W util.RoutingNumber] struct {
	lw        atomic.Pointer[landmarkTable[W]]
	vlw       atomic.Pointer[landmarkTable[W]]
	landmarks atomic.Pointer[[]da.Index]
}

type landmarkTable[W util.RoutingNumber] struct {
	values []W
	rows   da.Index
	cols   da.Index
}

func newLandmarkTable[W util.RoutingNumber](rows, cols da.Index) *landmarkTable[W] {
	return &landmarkTable[W]{values: make([]W, rows*cols), rows: rows, cols: cols}
}

func (table *landmarkTable[W]) row(row da.Index) []W {
	start := row * table.cols
	return table.values[start : start+table.cols]
}

func (table *landmarkTable[W]) at(row da.Index, col da.Index) W {
	return table.values[row*table.cols+col]
}

func (table *landmarkTable[W]) matrix() [][]W {
	rows := make([][]W, table.rows)
	for i := range rows {
		rows[i] = table.row(da.Index(i))
	}
	return rows
}

func NewLandmark[W util.RoutingNumber]() *Landmark[W] {
	lm := &Landmark[W]{}
	lm.lw.Store(newLandmarkTable[W](0, 0))
	lm.vlw.Store(newLandmarkTable[W](0, 0))
	landmarks := make([]da.Index, 0)
	lm.landmarks.Store(&landmarks)
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
		if v.GetLon() > maxLon {
			maxLon = v.GetLon()
		}
		if v.GetLon() < minLon {
			minLon = v.GetLon()
		}

		if v.GetLat() > maxLat {
			maxLat = v.GetLat()
		}
		if v.GetLat() < minLat {
			minLat = v.GetLat()
		}
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
		validLandmark := graph.GetOutDegree(v.GetID()) != 0 && graph.GetInDegree(v.GetID()) != 0
		if dist < minMidDist && validLandmark {
			minMidDist = dist
			midLandmark = v
		}
	}

	vs := graph.GetVertices()
	vsCopy := make([]da.Vertex, n)
	copy(vsCopy, vs)

	// mirip algoritma graham scan buat bikin convex hull
	// graham scan: sort Points by their polar angles around a p0 (bottomost point or rightmost & bottomost point if tie)
	// ini: sort Points by their initial bearing angles around a p0 (center point/coordinate)
	// karena geographic coordinate, sort by initial bearing angle (sudut clockwise antara garis yang menghubungkan titik pivot ke other point dan garis meridian)

	sort.Slice(vsCopy, func(i, j int) bool { // O(V * logV)
		return geo.BearingTo(midLandmark.GetLat(), midLandmark.GetLon(), vsCopy[j].GetLat(), vsCopy[j].GetLon()) <
			geo.BearingTo(midLandmark.GetLat(), midLandmark.GetLon(), vsCopy[i].GetLat(), vsCopy[i].GetLon())
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
			validLandmark := graph.GetOutDegree(v.GetID()) != 0 && graph.GetInDegree(v.GetID()) != 0

			if midToVDist > maxDist && validLandmark {
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

type queryRet[W util.RoutingNumber] struct {
	il  int
	sps []W
}

func newQueryRet[W util.RoutingNumber](il int, sps []W) queryRet[W] {
	return queryRet[W]{il, sps}
}

func (qr *queryRet[W]) getIndex() int {
	return qr.il
}

func (qr *queryRet[W]) getSpCosts() []W {
	return qr.sps
}

/*
[1] Goldberg, A.V. and Harrelson,  (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.

preprocessing phase of A*, landmark, and triangle inequality (ALT) described in [1]

time complexity of ALT preprocessing:

O(m*logm * k), m=number of edges,k=number of landmarks
*/
func (lm *Landmark[W]) PreprocessALT(k int, m *metrics.Metric[W], graph *da.Graph, logger *zap.Logger) error {
	if k > 64 {
		return errors.New("too much landmarks!, the maximum number of landmarks is 64. ")
	}
	n := da.Index(graph.NumberOfVertices())

	if n < da.Index(k) {
		lm.lw.Store(newLandmarkTable[W](0, 0))
		lm.vlw.Store(newLandmarkTable[W](0, 0))
		landmarks := make([]da.Index, 0)
		lm.landmarks.Store(&landmarks)
		return nil
	}

	logger.Info("computing landmarks....")
	lw := newLandmarkTable[W](da.Index(k), n)
	landmarks := make([]da.Index, k)
	vlw := newLandmarkTable[W](n, da.Index(k))
	landmarksVertices := lm.SelectLandmarksTwo(k, graph)

	maxSearchSize := graph.NumberOfEdges()
	maxEdgesInCell := graph.GetMaxEdgesInCell()

	heapPool := sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey, W](uint32(maxSearchSize), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
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

			crpQuery := NewDijkstra(graph, m, false) // O((m+n)logn). at most n items in pq , decrease/insert key operation at most m times, extractMin operation at most n times
			sps := crpQuery.ShortestPath(sid, &heapPool)
			dijkstraOutChan <- newQueryRet(il, sps)
		}
	}

	calcDijkstraRev := func() {
		for qp := range dijkstraRevInChan {
			sid := qp.getSid()
			il := qp.getIndex()
			crpQuery := NewDijkstra(graph, m, true) // O((m+n)logn). at most n items in pq , decrease/insert key operation at most m times, extractMin operation at most n times

			sps := crpQuery.ShortestPath(sid, &heapPool)
			dijkstraRevOutChan <- newQueryRet(il, sps)
		}
	}

	go func() {
		for res := range dijkstraRevOutChan {
			il := res.getIndex()
			sps := res.getSpCosts()
			for v := 0; v < int(n); v++ {
				vlw.values[v*k+il] = sps[v]
			}
			wg.Done()
		}
	}()

	go func() {
		for res := range dijkstraOutChan {
			il := res.getIndex()
			sps := res.getSpCosts()
			copy(lw.row(da.Index(il)), sps)
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

	lm.landmarks.Store(&landmarks)
	lm.lw.Store(lw)
	lm.vlw.Store(vlw)

	logger.Info("done computing landmarks....")
	return nil
}

/*
[1] Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
[2] Bast, H. et al. (2016) “Route Planning in Transportation Networks,” in L.
Kliemann and P. Sanders (eds.) Algorithm Engineering: Selected Results and
Surveys. Cham: Springer International Publishing, pp. 19–80. Available at:
https://doi.org/10.1007/978-3-319-49487-6_2.
[3] Goldberg, A. and Harrelson, C. (2004) “Computing the Shortest Path: A* Search Meets Graph Theory.” Available at: https://www.microsoft.com/en-us/research/publication/computing-the-shortest-path-a-search-meets-graph-theory/ (Accessed: February 9, 2026).

implementation of computing tighest lower bound of A*, landmarks, and triangle inequality, 6 Computing Lower Bounds in [1] or section 2.2 ALT in [2]
misal L adalah set of landmarks, dist(v,w) adalah shortest path cost dari vertex v ke vertex w
FindTighestLowerBound compute h(v)=max_{l in L}{dist(l,t)-dist(l,v), dist(v,l)-dist(t,l)}
fungsi potential/heuristik  h(v) meiliki sifat konsisten/feasible [3]

activeLandmarks berisi list index dari active query landmark (list index dari lm.landmarks)
*/
func (lm *Landmark[W]) FindTighestLowerBound(u, t da.Index, activeLandmarks []da.Index) W {
	// O(k), k = number of landmarks
	tighestLowerBound := -util.Infinity[W]()
	vlw := lm.vlw.Load()
	lw := lm.lw.Load()
	for i := 0; i < len(activeLandmarks); i++ {
		landmarkId := activeLandmarks[i]

		uToLandmark := vlw.at(u, landmarkId)
		tToLandmark := vlw.at(t, landmarkId)
		tighestLowerBound = max(tighestLowerBound, uToLandmark-tToLandmark)

		landmarkToT := lw.at(landmarkId, t)
		landmarkToU := lw.at(landmarkId, u)
		tighestLowerBound = max(tighestLowerBound, landmarkToT-landmarkToU)
	}

	return tighestLowerBound
}

type activeLandmark[W util.RoutingNumber] struct {
	i  da.Index
	lb W
}

func newActiveLandmark[W util.RoutingNumber](i da.Index, lb W) activeLandmark[W] {
	return activeLandmark[W]{i, lb}
}

const (
	activeLandmarkSize = 2
)

/*
https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf

Use only an active subset:  (page 6)
– prefer landmarks that give the best lower bound on dist(s, t).
*/
func (lm *Landmark[W]) SelectBestQueryLandmarks(s, t da.Index) []da.Index {
	landmarks := *lm.landmarks.Load()
	bestLandmarks := make([]da.Index, 0, activeLandmarkSize)

	oriLandmarks := make([]da.Index, len(landmarks))
	for i := 0; i < len(landmarks); i++ {
		oriLandmarks[i] = da.Index(i)
	}

	lowerBounds := make([]activeLandmark[W], len(landmarks))
	for i := 0; i < len(landmarks); i++ {
		lid := landmarks[i]
		lb, _ := lm.FindTighestConsistentLowerBound(lid, s, t, oriLandmarks)
		lowerBounds[i] = newActiveLandmark(da.Index(i), lb)
	}
	// O(k* logk), k = number of landmarks
	sort.Slice(lowerBounds, func(i, j int) bool {
		return lowerBounds[i].lb > lowerBounds[j].lb
	})

	lbs := lowerBounds[:util.MinInt(len(lowerBounds), activeLandmarkSize)]
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
func (lm *Landmark[W]) FindTighestConsistentLowerBound(u, s, t da.Index, activeLandmarks []da.Index) (W, W) {
	pifu := lm.FindTighestLowerBound(u, t, activeLandmarks) // consistent lower bound dari dist(u,t)
	piru := lm.FindTighestLowerBound(s, u, activeLandmarks) // consistent lower bound dari dist(s,u)
	pirt := lm.FindTighestLowerBound(s, t, activeLandmarks) // consistent lower bound dari dist(s, t)

	beta := pirt // h_f(s)
	pfu := max(pifu, pirt-piru+beta)
	pru := -pfu

	return pfu, pru
}

func (lm *Landmark[W]) GetLandmarkVId(i da.Index) da.Index {
	return (*lm.landmarks.Load())[i]
}

func (lm *Landmark[W]) GetLandmarkVIds() []da.Index {
	return *lm.landmarks.Load()
}

func (lm *Landmark[W]) GetLandmarkVWeights() [][]W {
	return lm.lw.Load().matrix()
}

func (lm *Landmark[W]) GetVerticesLandmarkWeights() [][]W {
	return lm.vlw.Load().matrix()
}

func (lm *Landmark[W]) UpdateLandmarks(landmarkFilePath string, readBuf *bufio.Reader) error {
	newLandmark, err := ReadLandmark[W](landmarkFilePath, readBuf)
	if err != nil {
		return fmt.Errorf("UpdateLandmarks: failed to read new precalculated landmark distances: %v: %w", err, err)
	}

	lm.landmarks.Store(newLandmark.landmarks.Load())
	lm.lw.Store(newLandmark.lw.Load())
	lm.vlw.Store(newLandmark.vlw.Load())
	return nil
}
