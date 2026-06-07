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

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Landmark struct {
	lw        atomic.Value // distance from each landmarks to every vertices in graph
	vlw       atomic.Value // distance from all vertices to each landmarks
	landmarks atomic.Value // landmark vertex ids

}

type landmarkTable struct {
	values []float64 // values[row*cols+col] = values2D[row][col]. lw: rows=landmarks, cols=vertices
	rows   da.Index
	cols   da.Index
}

func newLandmarkTable(rows, cols da.Index) *landmarkTable {
	return &landmarkTable{values: make([]float64, rows*cols), rows: rows, cols: cols}
}

func (table *landmarkTable) row(row da.Index) []float64 {
	start := row * table.cols
	return table.values[start : start+table.cols]
}

func (table *landmarkTable) at(row da.Index, col da.Index) float64 {
	return table.values[row*table.cols+col]
}

func (table *landmarkTable) matrix() [][]float64 {
	rows := make([][]float64, table.rows)
	for i := range rows {
		rows[i] = append([]float64(nil), table.row(da.Index(i))...)
	}
	return rows
}

func NewLandmark() *Landmark {
	lm := &Landmark{}
	lm.lw.Store(newLandmarkTable(0, 0))
	lm.vlw.Store(newLandmarkTable(0, 0))
	landmarks := make([]da.Index, 0)
	lm.landmarks.Store(landmarks)
	return lm
}

/*
[1] Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.

this is an implementation of planar landmark selection described in section 7 paper [1] versi 2

O(V*logV)
*/
func (lm *Landmark) SelectLandmarksTwo(k int, graph *da.Graph) []da.Vertex {

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

type queryRet struct {
	il  int
	sps []float64
}

func newQueryRet(il int, sps []float64) queryRet {
	return queryRet{il, sps}
}

func (qr *queryRet) getIndex() int {
	return qr.il
}

func (qr *queryRet) getSpCosts() []float64 {
	return qr.sps
}

/*
[1] Goldberg, A.V. and Harrelson,  (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.

preprocessing phase of A*, landmark, and triangle inequality (ALT) described in [1]

time complexity of ALT preprocessing:

O(m*logm * k), m=number of edges,k=number of landmarks
*/
func (lm *Landmark) PreprocessALT(k int, m *metrics.Metric, graph *da.Graph, logger *zap.Logger) error {
	if k > 64 {
		return errors.New("too much landmarks!, the maximum number of landmarks is 64. ")
	}
	n := da.Index(graph.NumberOfVertices())

	if n < da.Index(k) {
		lm.lw.Store(newLandmarkTable(0, 0))
		lm.vlw.Store(newLandmarkTable(0, 0))
		lm.landmarks.Store(make([]da.Index, 0))
		return nil
	}

	logger.Info("computing landmarks....")
	lw := newLandmarkTable(da.Index(k), n)
	landmarks := make([]da.Index, k)
	vlw := newLandmarkTable(n, da.Index(k))
	landmarksVertices := lm.SelectLandmarksTwo(k, graph)

	maxSearchSize := graph.NumberOfEdges()
	maxEdgesInCell := graph.GetMaxEdgesInCell()

	heapPool := sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](uint32(maxSearchSize), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	dijkstraInChan := make(chan queryParam, landmarkChanSize)
	dijkstraRevInChan := make(chan queryParam, landmarkChanSize)

	dijkstraOutChan := make(chan queryRet, landmarkChanSize)
	dijkstraRevOutChan := make(chan queryRet, landmarkChanSize)

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

	lm.landmarks.Store(landmarks)
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
FindTighestLowerBound compute h(v)=max_{l\inL}{dist(l,t)-dist(l,v), dist(v,l)-dist(t,l)}
fungsi potential/heuristik  h(v) meiliki sifat konsisten/feasible [3]

activeLandmarks berisi list index dari active query landmark (list index dari lm.landmarks)
*/
func (lm *Landmark) FindTighestLowerBound(u, t da.Index, activeLandmarks []da.Index) float64 {
	// O(k), k = number of landmarks
	tighestLowerBound := -pkg.INF_WEIGHT
	vlw := lm.vlw.Load().(*landmarkTable)
	lw := lm.lw.Load().(*landmarkTable)
	for i := 0; i < len(activeLandmarks); i++ {
		landmarkId := activeLandmarks[i]

		lbOne := vlw.at(u, landmarkId) - vlw.at(t, landmarkId)
		lbTwo := lw.at(landmarkId, t) - lw.at(landmarkId, u)

		betterLb := 0.0
		if util.Gt(lbOne, lbTwo) {
			betterLb = lbOne
		} else {
			betterLb = lbTwo
		}

		if util.Gt(betterLb, tighestLowerBound) {
			tighestLowerBound = betterLb
		}
	}

	return tighestLowerBound
}

type activeLandmark struct {
	i  da.Index
	lb float64
}

func newActiveLandmark(i da.Index, lb float64) activeLandmark {
	return activeLandmark{i, lb}
}

const (
	activeLandmarkSize = 2
)

/*
https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf

Use only an active subset:  (page 6)
– prefer landmarks that give the best lower bound on dist(s, t).
*/
func (lm *Landmark) SelectBestQueryLandmarks(s, t da.Index) []da.Index {
	landmarks := lm.landmarks.Load().([]da.Index)
	bestLandmarks := make([]da.Index, 0, len(landmarks))

	oriLandmarks := make([]da.Index, len(landmarks))
	for i := 0; i < len(landmarks); i++ {
		oriLandmarks[i] = da.Index(i)
	}

	lowerBounds := make([]activeLandmark, len(landmarks))
	for i := 0; i < len(landmarks); i++ {
		lb, _ := lm.FindTighestConsistentLowerBound(s, s, t, oriLandmarks)
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

https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf

implementation of consistent/feasible potential function in 5.2 Consistent Approach ref [1]
calc \pi_f(u)=\frac{h_f(u)-h_r(u)}{2} and \pi_r(u)=\frac{h_r(u)-h_f(u)}{2}
h_f(u) adalah estimate sp cost dari u ke t
h_r(u) adalah estimate sp cost dari s ke u
*/
func (lm *Landmark) FindTighestConsistentLowerBound(u, s, t da.Index, activeLandmarks []da.Index) (float64, float64) {
	pifu := lm.FindTighestLowerBound(u, t, activeLandmarks) // estimate on dist(u,t)
	piru := lm.FindTighestLowerBound(s, u, activeLandmarks) // estimate on dist(s,u)

	pfu := ((pifu - piru) / 2.0)
	pru := ((piru - pifu) / 2.0)

	return pfu, pru
}

func (lm *Landmark) GetLandmarkVId(i da.Index) da.Index {
	return lm.landmarks.Load().([]da.Index)[i]
}

func (lm *Landmark) GetLandmarkVIds() []da.Index {
	return lm.landmarks.Load().([]da.Index)
}

func (lm *Landmark) GetLandmarkVWeights() [][]float64 {
	return lm.lw.Load().(*landmarkTable).matrix()
}

func (lm *Landmark) GetVerticesLandmarkWeights() [][]float64 {
	return lm.vlw.Load().(*landmarkTable).matrix()
}

func (lm *Landmark) UpdateLandmarks(landmarkFilePath string, readBuf *bufio.Reader) error {
	newLandmark, err := ReadLandmark(landmarkFilePath, readBuf)
	if err != nil {
		return fmt.Errorf("UpdateLandmarks: failed to read new precalculated landmark distances: %v: %w", err, err)
	}

	lm.landmarks.Store(newLandmark.landmarks.Load())
	lm.lw.Store(newLandmark.lw.Load())
	lm.vlw.Store(newLandmark.vlw.Load())
	return nil
}
