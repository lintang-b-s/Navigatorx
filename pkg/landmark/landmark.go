package landmark

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"sort"
	"strconv"
	"sync"

	"github.com/cockroachdb/errors"

	"github.com/klauspost/compress/s2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Landmark struct {
	lw        [][]float64 // distance from each landmarks to every vertices in graph
	vlw       [][]float64 // distance from all vertices to each landmarks
	landmarks []da.Index  // landmark vertex ids

}

func NewLandmark() *Landmark {
	return &Landmark{
		lw:        make([][]float64, 0),
		landmarks: make([]da.Index, 0),
	}
}

/*
[1] Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.

this is an implementation of planar landmark selection described in section 7 paper [1] versi 2

O(V*logV)
*/
func (lm *Landmark) SelectLandmarksTwo(k int, graph *datastructure.Graph) []*da.Vertex {

	landmarks := make([]*da.Vertex, 0, k)
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

	midLandmark := &da.Vertex{}
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

	vs := graph.GetVertices()
	vsCopy := make([]*da.Vertex, n)
	copy(vsCopy, vs)

	landmarks = append(landmarks, midLandmark)

	// mirip algoritma graham scan buat bikin convex hull
	// graham scan: sort Points by their polar angles around a p0 (bottomost point or rightmost & bottomost point if tie)
	// ini: sort Points by their initial bearing angles around a p0 (center point/coordinate)
	// karena geographic coordinate, sort by initial bearing angle (sudut clockwise antara garis yang menghubungkan titik pivot ke other point dan garis meridian)

	sort.Slice(vsCopy, func(i, j int) bool { // O(V * logV)
		return geo.BearingTo(midLandmark.GetLat(), midLandmark.GetLon(), vsCopy[j].GetLat(), vsCopy[j].GetLon()) <
			geo.BearingTo(midLandmark.GetLat(), midLandmark.GetLon(), vsCopy[i].GetLat(), vsCopy[i].GetLon())
	})

	var (
		pieSize int
	)

	pieSize = n / k

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
func (lm *Landmark) PreprocessALT(k int, m *metrics.Metric, graph *datastructure.Graph, logger *zap.Logger) error {
	if k > 64 {
		return errors.New("too much landmarks!, the maximum number of landmarks is 64. ")
	}
	n := graph.NumberOfVertices()

	if n < k {
		lm.lw = make([][]float64, 0)
		lm.landmarks = make([]da.Index, 0)
		return nil
	}

	logger.Info("computing landmarks....")
	lm.lw = make([][]float64, k)
	lm.landmarks = make([]da.Index, k)

	lm.vlw = make([][]float64, n)
	for i := 0; i < n; i++ {
		lm.vlw[i] = make([]float64, k)
	}

	for i := 0; i < k; i++ {
		lm.lw[i] = make([]float64, n)
	}
	landmarks := lm.SelectLandmarksTwo(k, graph)

	maxSearchSize := graph.NumberOfEdges()
	maxEdgesInCell := graph.GetMaxEdgesInCell()

	heapPool := sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](uint32(maxSearchSize), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	calcDijkstra := func(qp queryParam) queryRet {
		sid := qp.getSid()
		il := qp.getIndex()

		asl := graph.GetDummyOutEdgeId(sid)

		crpQuery := NewDijkstra(graph, m, false) // O(mlogm). at most m items in pq (edge-based), decrease/insert key operation at most m times, extractMin operation at most m times
		sps := crpQuery.ShortestPath(asl, &heapPool)

		return newQueryRet(il, sps)
	}

	calcDijkstraRev := func(qp queryParam) queryRet {
		sid := qp.getSid()
		il := qp.getIndex()
		crpQuery := NewDijkstra(graph, m, true) // O(mlogm)
		at := graph.GetDummyInEdgeId(sid)       // dummy edge (s,s)

		sps := crpQuery.ShortestPath(at, &heapPool)
		return newQueryRet(il, sps)
	}

	wpdijkstra := concurrent.NewWorkerPool[queryParam, queryRet](WORKERS, k)
	wpdijkstraRev := concurrent.NewWorkerPool[queryParam, queryRet](WORKERS, k)
	for i := 0; i < k; i++ {
		landmark := landmarks[i]
		sid := landmark.GetID()
		lm.landmarks[i] = sid

		wpdijkstra.AddJob(newQueryparam(i, sid))
		wpdijkstraRev.AddJob(newQueryparam(i, sid))

	}

	wpdijkstra.Close()
	wpdijkstra.Start(calcDijkstra)
	wpdijkstra.Wait()

	wpdijkstraRev.Close()
	wpdijkstraRev.Start(calcDijkstraRev)
	wpdijkstraRev.Wait()

	wg := sync.WaitGroup{}
	wg.Add(1)
	go func() {
		defer wg.Done()
		for res := range wpdijkstra.CollectResults() {
			il := res.getIndex()
			sps := res.getSpCosts()
			copy(lm.lw[il], sps)
		}
	}()

	for res := range wpdijkstraRev.CollectResults() {
		il := res.getIndex()
		sps := res.getSpCosts()
		for v := 0; v < n; v++ {
			lm.vlw[v][il] = sps[v]
		}
	}

	wg.Wait()

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
	for i := 0; i < len(activeLandmarks); i++ {
		landmarkId := activeLandmarks[i]

		lbOne := lm.vlw[u][landmarkId] - lm.vlw[t][landmarkId]
		lbTwo := lm.lw[landmarkId][t] - lm.lw[landmarkId][u]

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
	bestLandmarks := make([]da.Index, 0, len(lm.landmarks))

	oriLandmarks := make([]da.Index, len(lm.landmarks))
	for i := 0; i < len(lm.landmarks); i++ {
		oriLandmarks[i] = da.Index(i)
	}

	lowerBounds := make([]activeLandmark, len(lm.landmarks))
	for i := 0; i < len(lm.landmarks); i++ {
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

implementation of consistent/feasible potential function in 5.2 Consistent Approach ref [1]
calc \pi_f(u)=\frac{h_f(u)-h_r(u)}{2} and \pi_r(u)=\frac{h_r(u)-h_f(u)}{2}
h_f(u) adalah estimate sp cost dari u ke t
h_r(u) adalah estimate sp cost dari s ke u
*/
func (lm *Landmark) FindTighestConsistentLowerBound(u, s, t da.Index, activeLandmarks []da.Index) (float64, float64) {
	pifu := lm.FindTighestLowerBound(u, t, activeLandmarks) // estimate on dist(u,t)
	piru := lm.FindTighestLowerBound(s, u, activeLandmarks) // estimate on dist(s,u)
	pfu := (pifu - piru) / 2.0
	pru := -pfu

	return pfu, pru
}

func (lm *Landmark) GetLandmarkVId(i da.Index) da.Index {
	return lm.landmarks[i]
}

func (lm *Landmark) GetLandmarkVIds() []da.Index {
	return lm.landmarks
}

func (lm *Landmark) GetLandmarkVWeights() [][]float64 {
	return lm.lw
}

func (lm *Landmark) GetVerticesLandmarkWeights() [][]float64 {
	return lm.vlw
}

func (lm *Landmark) WriteLandmark(filename string, graph *datastructure.Graph) error {
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()

	snp := s2.NewWriter(f)
	if err != nil {
		return err
	}

	defer snp.Close()

	w := bufio.NewWriter(snp)

	k := len(lm.landmarks)

	n := graph.NumberOfVertices()
	fmt.Fprintf(w, "%d %d\n", k, n)

	for i := 0; i < k; i++ {
		landmarkvId := lm.landmarks[i]
		fmt.Fprintf(w, "%d ", landmarkvId)

		for v := 0; v < n; v++ {
			sp := strconv.FormatFloat(lm.lw[i][v], 'f', -1, 64)
			fmt.Fprintf(w, "%s", sp)
			if v < n-1 {
				fmt.Fprintf(w, " ")
			}
		}

		fmt.Fprintf(w, "\n")

		for v := 0; v < n; v++ {
			sp := strconv.FormatFloat(lm.vlw[v][i], 'f', -1, 64)
			fmt.Fprintf(w, "%s", sp)
			if v < n-1 {
				fmt.Fprintf(w, " ")
			}
		}
		fmt.Fprintf(w, "\n")
	}

	if err = w.Flush(); err != nil {
		return errors.Wrapf(err, "WriteLandmark: failed to flush bufio writer")
	}

	return nil
}

const (
	landmarkBufferSize = 4096 * 2
)

func ReadLandmark(filename string) (*Landmark, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, err
	}

	snp := s2.NewReader(f)

	if err != nil {
		return nil, err
	}
	br := bufio.NewReaderSize(snp, landmarkBufferSize)

	line, err := util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	ff := util.Fields(line)
	k, err := strconv.Atoi(ff[0])
	if err != nil {
		return nil, err
	}
	n, err := da.ParseIndex(ff[1])
	if err != nil {
		return nil, err
	}

	landmarks := make([]da.Index, k)
	lw := make([][]float64, k)
	vlw := make([][]float64, n)

	for v := 0; v < int(n); v++ {
		vlw[v] = make([]float64, k)
	}

	for i := 0; i < k; i++ {
		line, err := util.ReadLine(br)
		if err != nil {
			return nil, err
		}
		ff := util.Fields(line)

		landmarkvId, err := da.ParseIndex(ff[0])
		if err != nil {
			return nil, err
		}

		landmarks[i] = da.Index(landmarkvId)
		lw[i] = make([]float64, n)

		for j := 1; j < len(ff); j++ {
			sp, err := strconv.ParseFloat(ff[j], 64)
			if err != nil {
				return nil, err
			}

			lw[i][j-1] = sp
		}

		line, err = util.ReadLine(br)
		if err != nil {
			return nil, err
		}
		ff = util.Fields(line)
		for v := 0; v < len(ff); v++ {
			sp, err := strconv.ParseFloat(ff[v], 64)
			if err != nil {
				return nil, err
			}

			vlw[v][i] = sp
		}
	}

	if err = f.Close(); err != nil {
		return nil, errors.Wrapf(err, "ReadLandmark: failed to close file: %s", filename)
	}

	lm := NewLandmark()
	lm.lw = lw
	lm.vlw = vlw
	lm.landmarks = landmarks

	return lm, nil
}
