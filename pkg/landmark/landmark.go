// Package landmark provides landmark selection and ALT (A*, Landmarks, and Triangle inequality) preprocessing & query phase.
package landmark

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"sync"
	"sync/atomic"

	"github.com/bytedance/gopkg/util/gopool"

	"github.com/klauspost/compress/s2"
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

func NewLandmark() *Landmark {
	lm := &Landmark{}
	lw := make([][]float64, 0)
	lm.lw.Store(lw)
	vlw := make([][]float64, 0)
	lm.vlw.Store(vlw)
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
	n := graph.NumberOfVertices()

	if n < k {
		lm.lw.Store(make([][]float64, 0))
		lm.landmarks.Store(make([]da.Index, 0))
		return nil
	}

	logger.Info("computing landmarks....")
	lw := make([][]float64, k)
	landmarks := make([]da.Index, k)

	vlw := make([][]float64, n)
	for i := 0; i < n; i++ {
		vlw[i] = make([]float64, k)
	}

	for i := 0; i < k; i++ {
		lw[i] = make([]float64, n)
	}
	landmarksVertices := lm.SelectLandmarksTwo(k, graph)

	maxSearchSize := graph.NumberOfEdges()
	maxEdgesInCell := graph.GetMaxEdgesInCell()

	heapPool := sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](uint32(maxSearchSize), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	dijkstraInChan := make(chan queryParam, 6)
	dijkstraRevInChan := make(chan queryParam, 6)

	dijkstraOutChan := make(chan queryRet, 6)
	dijkstraRevOutChan := make(chan queryRet, 6)

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
			for v := 0; v < n; v++ {
				vlw[v][il] = sps[v]
			}
			wg.Done()
		}
	}()

	go func() {
		for res := range dijkstraOutChan {
			il := res.getIndex()
			sps := res.getSpCosts()
			copy(lw[il], sps)
			wg.Done()
		}
	}()

	for i := 0; i < int(WORKERS); i++ {
		gopool.CtxGo(context.Background(), calcDijkstra)
		gopool.CtxGo(context.Background(), calcDijkstraRev)
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
	vlw := lm.vlw.Load().([][]float64)
	lw := lm.lw.Load().([][]float64)
	for i := 0; i < len(activeLandmarks); i++ {
		landmarkId := activeLandmarks[i]

		lbOne := vlw[u][landmarkId] - vlw[t][landmarkId]
		lbTwo := lw[landmarkId][t] - lw[landmarkId][u]

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
	return lm.lw.Load().([][]float64)
}

func (lm *Landmark) GetVerticesLandmarkWeights() [][]float64 {
	return lm.vlw.Load().([][]float64)
}

func (lm *Landmark) WriteLandmark(filename string, n int) error {
	dir := filepath.Dir(filename)
	if _, err := os.Stat(dir); os.IsNotExist(err) {
		if err := os.MkdirAll(dir, 0755); err != nil {
			return err
		}
	}

	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()

	snp := s2.NewWriter(f)

	defer snp.Close()

	w := bufio.NewWriter(snp)

	landmarks := lm.landmarks.Load().([]da.Index)
	k := len(landmarks)
	lw := lm.lw.Load().([][]float64)
	vlw := lm.vlw.Load().([][]float64)

	fmt.Fprintf(w, "%d %d\n", k, n)

	for i := 0; i < k; i++ {
		landmarkvId := landmarks[i]
		fmt.Fprintf(w, "%d ", landmarkvId)

		for v := 0; v < n; v++ {
			sp := strconv.FormatFloat(lw[i][v], 'f', -1, 64)
			fmt.Fprintf(w, "%s", sp)
			if v < n-1 {
				fmt.Fprintf(w, " ")
			}
		}

		fmt.Fprintf(w, "\n")

		for v := 0; v < n; v++ {
			sp := strconv.FormatFloat(vlw[v][i], 'f', -1, 64)
			fmt.Fprintf(w, "%s", sp)
			if v < n-1 {
				fmt.Fprintf(w, " ")
			}
		}
		fmt.Fprintf(w, "\n")
	}

	if err = w.Flush(); err != nil {
		return fmt.Errorf("WriteLandmark: failed to flush bufio writer: %w", err)
	}

	// http://stackoverflow.com/questions/10862375/when-to-flush-a-file-in-go
	// idk tapi tests/driving_direction  di github actions selalu fails baru baru ini
	// mungkin gara gara gak di f.Sync() dulu?
	// padahal di laptop " cd tests/driving_direction &&  go test -v ." selalu pass
	// aneh cok
	if err = f.Sync(); err != nil {
		f.Close()
		return fmt.Errorf("metric.WriteToFile: failed to sync temp file: %w", err)
	}

	return nil
}

func ReadLandmark(filename string, readBuf *bufio.Reader) (*Landmark, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, fmt.Errorf("ReadLandmark: failed to open file: %s: %w", filename, err)
	}

	snp := s2.NewReader(f)

	if err != nil {
		return nil, fmt.Errorf("ReadLandmark: failed to create new s2 reader for file: %s: %w", filename, err)
	}
	readBuf.Reset(snp)

	line, err := util.ReadLine(readBuf)
	if err != nil {
		return nil, fmt.Errorf("ReadLandmark: failed to read header line from file %s: %w", filename, err)
	}
	ff := util.Fields(line)
	k, err := strconv.Atoi(ff[0])
	if err != nil {
		return nil, fmt.Errorf("ReadLandmark: failed to parse landmark count from header %q in file %s: %w", line, filename, err)
	}
	n, err := da.ParseIndex(ff[1])
	if err != nil {
		return nil, fmt.Errorf("ReadLandmark: failed to parse vertex count from header %q in file %s: %w", line, filename, err)
	}

	landmarks := make([]da.Index, k)
	lw := make([][]float64, k)
	vlw := make([][]float64, n)

	for v := 0; v < int(n); v++ {
		vlw[v] = make([]float64, k)
	}

	for i := 0; i < k; i++ {
		line, err := util.ReadLine(readBuf)
		if err != nil {
			return nil, fmt.Errorf("ReadLandmark: failed to read landmark row %d from file %s: %w", i, filename, err)
		}
		ff := util.Fields(line)

		landmarkvId, err := da.ParseIndex(ff[0])
		if err != nil {
			return nil, fmt.Errorf("ReadLandmark: failed to parse landmark vertex id at row %d in file %s: %w", i, filename, err)
		}

		landmarks[i] = da.Index(landmarkvId)
		lw[i] = make([]float64, n)

		for j := 1; j < len(ff); j++ {
			sp, err := strconv.ParseFloat(ff[j], 64)
			if err != nil {
				return nil, fmt.Errorf("ReadLandmark: failed to parse lw[%d][%d] at row %d in file %s: %w", i, j-1, i, filename, err)
			}

			lw[i][j-1] = sp
		}

		line, err = util.ReadLine(readBuf)
		if err != nil {
			return nil, fmt.Errorf("ReadLandmark: failed to read reverse landmark row %d from file %s: %w", i, filename, err)
		}
		ff = util.Fields(line)
		for v := 0; v < len(ff); v++ {
			sp, err := strconv.ParseFloat(ff[v], 64)
			if err != nil {
				return nil, fmt.Errorf("ReadLandmark: failed to parse vlw[%d][%d] at reverse row %d in file %s: %w", v, i, i, filename, err)
			}

			vlw[v][i] = sp
		}
	}

	if err = f.Close(); err != nil {
		return nil, fmt.Errorf("ReadLandmark: failed to close file: %s: %w", filename, err)
	}

	lm := NewLandmark()
	lm.lw.Store(lw)
	lm.vlw.Store(vlw)
	lm.landmarks.Store(landmarks)

	return lm, nil
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
