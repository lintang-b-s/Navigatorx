package landmark

import (
	"bufio"
	"errors"
	"fmt"
	"math"
	"os"
	"sort"
	"strconv"
	"sync"

	"github.com/dsnet/compress/bzip2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
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
[1] Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.

this is an implementation of planar landmark selection described in section 7 paper [1]
*/
func (lm *Landmark) SelectLandmarks(k int, cst *customizer.Customizer) []*da.Vertex {
	thetaDif := 360.0 / float64(k)

	landmarks := make([]*da.Vertex, k)
	ivs := cst.GetGraph().GetVertices()

	minLon := math.MaxFloat64
	maxLon := -999999.0
	minLat := math.MaxFloat64
	maxLat := -999999.0
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
	n := cst.GetGraph().NumberOfVertices()

	vs := cst.GetGraph().GetVertices()
	vsCopy := make([]*da.Vertex, n)
	copy(vsCopy, vs)

	theta := 0.0
	for i := 0; i < k; i++ {
		// O(k * VlogV)

		thetaRad := util.DegreeToRadians(theta)
		sint := math.Sin(thetaRad)
		cost := math.Cos(thetaRad)
		sort.Slice(vsCopy, func(i, j int) bool { // O(V*logV)
			a := vsCopy[i].GetLon()*cost + vsCopy[i].GetLat()*sint
			b := vsCopy[j].GetLon()*cost + vsCopy[j].GetLat()*sint
			return a < b
		})
		var (
			cand *da.Vertex
		)

		cmaxLon := vsCopy[n-1].GetLon()
		cmaxLat := vsCopy[n-1].GetLat()

		if sint == 0 {
			minxDist := math.MaxFloat64
			for j := n / 2; j < n; j++ {
				// O(V)
				v := vsCopy[j]
				dist := geo.CalculateHaversineDistance(v.GetLat(), v.GetLon(),
					centerLat, cmaxLon)
				if dist < minxDist {
					minxDist = dist
					cand = v
				}
			}
		} else if cost == 0 {
			minyDist := math.MaxFloat64
			for j := n / 2; j < n; j++ {
				v := vsCopy[j]
				dist := geo.CalculateHaversineDistance(v.GetLat(), v.GetLon(),
					cmaxLat, centerLon)
				if dist < minyDist {
					minyDist = dist
					cand = v
				}
			}
		} else {
			cand = vsCopy[n-1]
		}
		landmarks[i] = cand

		theta += thetaDif
	}

	midLandmark := &da.Vertex{}
	minMidDist := math.MaxFloat64
	for _, v := range ivs {
		// O(V)
		dist := geo.CalculateHaversineDistance(v.GetLat(), v.GetLon(),
			centerLat, centerLon)
		if dist < minMidDist {
			minMidDist = dist
			midLandmark = v
		}
	}

	landmarks = append(landmarks, midLandmark)

	return landmarks
}

/*
[1] Goldberg, A.V. and Harrelson,  (2005) ‘Computing the shortest path: A search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.

preprocessing phase of A*, landmark, and triangle inequality (ALT) described in [1]

time complexity of ALT preprocessing:

O((n+m)logn * k), n=number of vertices,m=number of edges,k=number of landmarks
*/
func (lm *Landmark) PreprocessALT(k int, m *metrics.Metric, cst *customizer.Customizer, logger *zap.Logger) error {
	if k > 64 {
		return errors.New("too much landmarks!, the maximum number of landmarks is 64. ")
	}
	logger.Info("computing landmarks....")
	lm.lw = make([][]float64, k)
	lm.landmarks = make([]da.Index, k)
	n := cst.GetGraph().NumberOfVertices()

	lm.vlw = make([][]float64, n)
	for i := 0; i < n; i++ {
		lm.vlw[i] = make([]float64, k)
	}

	for i := 0; i < k; i++ {
		lm.lw[i] = make([]float64, n)
	}
	landmarks := lm.SelectLandmarks(k, cst)

	lock := sync.Mutex{}

	wg := sync.WaitGroup{}
	for i := 0; i < k; i++ {
		landmark := landmarks[i]
		sid := landmark.GetID()
		lm.landmarks[i] = sid
		as := cst.GetGraph().GetExitOffset(sid) + cst.GetGraph().GetOutDegree(sid) - 1

		wg.Add(1)
		go func(il int, asl da.Index) {
			defer wg.Done()

			crpQuery := NewDijkstra(cst.GetGraph(), m, false) // O((n+m)logn)
			sps := crpQuery.ShortestPath(asl)

			lock.Lock()
			copy(lm.lw[il], sps)
			lock.Unlock()
		}(i, as)

		go func(il int, sidl da.Index) {
			crpQuery := NewDijkstra(cst.GetGraph(), m, true) // O((n+m)logn)
			at := cst.GetGraph().GetEntryOffset(sidl) + cst.GetGraph().GetInDegree(sidl) - 1

			sps := crpQuery.ShortestPath(at)
			lock.Lock()
			for v := 0; v < n; v++ {
				lm.vlw[v][il] = sps[v]
			}
			lock.Unlock()
		}(i, sid)
	}

	wg.Wait()
	logger.Info("done computing landmarks....")
	return nil
}

/*
[1] Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
[2] Bast, H. et al. (2016) “Route Planning in Transportation Networks,” in L.
Kliemann and P. Sanders (eds.) Algorithm Engineering: Selected Results and
Surveys. Cham: Springer International Publishing, pp. 19–80. Available at:
https://doi.org/10.1007/978-3-319-49487-6_2.
[3] Goldberg, A. and Harrelson, C. (2004) “Computing the Shortest Path: A* Search Meets Graph Theory.” Available at: https://www.microsoft.com/en-us/research/publication/computing-the-shortest-path-a-search-meets-graph-theory/ (Accessed: February 9, 2026).

implementation of computing tighest lower bound of A*, landmarks, and triangle inequality, 6 Computing Lower Bounds in [1] or section 2.2 ALT in [2]
*/
func (lm *Landmark) FindTighestLowerBound(u, t da.Index) float64 {
	// O(k), k = number of landmarks
	tighestLowerBound := -math.MaxFloat64
	for i := 0; i < len(lm.landmarks); i++ {
		if lm.vlw[u][i] >= pkg.INF_WEIGHT || lm.lw[i][t] >= pkg.INF_WEIGHT ||
			lm.vlw[t][i] >= pkg.INF_WEIGHT || lm.lw[i][u] >= pkg.INF_WEIGHT {
			continue
		}
		lbOne := lm.vlw[u][i] - lm.vlw[t][i]
		lbTwo := lm.lw[i][t] - lm.lw[i][u]

		betterLb := math.Max(lbOne, lbTwo)
		tighestLowerBound = math.Max(tighestLowerBound, betterLb)

	}

	// lemma 2.1 from paper [3]:
	// Suppose \pi is feasible and for vertex t \in V we have \pi(t) <= 0. Then for any v \in V,
	// \pi(v) <= dist(v,t) and \pi(v) is nonnegative.
	// using triangle inequalities computed in the for loop above, we know that
	// tighestLowerBound = \pi(u) <= dist(u,t) holds
	// thus, after clamping tighestLowerBound, tighestLowerBound is feasible/admissible potential function for A* algorithm
	tighestLowerBound = math.Max(tighestLowerBound, 0)

	return tighestLowerBound
}

/*
[1] Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
[2] https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf
[3] Ikeda, T. et al. (1994) ‘A fast algorithm for finding better routes by AI search techniques’, in Proceedings of VNIS’94 - 1994 Vehicle Navigation and Information Systems Conference, pp. 291–296. Available at: https://doi.org/10.1109/VNIS.1994.396824.

implementation of consistent potential function in 5.2 Consistent Approach ref [1]
*/
func (lm *Landmark) FindTighestConsistentLowerBound(u, s, t da.Index) (float64, float64) {
	pifu := lm.FindTighestLowerBound(u, t)
	piru := lm.FindTighestLowerBound(u, s)
	pfu := (pifu - piru) / 2.0
	pru := -pfu

	return pfu, pru
}

func (lm *Landmark) WriteLandmark(filename string, cst *customizer.Customizer) error {
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()

	bz, err := bzip2.NewWriter(f, &bzip2.WriterConfig{})
	if err != nil {
		return err
	}
	defer bz.Close()

	w := bufio.NewWriter(bz)

	k := len(lm.landmarks)

	n := cst.GetGraph().NumberOfVertices()
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

	return w.Flush()
}

func ReadLandmark(filename string) (*Landmark, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, err
	}

	defer f.Close()

	bz, err := bzip2.NewReader(f, nil)

	if err != nil {
		return nil, err
	}
	br := bufio.NewReader(bz)

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

	lm := NewLandmark()
	lm.lw = lw
	lm.vlw=vlw
	lm.landmarks = landmarks
	return lm, nil
}
