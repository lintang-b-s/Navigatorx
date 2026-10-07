package partitioner

import (
	"math"

	"sort"
	"sync"

	"math/rand/v2"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type minCutJob struct {
	line []float64
}

func newMinCutJob(line []float64) minCutJob {
	return minCutJob{line: line}
}

func (mj minCutJob) getLine() []float64 {
	return mj.line
}

type inertialFlow struct {
	cg         *da.CellGraph
	iterations int
}

func NewInertialFlow(cg *da.CellGraph, iterations int) *inertialFlow {
	return &inertialFlow{cg: cg, iterations: iterations}
}

func (inf *inertialFlow) getCellGraph() *da.CellGraph {
	return inf.cg
}

/*
computeInertialFlowDinic.
[On Balanced Separators in Road Networks, Schild, et al.] https://aschild.github.io/papers/roadseparator.pdf

this implementation inspired by this doc: https://github.com/Telenav/open-source-spec/blob/master/routing_basic/doc/inertial_flow.md

return st-mincut dengan partisi S, T yang saling disjoint.
time complexity:
karena cuma call algoritma dinic unit capacity berkali kali sejumlah iterations, let k = number of iterations+2
ref1: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf

time complexity dinic algorithm on general capacity graph:
see lemma 4.1 ref1, O(n^2 * m), n,m=number of vertices & edges dari da.CellGraph

time complexity dinic algorithm on unit capacity graph:
see lemma 4.2 ref1, dinic unit capacity graph worst case: O(min{m * sqrt(m), m * n^(2/3)})

karena di implementasi inertial flow ini kita selalu pakai unit capacity..
O(k*min{m * sqrt(m), m * n^(2/3)}).
*/
func (inf *inertialFlow) computeInertialFlowDinic(sourceSinkRate float64) *MinCut {
	var (
		best                    = &MinCut{}
		bestNumberOfMinCutEdges = math.MaxInt
	)

	n := inf.cg.NumberOfCellVertices()
	iterations := inf.iterations
	if n >= LARGE_GRAPH_NUMBER_OF_VERTICES {
		iterations = INERTIAL_FLOW_ITERATION_LARGE_GRAPH
	}

	inertialFlowInChan := make(chan minCutJob, iterations+2)
	inertialFlowOutChan := make(chan *MinCut, iterations+2)

	balanceDelta := func(numPartTwoNodes int, numOfCellVertices int) int {
		diff := numOfCellVertices/2 - numPartTwoNodes
		if diff < 0 {
			diff = -diff
		}
		return diff
	}

	wg := sync.WaitGroup{}

	go func() {
		numOfCellVertices := inf.cg.NumberOfCellVertices()
		for minCut := range inertialFlowOutChan {
			if minCut.GetNumOfCutEdges() < bestNumberOfMinCutEdges ||
				(bestNumberOfMinCutEdges == minCut.GetNumOfCutEdges() &&
					balanceDelta(minCut.GetNumNodesInPartitionTwo(),
						numOfCellVertices) < balanceDelta(best.GetNumNodesInPartitionTwo(),
						numOfCellVertices)) {
				best = minCut
				bestNumberOfMinCutEdges = minCut.GetNumOfCutEdges()
			}
			wg.Done()
		}
	}()

	computeMinCut := func() {
		for input := range inertialFlowInChan {
			icg := inf.getCellGraph()
			n := icg.NumberOfCellVertices()
			dn := NewDinicMaxFlow[int32](n, true, true)
			inf.initNetworkCapacity(n, dn)
			sources, sinks := inf.selectFirstLastKthVertices(input.getLine(), sourceSinkRate)
			s, t := dn.createArtificialSourceSink(sources, sinks)
			inertialFlowOutChan <- dn.ComputeMaxflowMinCut(s, t) //  O(min{m * sqrt(m), m * n^(2/3)}), n,m=number of vertices & edges dari da.CellGraph
		}
	}

	for i := 0; i < INERTIAL_FLOW_WORKERS; i++ {
		go computeMinCut()
	}

	for i := 0; i < iterations; i++ {
		slope := -1 + float64(i)*2.0/float64(iterations) //  (-1,0), ....., (0, 1)
		wg.Add(1)
		inertialFlowInChan <- newMinCutJob([]float64{slope, (1 - math.Abs(slope))})
	}

	wg.Add(2)
	inertialFlowInChan <- newMinCutJob([]float64{1, 1})
	inertialFlowInChan <- newMinCutJob([]float64{-1, 1})

	close(inertialFlowInChan)

	wg.Wait()
	close(inertialFlowOutChan)

	return best
}

type vertexEmb struct {
	vId        da.Index
	dotProduct float64
}

func newVertexEmb(vId da.Index, dotProduct float64) vertexEmb {
	return vertexEmb{vId, dotProduct}
}

func (v vertexEmb) getDotProd() float64 {
	return v.dotProduct
}

func (inf *inertialFlow) initNetworkCapacity(n int, dn *DinicMaxFlow[int32]) {
	for u := da.Index(0); u < da.Index(n); u++ {
		inf.cg.ForOutEdgesOf(u, func(v da.Index) {
			dn.AddEdge(u, v, 1, true)
		})
	}
}

// selectFirstLastKthVertices. sort vertices by linear kombinaasi latitude & longitude
func (inf *inertialFlow) selectFirstLastKthVertices(l []float64, ratio float64) ([]da.Index, []da.Index) {

	n := inf.cg.NumberOfCellVertices()

	vertEmbeds := make([]vertexEmb, n)
	i := 0
	inf.cg.ForEachCellVertices(func(vId, _ da.Index, coord da.Coordinate) {
		lat, lon := coord.GetLat(), coord.GetLon()
		proj := dot(lon, lat, l[0], l[1])
		vertEmbeds[i] = newVertexEmb(vId, proj)
		i++
	})

	kth := int(float64(n) * ratio)

	if kth == 0 {
		kth = 1
	}

	sourceNodes := make([]da.Index, 0, kth)
	sinkNodes := make([]da.Index, 0, kth)

	if USE_RANDOMIZED_SELECT {
		// expected runtime O(n)

		// inspiration: https://daniel-j-h.github.io/post/selection-algorithms-for-partitioning/
		q := inf.randomizedSelect(vertEmbeds, 0, n-1, kth, func(left, right int) bool {
			return vertEmbeds[left].getDotProd() <= vertEmbeds[right].getDotProd()
		}) // q is the index of the kth-smallest element in the vertEmbeds

		for i := 0; i < kth; i++ {
			sourceNodes = append(sourceNodes, vertEmbeds[i].vId)
		}

		// sampai sini kita mendapatkan semua elements didalam arr[q+1, n-1] lebih dari arr[q]
		// kita bisa randomizedSelect arr[q+1, n-1] untuk mendapatkan last k sinks
		lastKth := min(kth, n-kth)
		inf.randomizedSelect(vertEmbeds, q+1, n-1, lastKth, func(left, right int) bool {
			return vertEmbeds[left].getDotProd() > vertEmbeds[right].getDotProd()
		})

		for i := q + 1; i < q+1+lastKth; i++ {
			sinkNodes = append(sinkNodes, vertEmbeds[i].vId)
		}
	} else {
		// expected runtime O(nlogn) kalau sort.Slice randomized quicksort
		sort.Slice(vertEmbeds, func(i, j int) bool {
			return vertEmbeds[i].getDotProd() < vertEmbeds[j].getDotProd()
		})

		for i := 0; i < kth; i++ {
			sourceNodes = append(sourceNodes, vertEmbeds[i].vId)
			sinkNodes = append(sinkNodes, vertEmbeds[n-1-i].vId)
		}
	}

	return sourceNodes, sinkNodes
}

func dot(x1, y1, x2, y2 float64) float64 {
	return x1*x2 + y1*y2
}

// randomizedSelect. return the i-th smallest element (or largest depends on comp) of the array arr[p...r]
// & partition the arr such that all elements (arr[p,..q]) left of i-th smallest element  are smaller (or largest depends on comp) than  the pivot element arr[q] & all elements (arr[q+1,...,r]) in the right of i-th smallest element
// expected runtime O(n), n=len(arr). worst case O(n^2)
// read chapter 9.2 CLRS (introduction to algorithm by Cormen, et al. 3rd edition) for the time complexity analysis
func (inf *inertialFlow) randomizedSelect(arr []vertexEmb, p, r, i int, comp func(left, right int) bool) int {
	if p == r {
		return p
	}

	q := inf.randomizedPartition(arr, p, r, comp)
	k := q - p + 1 // size of arr[p,...,q] (include pivot element arr[q])
	if i == k {
		return q
	} else if i < k {
		return inf.randomizedSelect(arr, p, q-1, i, comp)
	}
	return inf.randomizedSelect(arr, q+1, r, i-k, comp) // i-k th smallest/largest element di arr[q+1,...,r] karena di next recursion kita operate di arr[q+1,...,r]
}

func (inf *inertialFlow) randomizedPartition(arr []vertexEmb, p, r int, comp func(left, right int) bool) int {
	i := p - 1

	pivotId := p + rand.IntN(r-p+1)
	arr[pivotId], arr[r] = arr[r], arr[pivotId]
	for j := p; j < r; j++ {
		if comp(j, r) {
			i++
			arr[i], arr[j] = arr[j], arr[i]
		}
	}

	arr[i+1], arr[r] = arr[r], arr[i+1]

	return i + 1
}

func (dmf *DinicMaxFlow[int32]) createArtificialSourceSink(sourceNodes, sinkNodes []da.Index) (da.Index, da.Index) {
	ars := da.Index(dmf.n)     // artificial source vertex id
	art := da.Index(dmf.n + 1) // artificial sink vertex id

	dmf.AddArtificialVertex(ars)
	dmf.AddArtificialVertex(art)

	infcap := int32(math.MaxInt32)
	for _, s := range sourceNodes {
		dmf.AddEdge(ars, s, infcap, true)
	}

	for _, t := range sinkNodes {
		dmf.AddEdge(t, art, infcap, true)
	}
	return ars, art
}
