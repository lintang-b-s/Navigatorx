package partitioner

import (
	"math"
	"math/rand"
	"sort"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
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
	graph      *datastructure.PartitionGraph
	iterations int
	directed   bool
}

func NewInertialFlow(graph *datastructure.PartitionGraph, iterations int, directed bool) *inertialFlow {
	return &inertialFlow{graph: graph, iterations: iterations, directed: directed}
}

func (inf *inertialFlow) getPartitionGraph() *datastructure.PartitionGraph {
	return inf.graph
}

/*
computeInertialFlowDinic.
[On Balanced Separators in Road Networks, Schild, et al.] https://aschild.github.io/papers/roadseparator.pdf

return st-mincut dengan partisi S, T yang saling disjoint.
time complexity:
karena cuma call algoritma dinic berkali kali sejumlah iterations, let k = number of iterations+2
O(k*n^2*m).
*/
func (inf *inertialFlow) computeInertialFlowDinic(sourceSinkRate float64) *MinCut {
	var (
		best                    *MinCut = &MinCut{}
		bestNumberOfMinCutEdges         = math.MaxInt
	)

	n := inf.graph.NumberOfVertices()
	iterations := inf.iterations
	if n >= LARGE_GRAPH_NUMBER_OF_VERTICES {
		iterations = INERTIAL_FLOW_ITERATION_LARGE_GRAPH
	}

	wpInertialFlow := concurrent.NewWorkerPool[minCutJob, *MinCut](INERTIAL_FLOW_WORKERS, iterations+2)

	balanceDelta := func(numPartTwoNodes int, numberOfVertices int) int {
		diff := numberOfVertices/2 - numPartTwoNodes
		if diff < 0 {
			diff = -diff
		}
		return diff
	}

	for i := 0; i < iterations; i++ {
		slope := -1 + float64(i)*float64(2/iterations)
		wpInertialFlow.AddJob(newMinCutJob([]float64{slope, (1 - math.Abs(slope))})) //  (-1,0), ....., (0, 1)
	}

	wpInertialFlow.AddJob(newMinCutJob([]float64{1, 1}))
	wpInertialFlow.AddJob(newMinCutJob([]float64{-1, 1}))

	computeMinCut := func(input minCutJob) *MinCut {
		dn := NewDinicMaxFlow(inf.getPartitionGraph(), false, true)
		var (
			sources []datastructure.Index
			sinks   []datastructure.Index
		)

		sources, sinks = dn.selectFirstLastKthVertices(input.getLine(), sourceSinkRate)

		s, t := dn.createArtificialSourceSink(sources, sinks, inf.directed)
		return dn.ComputeMaxflowMinCut(s, t) //  O(n^2 * m), n,m=number of vertices & edges dari da.PartitionGraph
	}

	wpInertialFlow.Close()
	wpInertialFlow.Start(computeMinCut)
	wpInertialFlow.Wait()

	numberOfVertices := inf.graph.NumberOfVertices()
	for minCut := range wpInertialFlow.CollectResults() {

		if minCut.GetNumOfCutEdges() < bestNumberOfMinCutEdges ||
			(bestNumberOfMinCutEdges == minCut.GetNumOfCutEdges() &&
				balanceDelta(minCut.GetNumNodesInPartitionTwo(),
					numberOfVertices) < balanceDelta(best.GetNumNodesInPartitionTwo(),
					numberOfVertices)) {
			best = minCut
			bestNumberOfMinCutEdges = minCut.GetNumOfCutEdges()
		}
	}

	return best
}

type vertexEmb struct {
	idx        int
	dotProduct float64
}

func newVertexEmb(idx int, dotProduct float64) vertexEmb {
	return vertexEmb{idx, dotProduct}
}

func (v vertexEmb) getDotProd() float64 {
	return v.dotProduct
}

// selectFirstLastKthVertices. sort vertices by linear kombinaasi latitude & longitude 
func (dn *DinicMaxFlow) selectFirstLastKthVertices(l []float64, ratio float64) ([]datastructure.Index, []datastructure.Index) {

	vertices := dn.graph.GetVertices()

	n := len(vertices)

	vertEmbeds := make([]vertexEmb, n)
	for i, v := range vertices {
		lat, lon := v.GetVertexCoordinate()
		proj := dot(lon, lat, l[0], l[1])
		vertEmbeds[i] = newVertexEmb(i, proj)
	}
	kth := int(float64(n) * ratio)

	if kth == 0 {
		kth = 1
	}

	sourceNodes := make([]datastructure.Index, 0, kth)
	sinkNodes := make([]datastructure.Index, 0, kth)

	if USE_RANDOMIZED_SELECT {
		// expected runtime O(n)

		// inspiration: https://daniel-j-h.github.io/post/selection-algorithms-for-partitioning/
		q := dn.randomizedSelect(vertEmbeds, 0, n-1, kth, func(left, right int) bool {
			return vertEmbeds[left].getDotProd() <= vertEmbeds[right].getDotProd()
		}) // q is the index of the kth-smallest element in the vertEmbeds

		for i := 0; i < kth; i++ {
			sourceNodes = append(sourceNodes, vertices[vertEmbeds[i].idx].GetID())
		}

		// sampai sini kita mendapatkan semua elements didalam arr[q+1, n-1] lebih dari arr[q]
		// kita bisa randomizedSelect arr[q+1, n-1] untuk mendapatkan last k sinks
		lastKth := util.MinInt(kth, n-kth)
		dn.randomizedSelect(vertEmbeds, q+1, n-1, lastKth, func(left, right int) bool {
			return vertEmbeds[left].getDotProd() > vertEmbeds[right].getDotProd()
		})

		for i := q + 1; i < q+1+lastKth; i++ {
			sinkNodes = append(sinkNodes, vertices[vertEmbeds[i].idx].GetID())
		}
	} else {
		// expected runtime O(nlogn) kalau sort.Slice randomized quicksort
		sort.Slice(vertEmbeds, func(i, j int) bool {
			return vertEmbeds[i].getDotProd() < vertEmbeds[j].getDotProd()
		})

		for i := 0; i < kth; i++ {
			sourceNodes = append(sourceNodes, vertices[vertEmbeds[i].idx].GetID())
			sinkNodes = append(sinkNodes, vertices[vertEmbeds[n-1-i].idx].GetID())
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
func (dn *DinicMaxFlow) randomizedSelect(arr []vertexEmb, p, r, i int, comp func(left, right int) bool) int {
	if p == r {
		return p
	}

	q := dn.randomizedPartition(arr, p, r, comp)
	k := q - p + 1 // size of arr[p,...,q] (include pivot element arr[q])
	if i == k {
		return q
	} else if i < k {
		return dn.randomizedSelect(arr, p, q-1, i, comp)
	}
	return dn.randomizedSelect(arr, q+1, r, i-k, comp) // i-k th smallest/largest element di arr[q+1,...,r] karena di next recursion kita operate di arr[q+1,...,r]
}

func (dn *DinicMaxFlow) randomizedPartition(arr []vertexEmb, p, r int, comp func(left, right int) bool) int {
	i := p - 1
	rd := rand.New(rand.NewSource(time.Now().UnixNano())) // gak thread-safe

	pivotId := p + rd.Intn(r-p+1)
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

func (dn *DinicMaxFlow) createArtificialSourceSink(sourceNodes, sinkNodes []datastructure.Index, directed bool) (datastructure.Index, datastructure.Index) {
	artificialSource := datastructure.Index(dn.graph.NumberOfVertices())
	artificialSink := datastructure.Index(dn.graph.NumberOfVertices() + 1)

	dn.AddArtificialVertex(datastructure.NewPartitionVertex(artificialSource, datastructure.Index(ARTIFICIAL_SOURCE_ID), 0.0, 0.0))
	dn.AddArtificialVertex(datastructure.NewPartitionVertex(artificialSink, datastructure.Index(ARTIFICIAL_SINK_ID), 0.0, 0.0))

	for _, s := range sourceNodes {
		dn.AddArtificialEdge(artificialSource, s, pkg.INF_WEIGHT_INT, directed)
	}

	for _, t := range sinkNodes {
		dn.AddSinks(t)
		dn.AddArtificialEdge(t, artificialSink, pkg.INF_WEIGHT_INT, directed)
	}
	return artificialSource, artificialSink
}
