// Package metrics provides utilities for managing and calculating graph metrics (edge & turn costs) and stalling tables.
package metrics

import (
	"bufio"
	"fmt"
	"sync"
	"sync/atomic"

	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Metric struct {
	// https://go101.org/article/concurrent-atomic-operation.html   https://pkg.go.dev/sync/atomic#Pointer.Load   https://go.dev/ref/mem#atomic
	weights                                     atomic.Pointer[da.OverlayWeights]
	entryStallingTables                         atomic.Value // stallingTables for vertex-v, i-th incoming edge and j-th incoming edge:  stallingTables[v][i*inDegree[v]+j]
	exitStallingTables                          atomic.Value
	costFunction                                atomic.Pointer[costfunction.TimeFunction]
	lastSegmentSpeedFiles, lastTurnPenaltyFiles atomic.Value
	metricFilepath, timeFunctionFilePath        string
	lock                                        sync.Mutex
}

type flatFloat64Table struct {
	values  []float64
	offsets []uint32
}

func newFlatFloat64Table(rowLengths []uint32) *flatFloat64Table {
	table := &flatFloat64Table{
		offsets: make([]uint32, len(rowLengths)+1),
	}
	var total uint64
	for i, length := range rowLengths {
		total += uint64(length)
		table.offsets[i+1] = uint32(total)
	}
	table.values = make([]float64, total)
	return table
}

func (table *flatFloat64Table) row(index da.Index) []float64 {
	return table.values[table.offsets[index]:table.offsets[index+1]]
}

func (table *flatFloat64Table) value(row da.Index, offset da.Index) float64 {
	return table.values[table.offsets[row]+uint32(offset)]
}

func NewMetric(numOfVertices int, timeFunctionFilePath string, overlayWeights *da.OverlayWeights, metricFilepath string, readBuf *bufio.Reader,
) *Metric {
	var (
		err error
		tf  *costfunction.TimeFunction
	)
	if timeFunctionFilePath != "" {
		tf, err = costfunction.ReadFromFile(timeFunctionFilePath, readBuf)
		if err != nil {
			panic(fmt.Errorf("NewMetric: failed to read time function: %v", err))
		}
	} else {
		tf = costfunction.NewTimeCostFunctionEmpty()
	}

	m := &Metric{
		metricFilepath:       metricFilepath,
		timeFunctionFilePath: timeFunctionFilePath,
		lock:                 sync.Mutex{},
		// lastSegmentSpeedFiles: make([]string, 0),
		// lastTurnPenaltyFiles:  make([]string, 0),
	}
	m.weights.Store(overlayWeights)
	emptyLengths := make([]uint32, numOfVertices)
	m.entryStallingTables.Store(newFlatFloat64Table(emptyLengths))
	m.exitStallingTables.Store(newFlatFloat64Table(emptyLengths))
	m.costFunction.Store(tf)

	m.lastSegmentSpeedFiles.Store(make([]string, 0))
	m.lastTurnPenaltyFiles.Store(make([]string, 0))

	return m
}

/*
1. query phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.

ingat bahwa pada graf standard (tanpa incorporate turn costs), kita menjalankan dijkstra dengan repeatly memilih vertex u  dengan minimum shortest path estimate dari priority queue, add ke  set of scanned vertices S
dan relax semua out edges dari vertex u

karena kita incorporate turn costs, kita menggunakan turn-aware dijkstra (see ref[1]) pada forwardGraphSearch dan backwardGraphSearch :
di implementasi ini kita pakai edgeId sebagai item dari priority queue node
di turn-aware dijkstra yang dijelaskan ref [1], kita maintain triples (v,i,d) pada priority queue
v adalah vertex id, i adalah entry point (in edge yang head nya = v) pada v, dan d adalah shortest-path estimate dari s ke v melalui entry point i.
pendekatan seperti ini lebih lambat jika dibanding dengan hanya menggunakan vertex v sebagai item di priority queue node karena size dari pq tergantung dari jumlah edges yang di scan
dan biasanya di graph road network openstreetmap jumlah edges jauh lebih banyak dibanding jumlah vertices
let dist'(s,⋅) adalah shortest path estimate dari simpul s ke any (entry point, vertex). demikian juga untuk t ke any (exit point, vertex)
di implementasi ini, kita entry ke simpul s dengan menggunakan dummy edge (s,s) dengan turn cost 0 ke exit point manapun.
di awal kita set dist'(s,(dummyEntry point, s))=0 dan dist'(t, (dummyExit point, t))=0

untuk mengurangi slowdown dari pendekatan turn-aware dijkstra, kita menerapkan teknik stalling yang dijelaskan pada ref [1]
inti dari teknik stalling adalah:
misal kita punya vertex u dengan entry point i1, i2 dan exit point j1, j2. misal outDegree dari u adalah 2
contoh turn cost dari case ini adalah ketika kita keluar dari vertex u melalui entry point i1 ke j2
disini kita simpan turn cost dari i1->u->i2 di graph.turnTables[u.turnTablePtr + 0*2+1]

kita represent turn cost dari entry i1 ke exit i2 melalui u dengna T_u[i1,i2]
misal kita udah scan (i1,u,d_{i1}) sebelumnya
(i2, u, d_{i2}) bisa lebih baik dari (i1,u,d_{i1}) iff (lebih baik maksudnya shortest path estimate dari s ke u dengan turn costs lebih baik melalui entry point i2 dibanding i1):
terdapat k in {j1, j2}, dist'(s,(i2, u)) + T_u[i2,k] <= dist'(s, (i1, u)) + T_u[i1, k]

dengan ini, kita tahu (i2, u, d_{i2}) tidak lebih baik dari (i1,u,d_{i1}) (atau  (i2, u, d_{i2}) bisa kita prune) iff:
untuk semua k in {j1, j2}, dist'(s,(i2, u)) + T_u[i2,k] > dist'(s, (i1, u)) + T_u[i1, k]
atau
dist'(s,(i2, u)) > dist'(s, (i1, u)) + max_k { T_u[i1, k] -  T_u[i2,k]}

setiap kali kita scan entry point i dari vertex v with distance dist'(s,(i,v))
kita set b_v (bs.stallingEntry di implementasi ini, tapi langsung pakai edgeId instead of (entryPoint, v)) setiap entry point k dari v, dengan
b_v[k] = min{ b_v[k], dist'(s,(i,v)) + max_j { T_v[i, j] -  T_v[k,j]} }, inisialisasi awal dari b_v[⋅] adalah infinity utk semua vertices v
setelah scan (i,v, dist'(s,(i,v))), kita relaksasi semua out edges dari v
misal salah satu edge nya adalah (v,w) dengan entry point wi1
kita gak insert (wi1, w, dist'(s,(wi1,w))) ke heap jika dist'(s,(wi1,w)) > b_w[wi1]
max_j { T_v[i, j] -  T_v[k,j]}  kita precompute untuk setiap pasang (i,k) di metric.go
yang kita implementasikan di forwardGraphSearch (dan backwardGraph search, tapi untuk backward graph search kita pakai turn cost dari exit ke entry)

karena kita incorporate turn costs, kita menggunakan turn-aware dijkstra (see ref[1]):
di implementasi ini kita pakai edgeId sebagai item dari priority queue node
*/
func (met *Metric) BuildStallingTables(overlayGraph *da.OverlayGraph, graph *da.Graph) {
	vertexCount := graph.NumberOfVertices()
	entryLengths := make([]uint32, vertexCount)
	exitLengths := make([]uint32, vertexCount)
	for vID := 0; vID < vertexCount; vID++ {
		n := uint64(graph.GetInDegree(da.Index(vID)))
		m := uint64(graph.GetOutDegree(da.Index(vID)))
		entryLengths[vID] = uint32(n * n)
		exitLengths[vID] = uint32(m * m)
	}
	entryStallingtables := newFlatFloat64Table(entryLengths)
	exitStallingTables := newFlatFloat64Table(exitLengths)
	for vId := da.Index(0); vId < da.Index(graph.NumberOfVertices()); vId++ {

		n := graph.GetInDegree(vId)
		m := graph.GetOutDegree(vId)

		if n == 0 && m == 0 {
			continue
		}

		entryStallingTable := entryStallingtables.row(vId)
		exitStallingTable := exitStallingTables.row(vId)

		for i := da.Index(0); i < n; i++ {
			for j := da.Index(0); j < n; j++ {
				maxDiff := -1.0
				for k := da.Index(0); k < m; k++ {
					Tv_ik := met.GetTurnCost(graph.GetTurnTableId(vId, i, k))
					Tv_jk := met.GetTurnCost(graph.GetTurnTableId(vId, j, k))
					maxDiff = util.MaxFloat(Tv_ik-Tv_jk, maxDiff)
				}

				// ini compute max_k{T_v[i,k] - T_v[j,k]}
				entryStallingTable[i*n+j] = maxDiff
			}
		}

		for i := da.Index(0); i < m; i++ {
			for j := da.Index(0); j < m; j++ {
				maxDiff := -1.0
				for k := da.Index(0); k < n; k++ {
					Tv_ki := met.GetTurnCost(graph.GetTurnTableId(vId, k, i))
					Tv_kj := met.GetTurnCost(graph.GetTurnTableId(vId, k, j))
					maxDiff = util.MaxFloat(Tv_ki-Tv_kj, maxDiff)
				}

				// ini compute max_k{T_v[k,i] - T_v[k,j]}

				exitStallingTable[i*m+j] = maxDiff
			}
		}
	}
	met.entryStallingTables.Store(entryStallingtables)
	met.exitStallingTables.Store(exitStallingTables)
}

func (met *Metric) GetWeights() *da.OverlayWeights {
	return met.weights.Load()
}

func (met *Metric) SetTimeFunction(tf *costfunction.TimeFunction) {
	met.costFunction.Store(tf)
}

func (met *Metric) GetCostFunction() *costfunction.TimeFunction {
	return met.costFunction.Load()
}

// GetWeight. get weight dari outEdge dengan id eId
// eId adalah id/index dari outEdge yang ingin didapat weightnya
func (met *Metric) GetWeight(eId da.Index) float64 {
	cf := met.costFunction.Load()
	return cf.GetWeight(eId)
}

func (met *Metric) GetWeightFromLength(eId da.Index, length float64) float64 {
	cf := met.costFunction.Load()
	return cf.GetWeightFromLength(eId, length)
}

func (met *Metric) GetSegmentLength(eId da.Index) float64 {
	cf := met.costFunction.Load()
	return cf.GetSegmentLength(eId)
}

func (met *Metric) GetSegmentSpeed(eId da.Index) float64 {
	cf := met.costFunction.Load()
	return cf.GetSegmentSpeed(eId)
}

// GetEntryStallingTableCost. get precomputed max_k{T_v[i,k] - T_v[j,k]}
func (met *Metric) GetEntryStallingTableCost(uId da.Index, offset da.Index) float64 {
	return met.entryStallingTables.Load().(*flatFloat64Table).value(uId, offset)
}

// GetExitStallingTableCost. get precomputed  max_k{T_v[k,i] - T_v[k,j]}
func (met *Metric) GetExitStallingTableCost(uId da.Index, offset da.Index) float64 {
	return met.exitStallingTables.Load().(*flatFloat64Table).value(uId, offset)
}

func (met *Metric) GetShortcutWeight(offset da.Index) float64 {
	weights := met.weights.Load()
	return weights.GetWeight(offset)
}

func (met *Metric) GetTurnCost(turnTableId da.Index) float64 {
	cf := met.costFunction.Load()
	return cf.GetTurnCost(turnTableId)
}

func (met *Metric) GetFilePath() string {
	return met.metricFilepath
}

func (met *Metric) SetLastSegmentSpeedFiles(filepaths []string) {
	met.lastSegmentSpeedFiles.Store(filepaths)
}

func (met *Metric) SetLastTurnPenaltyFiles(filepaths []string) {
	met.lastTurnPenaltyFiles.Store(filepaths)
}

func (met *Metric) GetLastSegmentSpeedFiles() []string {
	return met.lastSegmentSpeedFiles.Load().([]string)
}

func (met *Metric) GetLastTurnPenaltyFiles() []string {
	return met.lastTurnPenaltyFiles.Load().([]string)
}

func (met *Metric) UpdateMetrics(readBuf *bufio.Reader) error {
	newMet, err := ReadFromFile(met.metricFilepath, met.timeFunctionFilePath, readBuf)
	if err != nil {
		return fmt.Errorf("UpdateMetrics: failed to read new metrics, filepath: %s: %w", met.metricFilepath, err)
	}

	met.lock.Lock()
	defer met.lock.Unlock()

	met.weights.Store(newMet.weights.Load())
	met.entryStallingTables.Store(newMet.entryStallingTables.Load())
	met.exitStallingTables.Store(newMet.exitStallingTables.Load())
	met.costFunction.Store(newMet.costFunction.Load())

	return nil
}
