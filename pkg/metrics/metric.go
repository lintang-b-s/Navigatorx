package metrics

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"sync"
	"sync/atomic"

	"github.com/cockroachdb/errors"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	cf "github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Metric struct {
	weights                        atomic.Pointer[da.OverlayWeights]
	entryStallingTables            atomic.Value // stallingTables for vertice-v, i-th incoming edge and j-th incoming edge:  stallingTables[v][i*inDegree[v]+j]
	exitStallingTables             atomic.Value
	costFunction                   atomic.Pointer[costfunction.TimeFunction]
	filepath, timeFunctionFilePath string
	lock                           sync.Mutex
}

func NewMetric(numOfVertices int, timeFunctionFilePath string, overlayWeights *da.OverlayWeights,
) *Metric {
	var (
		err error
		tf  *cf.TimeFunction
	)
	if timeFunctionFilePath != "" {
		tf, err = cf.ReadFromFile(timeFunctionFilePath)
		if err != nil {
			panic(fmt.Errorf("NewMetric: failed to read time function: %w", err))
		}
	} else {
		tf = cf.NewTimeCostFunctionEmpty()
	}

	m := &Metric{

		filepath:             "",
		timeFunctionFilePath: timeFunctionFilePath,
		lock:                 sync.Mutex{},
	}
	m.weights.Store(overlayWeights)
	m.entryStallingTables.Store(make([][]float64, numOfVertices))
	m.exitStallingTables.Store(make([][]float64, numOfVertices))
	m.costFunction.Store(tf)
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
	entryStallingtables := met.entryStallingTables.Load().([][]float64)
	exitStallingTables := met.exitStallingTables.Load().([][]float64)
	for vId := da.Index(0); vId < da.Index(graph.NumberOfVertices()); vId++ {

		n := graph.GetInDegree(vId)
		m := graph.GetOutDegree(vId)

		if n == 0 && m == 0 {
			entryStallingtables[vId] = make([]float64, 0)
			exitStallingTables[vId] = make([]float64, 0)
			continue
		}

		entryStallingTable := make([]float64, n*n)
		exitStallingTable := make([]float64, m*m)

		for i := da.Index(0); i < n; i++ {
			for j := da.Index(0); j < n; j++ {
				maxDiff := -1.0
				for k := da.Index(0); k < m; k++ {
					Tv_ik := met.GetTurnCost(graph.GetTurnType(vId, i, k))
					Tv_jk := met.GetTurnCost(graph.GetTurnType(vId, j, k))
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
					Tv_ki := met.GetTurnCost(graph.GetTurnType(vId, k, i))
					Tv_kj := met.GetTurnCost(graph.GetTurnType(vId, k, j))
					maxDiff = util.MaxFloat(Tv_ki-Tv_kj, maxDiff)
				}

				// ini compute max_k{T_v[k,i] - T_v[k,j]}

				exitStallingTable[i*m+j] = maxDiff
			}
		}

		entryStallingtables[vId] = entryStallingTable
		exitStallingTables[vId] = exitStallingTable
	}
	met.entryStallingTables.Store(entryStallingtables)
	met.exitStallingTables.Store(exitStallingTables)
}

func (met *Metric) GetWeights() *da.OverlayWeights {
	return met.weights.Load()
}

// GetWeight. get weight dari outEdge dengan id eId
// eId adalah id/index dari outEdge yang ingin didapat weightnya
func (met *Metric) GetWeight(eId da.Index,
	eDefaultWeight float64, eLength float64) float64 {
	cf := met.costFunction.Load()
	return cf.GetWeight(eId, eDefaultWeight, eLength)

}

// GetEntryStallingTableCost. get precomputed max_k{T_v[i,k] - T_v[j,k]}
func (met *Metric) GetEntryStallingTableCost(uId da.Index, offset da.Index) float64 {

	entryStallingTable := met.entryStallingTables.Load().([][]float64)
	return entryStallingTable[uId][offset]
}

// GetExitStallingTableCost. get precomputed  max_k{T_v[k,i] - T_v[k,j]}
func (met *Metric) GetExitStallingTableCost(uId da.Index, offset da.Index) float64 {

	exitStallingTable := met.exitStallingTables.Load().([][]float64)
	return exitStallingTable[uId][offset]
}

func (met *Metric) GetShortcutWeight(offset da.Index) float64 {
	weights := met.weights.Load()
	return weights.GetWeight(offset)
}

func (met *Metric) GetTurnCost(t pkg.TurnType) float64 {
	cf := met.costFunction.Load()
	return cf.GetTurnCost(t)
}

func (met *Metric) GetFilePath() string {
	return met.filepath
}

func (met *Metric) WriteToFile(filename string) error {
	dir := filepath.Dir(filename)
	if _, err := os.Stat(dir); os.IsNotExist(err) {
		if err := os.MkdirAll(dir, 0755); err != nil {
			return err
		}
	}

	f, err := os.Create(filename)
	if err != nil {
		return errors.Wrapf(err, "metrics.WriteToFile: failed to create file %v", filename)
	}
	defer f.Close()

	w := bufio.NewWriter(f)

	weights := met.weights.Load().GetWeights()
	entryStallingTables := met.entryStallingTables.Load().([][]float64)
	exitStallingTables := met.exitStallingTables.Load().([][]float64)
	fmt.Fprintf(w, "%d %d %d\n", len(weights), len(entryStallingTables), len(exitStallingTables))
	for i, weight := range weights {

		_, err = fmt.Fprintf(w, "%s", strconv.FormatFloat(weight, 'f', -1, 64))
		if err != nil {
			return errors.Wrapf(err, "metrics.WriteToFile: failed to write metrics weight %v", weight)
		}
		if i < len(weights)-1 {
			_, err := fmt.Fprintf(w, " ")
			if err != nil {
				return errors.Wrapf(err, "metrics.WriteToFile: failed to write metrics weight")
			}
		}
	}
	_, err = fmt.Fprintf(w, "\n")
	if err != nil {
		return errors.Wrapf(err, "metrics.WriteToFile: failed to write new line")
	}

	for i := range entryStallingTables {
		if entryStallingTables[i] == nil {
			_, err = fmt.Fprintf(w, "0\n")
			if err != nil {
				return errors.Wrapf(err, "metrics.WriteToFile: failed to write 0\n")
			}

			continue
		}
		_, err = fmt.Fprintf(w, "%d ", len(entryStallingTables[i]))
		if err != nil {
			return errors.Wrapf(err, "metrics.WriteToFile: failed to write len(entryStallingTables[i]): %v", len(entryStallingTables[i]))
		}
		for j, val := range entryStallingTables[i] {
			_, err = fmt.Fprintf(w, "%s", strconv.FormatFloat(val, 'f', -1, 64))
			if err != nil {
				return errors.Wrapf(err, "metrics.WriteToFile: failed to write entryStallingTables[i]: %v", entryStallingTables[i])
			}

			if j < len(entryStallingTables[i])-1 {
				_, err = fmt.Fprintf(w, " ")
				if err != nil {
					return errors.Wrapf(err, "metrics.WriteToFile: failed to write new line")
				}
			}
		}
		_, err = fmt.Fprintf(w, "\n")
		if err != nil {
			return errors.Wrapf(err, "metrics.WriteToFile: failed to write new line")
		}
	}

	for i := range exitStallingTables {
		if exitStallingTables[i] == nil {
			_, err = fmt.Fprintf(w, "0\n")
			if err != nil {
				return errors.Wrapf(err, "metrics.WriteToFile: failed to write 0\n")
			}
			continue
		}
		_, err = fmt.Fprintf(w, "%d ", len(exitStallingTables[i]))
		if err != nil {
			return errors.Wrapf(err, "metrics.WriteToFile: failed to write len(exitStallingTables[i]): %v", len(exitStallingTables[i]))
		}
		for j, val := range exitStallingTables[i] {
			_, err = fmt.Fprintf(w, "%s", strconv.FormatFloat(val, 'f', -1, 64))
			if err != nil {
				return errors.Wrapf(err, "metrics.WriteToFile: failed to write exitStallingTables[i]: %v", exitStallingTables[i])
			}
			if j < len(exitStallingTables[i])-1 {
				_, err = fmt.Fprintf(w, " ")
				if err != nil {
					return errors.Wrapf(err, "metrics.WriteToFile: failed to write new line")
				}
			}
		}
		_, err = fmt.Fprintf(w, "\n")
		if err != nil {
			return errors.Wrapf(err, "metrics.WriteToFile: failed to write new line")
		}
	}

	if err = w.Flush(); err != nil {
		return errors.Wrapf(err, "metric.WriteToFile: failed to flush bufio writer")
	}

	return nil
}

const (
	metricsBufferSize = 4096 * 2
)

func ReadFromFile(filename string, timeFunctionFilePath string) (*Metric, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to open file %v", filename)
	}

	defer f.Close()

	r := bufio.NewReaderSize(f, metricsBufferSize)

	line, err := util.ReadLine(r)
	if err != nil {
		return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to util.ReadLine(r)")
	}

	parts := util.Fields(line)
	if len(parts) != 3 {
		return nil, errors.Errorf("metrics.ReadFromFile: expected 3 header fields, got %v", len(parts))
	}

	var numWeights uint32
	var numEntryStallingTables int
	var numExitStallingTables int

	numWeights, err = util.ParseUInt32(parts[0])
	if err != nil {
		return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to parse numWeights: %v", parts[0])
	}

	numEntryStallingTables, err = util.ParseInt(parts[1])
	if err != nil {
		return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to parse numEntryStallingTables: %v", parts[1])
	}

	numExitStallingTables, err = util.ParseInt(parts[2])
	if err != nil {
		return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to parse numExitStallingTables: %v", parts[2])
	}

	line, err = util.ReadLine(r)
	if err != nil {
		return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to util.ReadLine(r)")

	}

	weights := make([]float64, numWeights)
	parts = util.Fields(line)

	if uint32(len(parts)) != numWeights {
		return nil, errors.Errorf("metrics.ReadFromFile: expected %v weights, got %v", numWeights, len(parts))
	}

	for i, weight := range parts {
		weights[i], err = strconv.ParseFloat(weight, 64)
		if err != nil {
			return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to parse weight[%d]: %v", i, weight)
		}
	}

	entryStallingTables := make([][]float64, numEntryStallingTables)
	exitStallingTables := make([][]float64, numExitStallingTables)

	for i := 0; i < numEntryStallingTables; i++ {
		line, err = util.ReadLine(r)
		if err != nil {
			return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to util.ReadLine(r)")
		}

		parts = util.Fields(line)
		if len(parts) == 1 && parts[0] == "0" {
			continue
		}

		if len(parts) < 2 {
			return nil, errors.Errorf("metrics.ReadFromFile: invalid entryStallingTables[%d] format", i)
		}

		var numElements int
		numElements, err = util.ParseInt(parts[0])
		if err != nil {
			return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to parse entryStallingTables[%d] size: %v", i, parts[0])
		}

		if len(parts)-1 != numElements {
			return nil, errors.Errorf("metrics.ReadFromFile: entryStallingTables[%d] expected %d elements, got %d", i, numElements, len(parts)-1)
		}

		stallingTable := make([]float64, numElements)
		for j, val := range parts[1:] {
			stallingTable[j], err = strconv.ParseFloat(val, 64)
			if err != nil {
				return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to parse entryStallingTables[%d][%d]: %v", i, j, val)
			}
		}

		entryStallingTables[i] = stallingTable
	}

	for i := 0; i < numExitStallingTables; i++ {
		line, err = util.ReadLine(r)
		if err != nil {
			return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to util.ReadLine(r)")
		}

		parts = util.Fields(line)
		if len(parts) == 1 && parts[0] == "0" {
			continue
		}

		if len(parts) < 2 {
			return nil, errors.Errorf("metrics.ReadFromFile: invalid exitStallingTables[%d] format", i)
		}

		var numElements int
		numElements, err = util.ParseInt(parts[0])
		if err != nil {
			return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to parse exitStallingTables[%d] size: %v", i, parts[0])
		}

		if len(parts)-1 != numElements {
			return nil, errors.Errorf("metrics.ReadFromFile: exitStallingTables[%d] expected %d elements, got %d", i, numElements, len(parts)-1)
		}

		stallingTable := make([]float64, numElements)
		for j, val := range parts[1:] {
			stallingTable[j], err = strconv.ParseFloat(val, 64)
			if err != nil {
				return nil, errors.Wrapf(err, "metrics.ReadFromFile: failed to parse exitStallingTables[%d][%d]: %v", i, j, val)
			}
		}

		exitStallingTables[i] = stallingTable
	}
	tf, err := cf.ReadFromFile(timeFunctionFilePath)
	if err != nil {
		panic(fmt.Errorf("NewMetric: failed to read time function: %w", err))
	}
	metric := &Metric{
		filepath: filename,
	}

	ovWeights := da.NewOverlayWeights(numWeights)
	ovWeights.SetWeights(weights)
	metric.weights.Store(ovWeights)
	metric.entryStallingTables.Store(entryStallingTables)
	metric.exitStallingTables.Store(exitStallingTables)
	metric.costFunction.Store(tf)
	return metric, nil
}

func (met *Metric) UpdateMetrics() error {
	newMet, err := ReadFromFile(met.filepath, met.timeFunctionFilePath)
	if err != nil {
		return errors.Wrapf(err, "error reading new metrics, filepath: %s", met.filepath)
	}

	met.lock.Lock()
	defer met.lock.Unlock()

	met.weights.Store(newMet.weights.Load())
	met.entryStallingTables.Store(newMet.entryStallingTables.Load())
	met.exitStallingTables.Store(newMet.exitStallingTables.Load())
	met.costFunction.Store(newMet.costFunction.Load())

	return nil
}
