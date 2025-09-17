package metrics

import (
	"bufio"
	"errors"
	"fmt"
	"io"
	"math"
	"os"
	"strings"

	"github.com/lintang-b-s/navigatorx-crp/pkg/costfunction"
	"github.com/lintang-b-s/navigatorx-crp/pkg/datastructure"
)

type Metric struct {
	weights             *datastructure.OverlayWeights
	entryStallingTables [][]float64 // stallingTables for vertice-v, i-th incoming edge and j-th incoming edge:  stallingTables[v][i*inDegree[v]+j]
	exitStallingTables  [][]float64
	costFunction        costfunction.CostFunction
}

func NewMetric(graph *datastructure.Graph, costFunction costfunction.CostFunction, overlayWeights *datastructure.OverlayWeights) *Metric {

	return &Metric{
		weights:             overlayWeights,
		entryStallingTables: make([][]float64, graph.NumberOfVertices()),
		exitStallingTables:  make([][]float64, graph.NumberOfVertices()),
		costFunction:        costFunction,
	}
}

/*
BuildStallingTables. Customizable Route Planning, Daniel Delling, et al. Page 7:

To determine quickly whether a distance label at an entry point can improve the implicit distance labels
of the exit points, we keep an array of size p (the number of entry points) for each vertex v, with each entry
denoted by b_v[i]. We initialize all values in the array with ∞. Whenever we scan an entry point i at a vertex
v with a distance d, we set each b_v[k] to min{b_v[k], d + max_j {Tv [i, j] − Tv [k, j]}}, with j denoting the exit
points of v. However, to properly deal with turn restrictions, we must not update  b_v[k] if there exists an exit
point that can be reached from k, but not from i. We then prune the search as follows: when processing an
element (u, i, d), we only insert it into the heap if d ≤ bu [i].

we precompute (during customization) the max_k {Tv [i, k] − Tv [j, k]} entries for all pairs of entry points of each vertex.
*/
func (met *Metric) BuildStallingTables(overlayGraph *datastructure.OverlayGraph, graph *datastructure.Graph) {

	for vId := datastructure.Index(0); vId < datastructure.Index(graph.NumberOfVertices()); vId++ {

		n := graph.GetInDegree(vId)
		m := graph.GetOutDegree(vId)

		if n == 0 || m == 0 {
			continue
		}

		entryStallingTable := make([]float64, n*n)
		exitStallingTable := make([]float64, m*m)

		for i := datastructure.Index(0); i < n; i++ {
			for j := datastructure.Index(0); j < n; j++ {
				maxDiff := -1.0
				for k := datastructure.Index(0); k < m; k++ {
					Tv_ik := graph.GetTurnType(vId, i, k)
					Tv_jk := graph.GetTurnType(vId, j, k)
					maxDiff = math.Max(maxDiff, float64(Tv_ik-Tv_jk))
				}

				entryStallingTable[i*n+j] = maxDiff
			}

			for i := datastructure.Index(0); i < m; i++ {
				for j := datastructure.Index(0); j < m; j++ {
					maxDiff := -1.0
					for k := datastructure.Index(0); k < n; k++ {
						Tv_ki := graph.GetTurnType(vId, k, i)
						Tv_kj := graph.GetTurnType(vId, k, j)
						maxDiff = math.Max(maxDiff, float64(Tv_ki-Tv_kj))
					}

					exitStallingTable[i*m+j] = maxDiff
				}
			}

			met.entryStallingTables[vId] = entryStallingTable
			met.exitStallingTables[vId] = exitStallingTable
		}
	}
	return
}

func (met *Metric) WriteToFile(filename string) error {
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()

	w := bufio.NewWriter(f)
	defer w.Flush()

	fmt.Fprintf(w, "%d %d %d", len(met.weights.GetWeights()), len(met.entryStallingTables), len(met.exitStallingTables))
	for i, weight := range met.weights.GetWeights() {
		fmt.Fprintf(w, "%f", weight)
		if i < len(met.weights.GetWeights())-1 {
			fmt.Fprintf(w, " ")
		}
	}
	fmt.Fprintf(w, "\n")

	for i := range met.entryStallingTables {
		if met.entryStallingTables[i] == nil {
			fmt.Fprintf(w, "0\n")
			continue
		}
		fmt.Fprintf(w, "%d ", len(met.entryStallingTables[i]))
		for j, val := range met.entryStallingTables[i] {
			fmt.Fprintf(w, "%f", val)
			if j < len(met.entryStallingTables[i])-1 {
				fmt.Fprintf(w, " ")
			}
		}
		fmt.Fprintf(w, "\n")
	}

	for i := range met.exitStallingTables {
		if met.exitStallingTables[i] == nil {
			fmt.Fprintf(w, "0\n")
			continue
		}
		fmt.Fprintf(w, "%d ", len(met.exitStallingTables[i]))
		for j, val := range met.exitStallingTables[i] {
			fmt.Fprintf(w, "%f", val)
			if j < len(met.exitStallingTables[i])-1 {
				fmt.Fprintf(w, " ")
			}
		}
		fmt.Fprintf(w, "\n")
	}

	return nil
}

func ReadFromFile(filename string) (*Metric, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, err
	}

	defer f.Close()

	r := bufio.NewReader(f)
	readLine := func() (string, error) {
		line, err := r.ReadString('\n')
		if err != nil {
			if errors.Is(err, io.EOF) && len(line) > 0 {
			} else if err != nil {
				return "", err
			}
		}
		return strings.TrimRight(line, "\r\n"), nil
	}

	line, err := readLine()
	if err != nil {
		return nil, err
	}

	parts := fields(line)
	if len(parts) != 3 {
		return nil, fmt.Errorf("invalid format")
	}

	numWeights := uint32(0)
	numEntryStallingTables := 0
	numExitStallingTables := 0
	_, err = fmt.Sscanf(parts[0], "%d", &numWeights)
	if err != nil {
		return nil, err
	}
	_, err = fmt.Sscanf(parts[1], "%d", &numEntryStallingTables)
	if err != nil {
		return nil, err
	}
	_, err = fmt.Sscanf(parts[2], "%d", &numExitStallingTables)
	if err != nil {
		return nil, err
	}

	line, err = readLine()
	weights := make([]float64, numWeights)
	parts = fields(line)
	if uint32(len(parts)) != numWeights {
		return nil, fmt.Errorf("invalid format")
	}
	for i, weight := range parts {
		_, err = fmt.Sscanf(weight, "%f", &weights[i])
		if err != nil {
			return nil, err
		}
	}

	entryStallingTables := make([][]float64, numEntryStallingTables)
	exitStallingTables := make([][]float64, numExitStallingTables)

	for i := 0; i < numEntryStallingTables; i++ {
		line, err = readLine()
		if err != nil {
			return nil, err
		}
		parts = fields(line)
		if len(parts) == 1 && parts[0] == "0" {
			continue
		}
		if len(parts) < 2 {
			return nil, fmt.Errorf("invalid format")
		}

		numElements := 0
		_, err = fmt.Sscanf(parts[0], "%d", &numElements)
		if err != nil {
			return nil, err
		}
		if len(parts)-1 != numElements {
			return nil, fmt.Errorf("invalid format")
		}

		stallingTable := make([]float64, numElements)
		for j, val := range parts[1:] {
			_, err = fmt.Sscanf(val, "%f", &stallingTable[j])
			if err != nil {
				return nil, err
			}
		}
		entryStallingTables[i] = stallingTable
	}

	for i := 0; i < numExitStallingTables; i++ {
		line, err = readLine()
		if err != nil {
			return nil, err
		}
		parts = fields(line)
		if len(parts) == 1 && parts[0] == "0" {
			continue
		}
		if len(parts) < 2 {
			return nil, fmt.Errorf("invalid format")
		}

		numElements := 0
		_, err = fmt.Sscanf(parts[0], "%d", &numElements)
		if err != nil {
			return nil, err
		}
		if len(parts)-1 != numElements {
			return nil, fmt.Errorf("invalid format")
		}

		stallingTable := make([]float64, numElements)
		for j, val := range parts[1:] {
			_, err = fmt.Sscanf(val, "%f", &stallingTable[j])
			if err != nil {
				return nil, err
			}
		}
		exitStallingTables[i] = stallingTable
	}

	metric := &Metric{
		weights:             datastructure.NewOverlayWeights(numWeights),
		entryStallingTables: entryStallingTables,
		exitStallingTables:  exitStallingTables,
	}
	metric.weights.SetWeights(weights)

	return metric, nil

}

func fields(s string) []string {
	return strings.Fields(s)
}
