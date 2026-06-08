// Package partitioner provides algorithms for road network graph partitioning, including CUstomizable Route Planning (CRP) multilevel partitioning and inertial flow algorithm.
package partitioner

import (
	"fmt"
	"sync"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"go.uber.org/zap"
)

type MultilevelPartitioner struct {
	u []int //  cell size for  each cell levels. from biggest to smallest.
	// best parameter for customizable route planning by delling et al:
	// [2^8, 2^11, 2^14, 2^17, 2^20]
	l                      int            // max level of overlay graph
	cellVertices           [][][]da.Index // nodes in each cells in each level
	graph                  *da.Graph
	logger                 *zap.Logger
	prePartitionWithSCC    bool
	inertialFlowIterations int
	directed               bool
}

func NewMultilevelPartitioner(u []int, l, inertialFlowIterations int, graph *da.Graph, logger *zap.Logger, prePartitionWithSCC, directed bool) *MultilevelPartitioner {
	if len(u) != l {
		panic(fmt.Errorf("cell levels %d and cell array size %d must be the same", l, len(u)))
	}

	return &MultilevelPartitioner{
		u:                      u,
		l:                      l,
		cellVertices:           make([][][]da.Index, l),
		graph:                  graph,
		logger:                 logger,
		prePartitionWithSCC:    prePartitionWithSCC,
		inertialFlowIterations: inertialFlowIterations,
		directed:               directed,
	}
}

func (mp *MultilevelPartitioner) GetCellVertices() [][][]da.Index {
	return mp.cellVertices
}

func (mp *MultilevelPartitioner) SetCellVertices(cellVertices [][][]da.Index) {
	mp.cellVertices = cellVertices
}

/*
RunMultilevelPartitioning. Partitioning phase of Customizable Route Planning (CRP) By Delling et al. see section 5.1 Metric Independent Preprocessing (Partitioning) :  https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

 run L-level mutltilevel partitioning using inertial flow algorithm with U1 , . . . , UL maximum cell sizes.
pertama jalankan algoritma intertial flow pada graf G dengan parameter U_{L} untuk mendapatkan cells level L.
cells di level bawahnya didapatkan dengan menjalankan algoritma inertial flow pada individual cells of the level immediately above.

time complexity:
ref2: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf
see lemama 4.2 ref2, dinic unit capacity graph worst case: O(min{m * sqrt(m), m * n^(2/3)})
karena di implementasi inertial flow ini kita selalu pakai unit capacity..
let T_d(n)=worst case time complexity dinic algorithm on unit capacity graph pada graph n vertices dan m edges = O(min{m * sqrt(m), m * n^(2/3)})
for each level l, time complexity recursiveBisection.Partition() in each cell is O(2^{log_{1/(1-b)} U_{l+1}}* T_d(U_{l+1})), dengan U_{L+1}=n
*/

func (mp *MultilevelPartitioner) RunMultilevelPartitioning() {
	// start from highest level
	nodeIDs := mp.graph.GetVerticeIds()
	mp.logger.Sugar().Infof("partitioning level %d with max cell size %d", mp.l, mp.u[mp.l-1])
	if len(nodeIDs) > mp.u[mp.l-1] {

		inertialFlowPartitioner := NewRecursiveBisection(mp.graph, mp.u[mp.l-1], mp.logger,
			mp.prePartitionWithSCC, mp.inertialFlowIterations, mp.directed)
		inertialFlowPartitioner.Partition(nodeIDs)
		mp.cellVertices[mp.l-1] = append(mp.cellVertices[mp.l-1], mp.groupEachPartition(inertialFlowPartitioner.GetFinalPartition())...)
	} else {
		mp.cellVertices[mp.l-1] = [][]da.Index{nodeIDs}
	}
	mp.logger.Sugar().Infof("level %d done, total cells: %d", mp.l, len(mp.cellVertices[mp.l-1]))

	// next partition each cell in previous level
	for level := mp.l - 2; level >= 0; level-- {
		mp.logger.Sugar().Infof("partitioning level %d with max cell size %d", level+1, mp.u[level])

		cellInChan := make(chan []da.Index, CellInOutChanSize)
		cellOutchan := make(chan [][]da.Index, CellInOutChanSize)
		wg := sync.WaitGroup{}
		computeRecursiveBisection := func() {
			for cell := range cellInChan {
				inertialFlowPartitioner := NewRecursiveBisection(mp.graph, mp.u[level], mp.logger, mp.prePartitionWithSCC,
					mp.inertialFlowIterations, mp.directed)
				inertialFlowPartitioner.Partition(cell)
				partitions := mp.groupEachPartition(inertialFlowPartitioner.GetFinalPartition())
				cellOutchan <- partitions
			}
		}

		go func() {
			for partitions := range cellOutchan {
				mp.cellVertices[level] = append(mp.cellVertices[level], partitions...)
				wg.Done()
			}
		}()

		for q := 0; q < LEVEL_WORKERS; q++ {
			go computeRecursiveBisection()
		}

		for _, cell := range mp.cellVertices[level+1] {
			wg.Add(1)
			cellInChan <- cell
		}

		close(cellInChan)

		wg.Wait()
		close(cellOutchan)

		mp.logger.Sugar().Infof("level %d total cells: %d", level+1, len(mp.cellVertices[level]))
	}
}

func (mp *MultilevelPartitioner) SaveToFile(filename string) error {
	return mp.writeMLPToMLPFile(filename)
}

func (mp *MultilevelPartitioner) groupEachPartition(partition []int) [][]da.Index {
	cellSet := make(map[int]struct{})
	for _, cellId := range partition {
		cellSet[cellId] = struct{}{}
	}
	cells := make([][]da.Index, len(cellSet))

	for nodeId, cellId := range partition {
		if cellId == -1 {
			continue
		}
		cells[cellId] = append(cells[cellId], da.Index(nodeId))
	}
	return cells // cellId -> vertices Id
}
