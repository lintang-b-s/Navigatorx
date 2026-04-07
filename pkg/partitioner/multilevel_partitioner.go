package partitioner

import (
	"context"
	"fmt"
	"sync"

	"github.com/bytedance/gopkg/util/gopool"
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

func (mpr *MultilevelPartitioner) GetCellVertices() [][][]da.Index {
	return mpr.cellVertices
}

func (mpr *MultilevelPartitioner) SetCellVertices(cellVertices [][][]da.Index) {
	mpr.cellVertices = cellVertices
}

/*
RunMultilevelPartitioning. run L-level mutltilevel partitioning using inertial flow algorithm with U1 , . . . , UL maximum cell sizes.
pertama jalankan algoritma intertial flow pada graf G dengan parameter U_{L} untuk mendapatkan cells level L.
cells di level bawahnya didapatkan dengan menjalankan algoritma inertial flow pada individual cells of the level immediately above.

time complexity:
for each level l, time complexity recursiveBisection.Partition() is O(U_{l+1}5), dengan U_{L+1}=n
T(n, U1,...,UL) = O(n^5) + \sum_{l=2}^{L} O(U_l^5 * n/U_l) = O(n^5)
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
		numOfCellInLev := len(mp.cellVertices[level+1])

		cellInChan := make(chan []da.Index, numOfCellInLev)
		cellOutchan := make(chan [][]da.Index, numOfCellInLev)
		wg := sync.WaitGroup{}
		computeRecursiveBisection := func() {
			for cell := range cellInChan {
				inertialFlowPartitioner := NewRecursiveBisection(mp.graph, mp.u[level], mp.logger, mp.prePartitionWithSCC,
					mp.inertialFlowIterations, mp.directed)
				inertialFlowPartitioner.Partition(cell)
				partitions := mp.groupEachPartition(inertialFlowPartitioner.GetFinalPartition())
				cellOutchan <- partitions
				wg.Done()
			}
		}

		for q := 0; q < LEVEL_WORKERS; q++ {
			gopool.CtxGo(context.Background(), computeRecursiveBisection)
		}

		for _, cell := range mp.cellVertices[level+1] {
			wg.Add(1)
			cellInChan <- cell
		}

		close(cellInChan)

		go func() {
			wg.Wait()
			close(cellOutchan)
		}()

		for partitions := range cellOutchan {
			mp.cellVertices[level] = append(mp.cellVertices[level], partitions...)
		}

		mp.logger.Sugar().Infof("level %d total cells: %d", level+1, len(mp.cellVertices[level]))
	}
}

func (mp *MultilevelPartitioner) SaveToFile(name string) error {
	return mp.writeMLPToMLPFile(fmt.Sprintf("./data/%s.mlp", name))
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
