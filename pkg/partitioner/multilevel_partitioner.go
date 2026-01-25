package partitioner

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"go.uber.org/zap"
)

type MultilevelPartitioner struct {
	u []int //  cell size for  each cell levels. from biggest to smallest.
	// best parameter for customizable route planning by delling et al:
	// [2^8, 2^11, 2^14, 2^17, 2^20]
	l            int                       // max level of overlay graph
	overlayNodes [][][]datastructure.Index // nodes in each cells in each level
	graph        *datastructure.Graph
	logger       *zap.Logger
}

func NewMultilevelPartitioner(u []int, l int, graph *datastructure.Graph, logger *zap.Logger) *MultilevelPartitioner {
	if len(u) != l {
		panic(fmt.Sprintf("cell levels %d and cell array size %d must be the same", l, len(u)))
	}
	return &MultilevelPartitioner{
		u:            u,
		l:            l,
		overlayNodes: make([][][]datastructure.Index, l),
		graph:        graph,
		logger:       logger,
	}
}

/*
RunMultilevelPartitioning. run L-level partitioning using inertial flow algorithm with U1 , . . . , UL maximum cell sizes.

Customizable Route Planning in Road Networks, Delling et al.
We then use PUNCH to generate an L-level partition (with maximum cell sizes U1 , . . . , UL ) in top-down fashion. We first run
PUNCH with parameter UL to obtain the top-level cells. Cells in lower levels are then obtained by running
PUNCH on individual cells of the level immediately abov
*/
func (mp *MultilevelPartitioner) RunMultilevelPartitioning() {
	// start from highest level
	nodeIDs := mp.graph.GetVerticeIds()

	mp.logger.Sugar().Infof("partitioning level %d with max cell size %d", mp.l-1, mp.u[mp.l-1])
	if len(nodeIDs) > mp.u[mp.l-1] {

		inertialFlowPartitioner := NewRecursiveBisection(mp.graph, mp.u[mp.l-1], mp.logger)
		inertialFlowPartitioner.Partition(nodeIDs)
		mp.overlayNodes[mp.l-1] = append(mp.overlayNodes[mp.l-1], mp.groupEachPartition(inertialFlowPartitioner.GetFinalPartition())...)
	} else {
		mp.overlayNodes[mp.l-1] = [][]datastructure.Index{nodeIDs}
	}
	mp.logger.Sugar().Infof("level %d done, total cells: %d", mp.l-1, len(mp.overlayNodes[mp.l-1]))

	// next partition each cell in previous level
	for level := mp.l - 2; level >= 0; level-- {
		mp.logger.Sugar().Infof("partitioning level %d with max cell size %d", level, mp.u[level])
		for cellId, cell := range mp.overlayNodes[level+1] {
			inertialFlowPartitioner := NewRecursiveBisection(mp.graph, mp.u[level], mp.logger)
			inertialFlowPartitioner.Partition(cell)

			partitions := mp.groupEachPartition(inertialFlowPartitioner.GetFinalPartition())
			mp.logger.Sugar().Infof("level %d, cellId %d done, total cells: %d", level, cellId, len(partitions))
			mp.overlayNodes[level] = append(mp.overlayNodes[level], partitions...)
		}
		mp.logger.Sugar().Infof("level %d total cells: %d", level, len(mp.overlayNodes[level]))
	}
}



func (mp *MultilevelPartitioner) SaveToFile(name string) error {
	return mp.writeMLPToMLPFile(fmt.Sprintf("crp_inertial_flow_%s.mlp", name))
}

func (mp *MultilevelPartitioner) groupEachPartition(partition []int) [][]datastructure.Index {
	cells := make([][]datastructure.Index, 0)
	for nodeId, cellId := range partition {
		if cellId == INVALID_PARTITION_ID {
			continue
		}

		for len(cells) <= cellId {
			cells = append(cells, make([]datastructure.Index, 0))
		}
		cells[cellId] = append(cells[cellId], datastructure.Index(nodeId))
	}
	return cells
}
