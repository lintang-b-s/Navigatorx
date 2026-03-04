package partitioner

import (
	"fmt"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
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
	unitCapacity bool
	minPrec      int
}

func NewMultilevelPartitioner(u []int, l int, graph *datastructure.Graph, logger *zap.Logger, unitCapacity bool) *MultilevelPartitioner {
	if len(u) != l {
		panic(fmt.Sprintf("cell levels %d and cell array size %d must be the same", l, len(u)))
	}
	return &MultilevelPartitioner{
		u:            u,
		l:            l,
		overlayNodes: make([][][]datastructure.Index, l),
		graph:        graph,
		logger:       logger,
		unitCapacity: unitCapacity,
	}
}

/*
RunMultilevelPartitioning. run L-level partitioning using inertial flow algorithm with U1 , . . . , UL maximum cell sizes.

Customizable Route Planning in Road Networks, Delling et al.
We then use PUNCH to generate an L-level partition (with maximum cell sizes U1 , . . . , UL ) in top-down fashion. We first run
PUNCH with parameter UL to obtain the top-level cells. Cells in lower levels are then obtained by running
PUNCH on individual cells of the level immediately above

time complexity:
T(N, U1,...,UL) \in O(N^4 * (N-U_{L}) + \sum_{l=2}^{L} U_l^4 *(U_{l} - U_{l-1}))
*/
func (mp *MultilevelPartitioner) RunMultilevelPartitioning() {
	// start from highest level
	nodeIDs := mp.graph.GetVerticeIds()
	k := mp.MinPrecision()

	mp.logger.Sugar().Infof("partitioning level %d with max cell size %d", mp.l, mp.u[mp.l-1])
	if len(nodeIDs) > mp.u[mp.l-1] {

		inertialFlowPartitioner := NewRecursiveBisection(mp.graph, mp.u[mp.l-1], mp.logger, k, mp.unitCapacity)
		inertialFlowPartitioner.Partition(nodeIDs)
		mp.overlayNodes[mp.l-1] = append(mp.overlayNodes[mp.l-1], mp.groupEachPartition(inertialFlowPartitioner.GetFinalPartition())...)
	} else {
		mp.overlayNodes[mp.l-1] = [][]datastructure.Index{nodeIDs}
	}
	mp.logger.Sugar().Infof("level %d done, total cells: %d", mp.l, len(mp.overlayNodes[mp.l-1]))

	// next partition each cell in previous level
	for level := mp.l - 2; level >= 0; level-- {
		mp.logger.Sugar().Infof("partitioning level %d with max cell size %d", level+1, mp.u[level])
		for _, cell := range mp.overlayNodes[level+1] {
			inertialFlowPartitioner := NewRecursiveBisection(mp.graph, mp.u[level], mp.logger, k, mp.unitCapacity)
			inertialFlowPartitioner.Partition(cell)

			partitions := mp.groupEachPartition(inertialFlowPartitioner.GetFinalPartition())
			mp.overlayNodes[level] = append(mp.overlayNodes[level], partitions...)
		}
		mp.logger.Sugar().Infof("level %d total cells: %d", level+1, len(mp.overlayNodes[level]))

	}
}

func (mp *MultilevelPartitioner) SaveToFile(name string) error {
	return mp.writeMLPToMLPFile(fmt.Sprintf("./data/crp_inertial_flow_%s.mlp", name))
}

func (mp *MultilevelPartitioner) groupEachPartition(partition []int) [][]datastructure.Index {
	cells := make([][]datastructure.Index, 0)
	for nodeId, cellId := range partition {
		if cellId == INVALID_PARTITION_ID {
			continue
		}

		if len(cells) <= cellId {
			n := len(cells)
			for j := 0; j < cellId-n+1; j++ {
				cells = append(cells, make([]datastructure.Index, 0))
			}
		}
		cells[cellId] = append(cells[cellId], datastructure.Index(nodeId))
	}
	return cells
}

func (mp *MultilevelPartitioner) MinPrecision() int {
	minWeight := math.MaxFloat64
	mp.graph.ForOutEdges(func(e *datastructure.OutEdge, exitPoint, head, tail, entryId datastructure.Index, percentage float64, idx datastructure.Index) {
		eWeight := e.GetWeight()
		if da.Eq(eWeight, 0) {
			return
		}
		minWeight = math.Min(eWeight, minWeight)
	})

	prec := util.CountDecimalPlacesF64(minWeight)
	prec = util.MinInt(prec, mp.minPrec)
	return prec
}

func (mp *MultilevelPartitioner) SetMinPrec(k int) {
	mp.minPrec = k
}
