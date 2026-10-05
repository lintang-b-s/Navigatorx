// Package partitioner provides algorithms for road network graph partitioning, including CUstomizable Route Planning (CRP) multilevel partitioning and inertial flow algorithm.
package partitioner

import (
	"fmt"
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

// todo: kode di package ini bisa dioptimze secara space (memory usage) ketika partitioning
// dengan cara not creating PartitionGraph edges, vertices, etc..
// kita bisa tinggal pake pointer Graph (di graph.go) di PartitionGraph
// terus tandain pakai bitmask vertices mana aja yang masuk ke current cell yang sedang dipartisi
// mungkin cuma bikin sice of vertices bitmask dan slice of vertices data??
// tambahin wrapper ForOutEdgesOf(u, handle func(...)) dengan only iterate outgoing edges (u,v) yang v in this current cell..
// setiap kali inertial flow selesai, kita applyPermutation vertices yang masuk di current cell aja.
// sources dan sinks di ujung-ujung range slice of vertices di current cell, dan other vertices yang masuk cell S dan T di applyPermutation di selain range ujung itu.
// saat ini di commit 15e4cbe6c154b41a71d8b9abff09ddfdd97c43a0 , pakai diy_solo_semarang.osm.pbf (dengan size ~105mb) partitioner makan RAM htop RES/RSS sekitar 3GB
// setiap kali panggil selectFirstLastKthVertices() kita only cell vertices (pakai bitmask diatas)
//  osrm-partition ./data/diy_solo_semarang.osrm --max-cell-sizes 256,2048,16384,131072,262144  -> cuma ~911mb

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
for each level l, time complexity recursiveBisection.Partition() in each cell is O(log_{1/(1-b)} (U_{l+1}) * U_{l+1}^{3/2}). dengan U_{L+1}=n
*/ // nolint: gofmt
func (mp *MultilevelPartitioner) RunMultilevelPartitioning() {
	// start from highest level
	nodeIDs := mp.graph.GetVerticeIds()
	progress := newPartitionProgress(mp.graph.NumberOfVertices())
	mp.logger.Sugar().Infof("partitioning level %d with max cell size %d", mp.l, mp.u[mp.l-1])
	fmt.Printf("Level %d progress: 0%%...", mp.l)
	if len(nodeIDs) > mp.u[mp.l-1] {

		inertialFlowPartitioner := NewRecursiveBisection(mp.graph, mp.u[mp.l-1], mp.logger,
			mp.prePartitionWithSCC, mp.inertialFlowIterations, mp.directed)
		inertialFlowPartitioner.setProgress(progress)
		inertialFlowPartitioner.Partition(nodeIDs)
		mp.cellVertices[mp.l-1] = append(mp.cellVertices[mp.l-1], mp.groupEachPartition(inertialFlowPartitioner.GetFinalPartition())...)
	} else {
		mp.cellVertices[mp.l-1] = [][]da.Index{nodeIDs}
		progress.add(len(nodeIDs))
	}
	progress.finish()
	mp.logger.Sugar().Infof("level %d done, total cells: %d", mp.l, len(mp.cellVertices[mp.l-1]))

	// percent partition each cell in previous level
	for level := mp.l - 2; level >= 0; level-- {
		progress = newPartitionProgress(mp.graph.NumberOfVertices())
		mp.logger.Sugar().Infof("partitioning level %d with max cell size %d", level+1, mp.u[level])
		fmt.Printf("Level %d progress: 0%%...", level+1)

		cellInChan := make(chan []da.Index, CellInOutChanSize)
		cellOutchan := make(chan [][]da.Index, CellInOutChanSize)
		wg := sync.WaitGroup{}
		computeRecursiveBisection := func() {
			for cell := range cellInChan {
				inertialFlowPartitioner := NewRecursiveBisection(mp.graph, mp.u[level], mp.logger, mp.prePartitionWithSCC,
					mp.inertialFlowIterations, mp.directed)
				inertialFlowPartitioner.setProgress(progress)
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

		progress.finish()
		mp.logger.Sugar().Infof("level %d total cells: %d", level+1, len(mp.cellVertices[level]))
	}
}

type partitionProgress struct {
	mu        sync.Mutex
	total     int
	completed int
	percent   int
}

func newPartitionProgress(total int) *partitionProgress {
	return &partitionProgress{total: total, percent: 2}
}

func (p *partitionProgress) add(vertices int) {
	p.mu.Lock()
	defer p.mu.Unlock()

	p.completed += vertices
	for p.percent <= 100 && p.total > 0 && p.completed*100 >= p.percent*p.total {
		// while completed/total >= percent/100
		fmt.Printf("%d%%...", p.percent)
		p.percent += 2
	}
}

func (p *partitionProgress) finish() {
	fmt.Println()
}

func (mp *MultilevelPartitioner) SaveToFile() error {
	root := config.ProfilesRoot()
	filename := fmt.Sprintf("%s/%s/inertial_flow_%s.mlp", root, pkg.ProfileName, pkg.RegionName)
	if err := util.IsPathExists(filename); err != nil {
		return err
	}
	return mp.writeMLPToFile(filename)
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
