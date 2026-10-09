// Package partitioner provides algorithms for road network graph partitioning, including CUstomizable Route Planning (CRP) multilevel partitioning and inertial flow algorithm.
package partitioner

import (
	"fmt"
	"math/rand"
	"sync"
	"time"

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
// osrm-partition ./data/diy_solo_semarang.osrm --max-cell-sizes 256,2048,16384,131072,262144  -> cuma ~911mb. hampir 9x dari ukuran file osmnya..

// partially done :), currently (7 oktober 2026) diy_solo_semarang.osm.pbf (105mb) peak htop RES/RSS  ~2.8gb. masih sekitar 27x dari ukuran file osmnya wkwkwkw :v, masih kalah jauh sama osrm-partition (https://github.com/Project-OSRM/osrm-backend/tree/master/src/partitioner) .
// mungkin next time, bisa cek pprof dari call di code partitioner....
// tapi sekarang lebih bagus runtimenya, sebelumnya sekitar 450s di versi baru partition langsung edge-based graph.. sekarang ~240s doang... :)
// keknya bisa pakai idenya osrm-partition kalau kita cukup partition node-based graph aja, then untuk assign cellId edge-based graph nodes nya pakai heuristic.. lihat getGraphBisection() di https://github.com/Project-OSRM/osrm-backend/blob/master/src/partitioner/partitioner.cpp
// dan edge_based_partition_ids di  https://github.com/Project-OSRM/osrm-backend/blob/master/src/partitioner/partitioner.cpp
// diskusi lengkap on why OSRM milih partisi ke node-based graph instead directly partisi edge-based graph ada di: https://github.com/Project-OSRM/osrm-backend/issues/3205#issuecomment-275169844
// karena number of nodes dari node-based graph lebih kecil dari edge-based graph, harusnya runtime + space nya lebih kecil...
// ok todo2: partition node-based graph, then use heuristic to assign cellId of each edge-based graph nodes.
// done :)
// sekarang cuma peak htop RES 1.7gb, cuma 16x dari ukuran file osm.. better, runtime cuma ~110s....
// jumlah boundary/overlay vertices juga lebih kecil, dari ~240k jadi ~210k
// tapi kok number of shortcut edges pas customization lebih banyak ya??
// hasil load test juga lebih jelek

type MultilevelPartitioner struct {
	u []int //  cell size for  each cell levels. from biggest to smallest.
	// best parameter for customizable route planning by delling et al:
	// [2^8, 2^11, 2^14, 2^17, 2^20]
	l                      int            // max level of overlay graph
	cellVertices           [][][]da.Index // nodes in each cells in each level
	graph                  *da.Graph
	logger                 *zap.Logger
	inertialFlowIterations int
}

func NewMultilevelPartitioner(u []int, l, inertialFlowIterations int, nbg *da.Graph, logger *zap.Logger) *MultilevelPartitioner {
	if len(u) != l {
		panic(fmt.Errorf("cell levels %d and cell array size %d must be the same", l, len(u)))
	}

	return &MultilevelPartitioner{
		u:                      u,
		l:                      l,
		cellVertices:           make([][][]da.Index, l),
		graph:                  nbg,
		logger:                 logger,
		inertialFlowIterations: inertialFlowIterations,
	}
}

func (mp *MultilevelPartitioner) GetCellVertices() [][][]da.Index {
	return mp.cellVertices
}

func (mp *MultilevelPartitioner) SetCellVertices(cellVertices [][][]da.Index) {
	mp.cellVertices = cellVertices
}

/*
RunMultilevelPartitioning. Partitioning phase of Customizable Route Planning (CRP) By Delling et al. read section 5.1 Metric-Independent Preprocessing (Partitioning) :  https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

 run L-level mutltilevel partitioning using inertial flow algorithm with U1 , . . . , UL maximum cell sizes.
pertama jalankan algoritma intertial flow pada graf G dengan parameter U_{L} untuk mendapatkan cells level L.
cells di level bawahnya didapatkan dengan menjalankan algoritma inertial flow pada individual cells of the level immediately above.

time complexity:
for each level l, time complexity recursiveBisection.Partition() in each cell is O( U_{l+1} * sqrt(U_{l+1}) * log_{1/(1-b)} (U_{l+1}) ). dengan U_{L+1}=n
*/ // nolint: gofmt
func (mp *MultilevelPartitioner) RunMultilevelPartitioning() {
	// start from highest level
	vIds := mp.graph.GetVerticeIds()
	progress := util.NewProgress(mp.graph.NumberOfVertices())
	mp.logger.Sugar().Infof("partitioning level %d with max cell size %d", mp.l, mp.u[mp.l-1])
	fmt.Printf("Level %d progress: 0%%...", mp.l)
	n := len(vIds)
	permutedvIds := make([]da.Index, n)
	gv := make([]da.Index, n)
	copy(permutedvIds, vIds)
	copy(gv, vIds)
	if n > mp.u[mp.l-1] {

		rb := NewRecursiveBisection(mp.graph, mp.u[mp.l-1], mp.logger,
			mp.inertialFlowIterations, true)
		rb.progress = progress
		rb.Partition(vIds)
		fp := rb.GetFinalPartition()
		cp := mp.groupEachPartition(fp)
		mp.cellVertices[mp.l-1] = append(mp.cellVertices[mp.l-1], cp...)
	} else {
		mp.cellVertices[mp.l-1] = [][]da.Index{vIds}
		progress.Add(len(vIds))
	}
	progress.Finish()
	mp.logger.Sugar().Infof("level %d done, total cells: %d", mp.l, len(mp.cellVertices[mp.l-1]))

	// partition each cell in previous level
	for level := mp.l - 2; level >= 0; level-- {
		progress = util.NewProgress(mp.graph.NumberOfVertices())
		mp.logger.Sugar().Infof("partitioning level %d with max cell size %d", level+1, mp.u[level])
		fmt.Printf("Level %d progress: 0%%...", level+1)

		cellInChan := make(chan []da.Index, CellInOutChanSize)
		cellOutchan := make(chan [][]da.Index, CellInOutChanSize)
		wg := sync.WaitGroup{}
		computeRecursiveBisection := func() {
			for cellvIds := range cellInChan {
				rb := NewRecursiveBisection(mp.graph, mp.u[level], mp.logger,
					mp.inertialFlowIterations, true)
				rb.progress = progress
				rb.Partition(cellvIds)
				fp := rb.GetFinalPartition()
				partitions := mp.groupEachPartition(fp)
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

		progress.Finish()
		mp.logger.Sugar().Infof("level %d total cells: %d", level+1, len(mp.cellVertices[level]))
	}
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
	return cells // cellId -> vertices Ids
}

// MapToEdgeBasedGraph use heuristic to assign cellId of each edge-based graph nodes from node-based graph partition
// inspired by osrm partitioner https://github.com/Project-OSRM/osrm-backend/issues/3205#issuecomment-275169844
// read TheMarex comment on that osrm github issue. in here, we use step 1-6 of osrm-partition, using method 1 of step 6 with little modification...
// the reason not implement method 2 of step 6, is because its too complex, much easier to implement method 1.
// ebg is the edge-based graph (or edge-expanded graph whatever). read: https://github.com/Project-OSRM/osrm-backend/wiki/Graph-representation
// ebgMapping map from edge-based graph (ebg) vertex id to packed id (tail, head) vertex of that ebg vertex
func (mp *MultilevelPartitioner) MapToEdgeBasedGraph(ebg *da.Graph, ebgMapping []uint64) {
	ebgCellVertices := make([][][]da.Index, mp.l)
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	nv := mp.graph.NumberOfVertices()
	ne := ebg.NumberOfVertices()

	n := da.Index(len(ebgMapping))
	for l := 0; l < mp.l; l++ {
		u := mp.u[l]
		ebgCellMap := make(map[da.Index]int, ne)
		ebgCellVertices[l] = make([][]da.Index, len(mp.cellVertices[l]))
		cellVerticesMap := make(map[da.Index]int, nv)
		for cellId, vertexIds := range mp.cellVertices[l] {
			for _, vertexId := range vertexIds {
				cellVerticesMap[vertexId] = cellId
			}
		}

		boundaryEbgVertices := make([]da.Index, 0, 100)
		// step 4-6 in that issue comment. step 6 use method 1
		for ebgvId := da.Index(0); ebgvId < n; ebgvId++ {
			packedId := ebgMapping[ebgvId]
			tail, head := da.Index(packedId&0xFFFFFFFF), da.Index(packedId>>32)
			tc := cellVerticesMap[tail]
			hc := cellVerticesMap[head]
			if tc == hc { // tail u and head v of edge (u,v) in the same cell...
				ebgCellVertices[l][tc] = append(ebgCellVertices[l][tc], ebgvId)
				ebgCellMap[ebgvId] = tc
			} else {
				// tail u and head v of edge (u,v) not in the same cell... a.k.a cut edges of the node-based graph
				boundaryEbgVertices = append(boundaryEbgVertices, ebgvId)
			}
		}

		for _, ebgvId := range boundaryEbgVertices {
			// step 6: https://github.com/Project-OSRM/osrm-backend/issues/3205#issuecomment-275169844
			// process cut edges of node-based graph
			packedId := ebgMapping[ebgvId]
			tail, head := da.Index(packedId&0xFFFFFFFF), da.Index(packedId>>32)
			tc := cellVerticesMap[tail]
			hc := cellVerticesMap[head]

			// tail cell != head cell

			// try assign tail cell
			// count the number of cut-edges introduced if we assign vertex ebgvId to tail cell
			numCutEdgesTailCell := 0
			ebg.ForOutEdgesOf(ebgvId, func(_, v, _ da.Index) {
				vc, ok := ebgCellMap[v]
				if ok && vc != tc {
					numCutEdgesTailCell++
				}
			})

			ebg.ForInEdgesOf(ebgvId, func(_, v, _ da.Index) {
				vc, ok := ebgCellMap[v]
				if ok && vc != tc {
					numCutEdgesTailCell++
				}
			})

			// try assign head cell
			numCutEdgesHeadCell := 0
			ebg.ForOutEdgesOf(ebgvId, func(_, v, _ da.Index) {
				vc, ok := ebgCellMap[v]
				if ok && vc != hc {
					numCutEdgesHeadCell++
				}
			})

			ebg.ForInEdgesOf(ebgvId, func(_, v, _ da.Index) {
				vc, ok := ebgCellMap[v]
				if ok && vc != hc {
					numCutEdgesHeadCell++
				}
			})

			if numCutEdgesTailCell < numCutEdgesHeadCell {
				// modification of method 1 in step 6 in that osrm-partition issue: The idea is to place edge-based graph node into cell in such a way that the number of cut edges is minimized.
				// ini heuristik paling bagus... number of cut edges, boundary/overlay vertices, dan shortcut edges hampir sama kaya directly partition the edge-based graph..
				ebgCellVertices[l][tc] = append(ebgCellVertices[l][tc], ebgvId)
			} else if numCutEdgesHeadCell < numCutEdgesTailCell {
				ebgCellVertices[l][hc] = append(ebgCellVertices[l][hc], ebgvId)
			} else {
				// use method 1 of step 6 in that comment issue:  https://github.com/Project-OSRM/osrm-backend/issues/3205#issuecomment-275169844
				// randomly pick cell
				// if r < 0.5, pick tail cell
				// else, pick head cell
				// jujur masih gak paham kenapa method 2 dari step 6 in that commment issue bisa lebih minimize number of border/boundary vertices di edge-based graph....
				r := rd.Float64()
				tcSize := len(ebgCellVertices[l][tc])
				hcSize := len(ebgCellVertices[l][hc])
				if util.Lt(r, 0.5) {
					if tcSize < u {
						ebgCellVertices[l][tc] = append(ebgCellVertices[l][tc], ebgvId)
					} else {
						ebgCellVertices[l][hc] = append(ebgCellVertices[l][hc], ebgvId)
					}
				} else {
					if hcSize < u {
						ebgCellVertices[l][hc] = append(ebgCellVertices[l][hc], ebgvId)
					} else {
						ebgCellVertices[l][tc] = append(ebgCellVertices[l][tc], ebgvId)
					}
				}
			}

		}
	}

	mp.cellVertices = ebgCellVertices // set new edge-based graph cell vertices. edge-based graph partition.
	mp.graph = ebg
}
