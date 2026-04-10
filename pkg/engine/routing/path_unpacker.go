package routing

import (
	"encoding/binary"
	"time"

	"github.com/dgraph-io/ristretto/v2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func NewPUCacheKey(start, target da.Index, level uint8) []byte {
	buf := make([]byte, 9)
	binary.LittleEndian.PutUint32(buf[:4], uint32(start))
	binary.LittleEndian.PutUint32(buf[4:8], uint32(target))
	buf[8] = level
	return buf
}

type PathUnpacker struct {
	engine *CRPRoutingEngine

	metrics *metrics.Metric

	puCache *ristretto.Cache[[]byte, []da.Index]

	useCache             bool
	oneToMany            bool
	runtime              int64
	forAlternativeRoutes bool
}

func NewPathUnpacker(engine *CRPRoutingEngine, metrics *metrics.Metric,
	puCache *ristretto.Cache[[]byte, []da.Index], useCache, oneToMany bool) *PathUnpacker {
	return &PathUnpacker{
		engine:  engine,
		metrics: metrics,

		puCache:   puCache,
		useCache:  useCache,
		oneToMany: oneToMany,
		runtime:   0,
	}
}

/*
[1] Path Unpacking phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.

unpackPath. unpack a level-i shortcut (v, w) by running Dijkstra  between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut.
jika level i-1 >= 1, kita menggunakan shortcut edges (dari subcells dari level-i cell containing the shortcut) di overlay graph level i-1
jika level i-1 = 0, kita menggunakan base edges yang terletak pada level-i cell containing the shortcut

this path unpacking use dijkstra

time complexity:
let n_p,m_p,n_op,and \hat{m_p} denote the maximum number of nodes, edges, overlay vertices (include overlay vertices in its all direct subcells/subcells in level-1), and shortcuts within any partition
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
lowest level cell: O(m_p*log(m_p)), in unpackInLowestLevelCell(), priority queue (4-ary heap) contains at most m_p (turn-based graph), decrease-key and insert at most O(m_p) operations, extract-min at-most O(m_p) operations
cell level > 1 : O((n_op + \hat{m_p})*log(n_op)), decrease-key and insert at most O(\hat{m_p}) operations, extract-min is at most O(n_op) operations
let q = number of shorcut edges in packedPath
let L = highest level of multilevel partition
time complexity of unpackPath: O(q * ( L *  (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p) ) )
*/
func (pu *PathUnpacker) unpackPath(packedPath []da.VertexEdgePair, sCellNumber, tCellNumber da.Pv) ([]da.Index, map[uint64]uint8) {
	unpackedEdgePath := make([]da.Index, 0, len(packedPath))

	now := time.Now()

	// todo: sebelumnya pakai worker pools di worker_pool.go, di benchmark ada additional 40000 B/op, bikin worker pool yang allocate less space ?

	shortcutPathSet := make(map[uint64]uint8)

	for i := 0; i < len(packedPath); {
		cur := packedPath[i]
		if !isBitOn(cur.GetEdge(), UNPACK_OVERLAY_OFFSET) {
			// original vertex (non-overlay vertex)

			unpackedEdgePath = append(unpackedEdgePath, cur.GetEdge())

			i++
		} else {
			// overlay vertex
			entryOverlayId := offBit(cur.GetEdge(), UNPACK_OVERLAY_OFFSET)

			entryVertex := pu.engine.overlayGraph.GetVertex(entryOverlayId)
			entryCellNumber := entryVertex.GetCellNumber()
			var queryLevel uint8

			if !pu.oneToMany {
				queryLevel = pu.engine.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)

			} else {
				queryLevel = cur.GetQueryLevel()
			}

			exitOverlayId := offBit(packedPath[i+1].GetEdge(), UNPACK_OVERLAY_OFFSET)
			exitVertex := pu.engine.overlayGraph.GetVertex(exitOverlayId)

			enOriVId := entryVertex.GetOriginalVertex()
			exOriVId := exitVertex.GetOriginalVertex()
			shortcutPathSet[util.Bitpack(uint32(enOriVId), uint32(exOriVId))] = queryLevel

			shortcutEdgeIdsPath := pu.unpackInLevelCell(entryOverlayId, exitOverlayId, queryLevel)
			unpackedEdgePath = append(unpackedEdgePath, shortcutEdgeIdsPath...)
			i += 2
		}
	}

	unpackedEdgePath = removeDuplicates(unpackedEdgePath)

	dur := time.Since(now).Milliseconds()
	pu.runtime = dur
	return unpackedEdgePath, shortcutPathSet
}

func (pu *PathUnpacker) unpackInLevelCell(sourceOverlayId da.Index,
	targetOverlayId da.Index,
	level uint8,
) []da.Index {

	if level == 1 {
		sourceOverlayVertex := pu.engine.overlayGraph.GetVertex(sourceOverlayId)
		sourceEntryId := sourceOverlayVertex.GetOriginalEdge()
		targetOverlayVertex := pu.engine.overlayGraph.GetVertex(targetOverlayId)
		neighborOfTarget := targetOverlayVertex.GetNeighborOverlayVertex()
		neighborOverlayVertex := pu.engine.overlayGraph.GetVertex(neighborOfTarget)
		targetEntryId := neighborOverlayVertex.GetOriginalEdge()

		edgePath := pu.unpackInLowestLevelCell(sourceEntryId, targetEntryId,
			sourceOverlayId, targetOverlayId)
		return edgePath
	}

	if pu.useCache {
		if overlayPath, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, level)); ok {

			edgePath := make([]da.Index, 0, UNPACKER_EDGE_PATH_SIZE)
			for i := 0; i < len(overlayPath); i += 2 {
				downLevelEdgePath := pu.unpackInLevelCell(overlayPath[i], overlayPath[i+1], level-1)
				edgePath = append(edgePath, downLevelEdgePath...)
			}

			return edgePath
		}
	}

	sVertex := pu.engine.overlayGraph.GetVertex(sourceOverlayId)
	sourceCellNumber := sVertex.GetCellNumber()

	pq := pu.engine.pufOverlayHeapPool.Get().(*da.QueryHeap[da.Index])

	pq.Clear()

	done := func() {
		pu.engine.pufOverlayHeapPool.Put(pq)

	}

	truncatedSourceCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, level)

	tVertex := pu.engine.overlayGraph.GetVertex(targetOverlayId)
	targetCellNumber := tVertex.GetCellNumber()
	truncatedTargetCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(targetCellNumber, level)

	util.AssertPanic(truncatedSourceCellNumber == truncatedTargetCellNumber, "cell number/id dari sourceOverlay vertex dan targetOverlay vertex haruslah sama")

	sVertexInfo := da.NewVertexInfo(0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	pq.Insert(sourceOverlayId, 0, sVertexInfo, sourceOverlayId)

	labelled := func(pq *da.QueryHeap[da.Index], v da.Index) bool {

		ok := util.Lt(pq.GetPriority(v), pkg.INF_WEIGHT)
		return ok
	}

	for pq.Size() > 0 {

		u := pq.ExtractMin()

		uOverlayId := u.GetItem()
		pq.Scan(uOverlayId)

		if uOverlayId == targetOverlayId {
			break
		}

		// traverse all out neighbor of u in level l-1 in the same cell as u
		pu.engine.overlayGraph.ForOutNeighborsOf(uOverlayId, int(level-1), func(vOverlayId da.Index, wOffset da.Index) {

			shortcutOutEdgeWeight := pu.metrics.GetShortcutWeight(wOffset)
			vOverlayVertex := pu.engine.overlayGraph.GetVertex(vOverlayId)

			newTravelTime := pq.GetPriority(uOverlayId) + shortcutOutEdgeWeight

			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vAlreadyLabelled := labelled(pq, vOverlayId)
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(vOverlayId))) {
				// relax shortcut edge

				pq.Scan(vOverlayId) // langsung scan exit overlay vertex v
				uOverlayVertex := pu.engine.overlayGraph.GetVertex(uOverlayId)
				originalUId := uOverlayVertex.GetOriginalVertex()

				if vOverlayId == targetOverlayId {
					// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
					// if v is the target overlay vertex, insert/decrease its key  pq

					if !vAlreadyLabelled {
						wVertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(originalUId,
							uOverlayId, true))
						pq.Insert(vOverlayId, newTravelTime, wVertexInfo, uOverlayId)
					} else {
						wNewPar := da.NewVertexEdgePair(originalUId,
							uOverlayId, true)
						pq.DecreaseKey(vOverlayId, newTravelTime, newTravelTime, wNewPar)
					}

				} else {
					pq.Set(vOverlayId, da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(originalUId,
						uOverlayId, true)), vOverlayId)
				}

				// visit next cell neighbor
				wNeighborId := vOverlayVertex.GetNeighborOverlayVertex()
				wNeigborVertex := pu.engine.overlayGraph.GetVertex(wNeighborId)

				wCellNumber := wNeigborVertex.GetCellNumber()
				truncatedWCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(wCellNumber, uint8(level))
				if truncatedWCellNumber != truncatedSourceCellNumber {
					// if w is not in the same cell as sourceOverlayId in level l, dont visit w
					return
				}

				// get out edge that point to wEntryVertex from vOverlayId
				newTravelTime += pu.engine.GetWeight(vOverlayVertex.GetOriginalEdge(), true)
				// relax edge
				wAlreadyLabelled := labelled(pq, wNeighborId)
				if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(wNeighborId))) {
					if !wAlreadyLabelled {
						wVertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
							vOverlayId, true))
						pq.Insert(wNeighborId, newTravelTime, wVertexInfo, wNeighborId)
					} else {
						wNewPar := da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
							vOverlayId, true)
						pq.DecreaseKey(wNeighborId, newTravelTime, newTravelTime, wNewPar)
					}
				}

			}

		})

	}

	overlayPath := make([]da.Index, 0, UNPACKER_OVERLAY_PATH_SIZE)

	overlayPath = append(overlayPath, targetOverlayId)

	curOverlayId := pq.Get(targetOverlayId).GetParent().GetEdge()
	for curOverlayId != da.INVALID_EDGE_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = pq.Get(curOverlayId).GetParent().GetEdge()
	}

	util.ReverseG(overlayPath)

	pq.Clear()

	util.AssertPanic(len(overlayPath)%2 == 0, "harusnya len(overlayPath) genap")

	if pu.useCache {
		pu.puCache.Set(NewPUCacheKey(sourceOverlayId, targetOverlayId, level), overlayPath, 1)
	}

	done()

	edgePath := make([]da.Index, 0, UNPACKER_EDGE_PATH_SIZE)
	for i := 0; i < len(overlayPath); i += 2 {
		curV := overlayPath[i]
		nextV := overlayPath[i+1]

		downLevelEdgePath := pu.unpackInLevelCell(curV, nextV, level-1)
		edgePath = append(edgePath, downLevelEdgePath...)
	}

	return edgePath
}

func (pu *PathUnpacker) unpackInLowestLevelCell(sourceEntryId, targetEntryId da.Index,
	sourceOverlayId, targetOverlayId da.Index) []da.Index {

	if pu.useCache {
		if edgeIds, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1)); ok {
			// fetch from cache

			return edgeIds
		}
	}

	// sourceEntryId inEdge that point to source vertex
	pq := pu.engine.pufBaseHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey])
	pq.Clear()

	done := func() {
		pu.engine.pufBaseHeapPool.Put(pq)
	}
	defer done()

	// sourceEntryId: id buat inEdge u->s
	// targetEntryId: id buat inEdge t->v

	// get source vertex
	sourceVertex := pu.engine.graph.GetVertex(pu.engine.graph.GetHeadFromInEdge(sourceEntryId))

	s := sourceVertex.GetID()

	// s and t in same cell in level 1 and both are overlay vertices
	// s is entry vertex, t is exit vertex of this cell

	_, sOutEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(sourceEntryId)

	sourceCellNumber := pu.engine.graph.GetCellNumber(s)

	offSourceEntryId := pu.engine.offsetForward(s, sourceEntryId, sourceCellNumber, sourceCellNumber)

	sQueryKey := da.NewCRPQueryKeyWithOutInEdgeId(s, offSourceEntryId, sOutEdge)
	sInfo := da.NewVertexInfo(0, da.NewVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false))

	pq.Insert(offSourceEntryId, 0, sInfo, sQueryKey)

	offTargetEntryId := da.INVALID_EDGE_ID

	for pq.Size() > 0 {

		queryKey := pq.ExtractMin()

		uItem := queryKey.GetItem()
		uId := uItem.GetNode()
		uEntryId := uItem.GetEntryExitPoint()
		uOutEdgeId := uItem.GetOutInEdgeId()

		pq.Scan(uEntryId)

		adjuEntryId := pu.engine.adjustForward(uId, uEntryId)

		if adjuEntryId == targetEntryId {
			offTargetEntryId = uEntryId
			break
		}

		// relax all out edges of u
		pu.engine.graph.ForOutEdgesOf(uId, pu.engine.graph.GetEntryOrder(uId, adjuEntryId), func(eId, head da.Index, weight, length float64, exitPoint, entryPoint da.Index, turnType pkg.TurnType,
			hwType pkg.OsmHighwayType) {
			vId := head

			vEntryId := pu.engine.graph.GetEntryOffset(vId) + entryPoint
			edgeWeight := pu.engine.GetWeight(eId, true)

			newTravelTime := pq.GetPriority(uEntryId) + edgeWeight + pu.metrics.GetTurnCost(turnType)

			if pu.engine.graph.GetCellNumber(vId) != sourceCellNumber && vEntryId != targetEntryId {
				// do not cross cell boundary
				return
			}

			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			offVEntryId := pu.engine.offsetForward(vId, vEntryId, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			// relax edge
			vAlreadyLabelled := util.Lt(pq.GetPriority(offVEntryId), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(offVEntryId))) {

				if !vAlreadyLabelled {
					queryKey := da.NewCRPQueryKeyWithOutInEdgeId(vId, offVEntryId, eId)
					vInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false))

					pq.Insert(offVEntryId, newTravelTime, vInfo, queryKey)
				} else {
					newPar := da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false)
					pq.DecreaseKey(offVEntryId, newTravelTime, newTravelTime, newPar)
				}
			}

		})

	}

	edgeIdPath := make([]da.Index, 0, UNPACKER_EDGE_PATH_SIZE)

	_, midOutEdgeId := pu.engine.graph.GetHeadOfInedgeWithOutEdge(targetEntryId)
	if util.Gt(pu.engine.GetWeight(midOutEdgeId, true), 0) {
		edgeIdPath = append(edgeIdPath, midOutEdgeId)
	}

	uId := offTargetEntryId
	for pq.Get(uId).GetParent().GetEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = sourceEntryId, include sp edges didalam current cell & sp edge entry cell ini
		prevOutEdgeId := pq.Get(uId).GetParent().GetOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)

		pEId := pq.Get(uId).GetParent().GetEdge()

		uId = pEId
	}

	util.ReverseG(edgeIdPath)

	pq.Clear()

	if pu.useCache {
		// https://github.com/dgraph-io/ristretto is thread-safe
		pu.puCache.Set(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1), edgeIdPath, 1)
	}

	return edgeIdPath
}

func (pu *PathUnpacker) GetStats() int64 {
	return pu.runtime
}

func (re *CRPRoutingEngine) GetEdgePath(edgeIdPath []da.Index) (*da.Coordinates, float64) {

	totalDistance := 0.0

	capacity := 0

	for i := 0; i < len(edgeIdPath); i++ {
		eId := edgeIdPath[i]
		totalDistance += re.graph.GetOutEdgeLength(eId)
		capacity += re.graph.GetEdgeGeometryLength(eId)
	}

	path := da.NewCoordinatesWithCap(capacity)

	for i := 0; i < len(edgeIdPath); i++ {
		eId := edgeIdPath[i]
		re.graph.AppendPathWithEdgeGeometry(path, eId)
	}

	return path, totalDistance
}

func (re *CRPRoutingEngine) GetPathDistance(edgeIdPath []da.Index) float64 {

	totalDistance := 0.0

	for i := 0; i < len(edgeIdPath); i++ {
		eId := edgeIdPath[i]
		totalDistance += re.graph.GetOutEdgeLength(eId)
	}

	return totalDistance
}

func (pu *PathUnpacker) setForAlternativeRoutes() {
	pu.forAlternativeRoutes = true
}
