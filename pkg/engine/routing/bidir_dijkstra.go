package routing

import (
	"math"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type BidirectionalDijkstra struct {
	engine *CRPRoutingEngine

	forwardPq          *da.QueryHeap[da.CRPQueryKey]
	backwardPq         *da.QueryHeap[da.CRPQueryKey]
	shortestTravelTime float64
	stallingEntry      []float64
	stallingExit       []float64
	forwardMid         da.VertexEdgePair
	backwardMid        da.VertexEdgePair
	viaVertices        []da.ViaVertex
	runtime            int64
	sForwardId         da.Index
	tBackwardId        da.Index
	sCellNumber        da.Pv
	tCellNumber        da.Pv

	upperBound float64
	clonePq    bool
	pq         *da.QueryHeap[da.CRPQueryKey]

	numSettledNodes  int
	useReversedEdges bool
}

func NewBidirectionalDijkstra(engine *CRPRoutingEngine, upperBound float64) BidirectionalDijkstra {
	dj := BidirectionalDijkstra{
		engine:      engine,
		forwardMid:  da.NewVertexEdgePair(0, 0, false),
		backwardMid: da.NewVertexEdgePair(0, 0, true),
		viaVertices: make([]da.ViaVertex, 0),

		numSettledNodes:    0,
		shortestTravelTime: 0,
		runtime:            0,
		upperBound:         upperBound,
	}

	dj.Preallocate()
	return dj
}

/*
https://kam.mff.cuni.cz/~spring/media/papers/5/bidirectional_dijkstra.pdf

bidirectional dijkstra, support turn costs
*/
func (bs *BidirectionalDijkstra) ShortestPathSearch(asId, atId da.Index) (float64, float64, []da.Coordinate,
	[]da.OutEdge, bool) {

	now := time.Now()

	s := bs.engine.graph.GetOutEdge(asId).GetHead()
	t := bs.engine.graph.GetInEdge(atId).GetTail()

	if s == t {
		return 0, 0, []da.Coordinate{}, []da.OutEdge{}, true
	}

	// strategy: use outEdges for forward search, use inEdges for backward search
	// for iterating outEdges, we need entryOffset. for iterating inEdges, we need exitOffset.

	sForwardId := bs.engine.graph.GetEntryOffset(s) + da.Index(bs.engine.graph.GetOutEdge(asId).GetEntryPoint())
	tBackwardId := bs.engine.graph.GetExitOffset(t) + da.Index(bs.engine.graph.GetInEdge(atId).GetExitPoint())
	bs.sForwardId = sForwardId
	bs.tBackwardId = tBackwardId

	bs.shortestTravelTime = 2 * pkg.INF_WEIGHT

	sVertexInfo := da.NewVertexInfo(0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, sForwardId, false))
	tVertexInfo := da.NewVertexInfo(0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, tBackwardId, true))
	sQueryKey := da.NewCRPQueryKey(s, sForwardId, false)
	tQueryKey := da.NewCRPQueryKey(t, tBackwardId, false)
	bs.forwardPq.Insert(sForwardId, 0, sVertexInfo, sQueryKey)
	bs.backwardPq.Insert(tBackwardId, 0, tVertexInfo, tQueryKey)

	close := func(id da.Index, queryHeap *da.QueryHeap[da.CRPQueryKey]) {
		queryHeap.Scan(id)
	}

	for bs.forwardPq.Size() > 0 && bs.backwardPq.Size() > 0 {
		minForward := bs.forwardPq.GetMinrank()
		minBackward := bs.backwardPq.GetMinrank()
		if da.Ge(minForward+minBackward, (bs.shortestTravelTime)*(bs.upperBound)) {
			break
		}

		queryKey := bs.forwardPq.ExtractMin()
		uItem := queryKey.GetItem()

		close(uItem.GetEntryExitPoint(), bs.forwardPq)
		bs.forwardGraphSearch(uItem, s, t)

		queryKey = bs.backwardPq.ExtractMin()
		uItem = queryKey.GetItem()

		close(uItem.GetEntryExitPoint(), bs.backwardPq)
		bs.backwardGraphSearch(uItem, s, t)

	}

	if da.Eq(bs.shortestTravelTime, 2*pkg.INF_WEIGHT) {
		return pkg.INF_WEIGHT, 2 * pkg.INF_WEIGHT, []da.Coordinate{}, []da.OutEdge{}, false
	}

	finalEdgePath := make([]da.OutEdge, 0)
	finalPath := make([]da.Coordinate, 0)

	mid := bs.forwardMid

	totalDistance := 0.0
	_, midOutEdge := bs.engine.graph.GetHeadOfInedgeWithOutEdge(mid.GetEdge())
	mid.SetEdge(midOutEdge.GetEdgeId())
	tail := bs.engine.graph.GetTailFromOutEdge(midOutEdge.GetEdgeId())

	if tail != midOutEdge.GetHead() {
		geom := bs.engine.graph.GetEdgeGeometry(midOutEdge.GetEdgeId())
		revGeom := util.ReverseG(geom)
		finalPath = append(finalPath, revGeom...)
		finalEdgePath = append(finalEdgePath, *midOutEdge)
		totalDistance += midOutEdge.GetLength()
	}

	curInfo := bs.forwardPq.Get(bs.forwardMid.GetEdge())

	for curInfo.GetParent().GetEdge() != sForwardId {
		parent := curInfo.GetParent()
		parentEdge := parent.GetEdge()
		parentCopy := parent

		// jadiin outEdge semua
		inEdge := bs.engine.graph.GetInEdge(parentCopy.GetEdge())
		_, outEdge := bs.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
		parentCopy.SetEdge(outEdge.GetEdgeId())

		finalEdgePath = append(finalEdgePath, *outEdge)

		geom := bs.engine.graph.GetEdgeGeometry(outEdge.GetEdgeId())
		revGeom := util.ReverseG(geom)
		finalPath = append(finalPath, revGeom...)
		totalDistance += outEdge.GetLength()
		curInfo = bs.forwardPq.Get(parentEdge)
	}

	finalPath = util.ReverseG(finalPath)
	finalEdgePath = util.ReverseG(finalEdgePath)

	mid = bs.backwardMid
	curInfo = bs.backwardPq.Get(bs.backwardMid.GetEdge())

	midOutEdge = bs.engine.graph.GetOutEdge(mid.GetEdge())
	tail = bs.engine.graph.GetTailFromOutEdge(midOutEdge.GetEdgeId())
	if tail != midOutEdge.GetHead() {
		geom := bs.engine.graph.GetEdgeGeometry(midOutEdge.GetEdgeId())
		finalPath = append(finalPath, geom...)
		finalEdgePath = append(finalEdgePath, *midOutEdge)
		totalDistance += midOutEdge.GetLength()
	}

	for curInfo.GetParent().GetEdge() != tBackwardId {
		parent := curInfo.GetParent()
		parentEdge := parent.GetEdge()
		parentCopy := parent

		outEdge := bs.engine.graph.GetOutEdge(parentCopy.GetEdge())
		finalEdgePath = append(finalEdgePath, *outEdge)

		geom := bs.engine.graph.GetEdgeGeometry(outEdge.GetEdgeId())
		finalPath = append(finalPath, geom...)
		totalDistance += outEdge.GetLength()

		curInfo = bs.backwardPq.Get(parentEdge)
	}

	dur := time.Since(now).Milliseconds()
	bs.runtime = dur

	return bs.shortestTravelTime, totalDistance, finalPath, finalEdgePath, true
}

func (bs *BidirectionalDijkstra) forwardGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {

	uId := uItem.GetNode()
	uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

	uEntryPoint := uEntryId - bs.engine.graph.GetEntryOffset(uId)
	uEntryIdTravelTime := bs.forwardPq.GetPriority(uEntryId)

	// stalling
	uInDeg := bs.engine.graph.GetInDegree(uId)
	otherUEntryId := bs.engine.graph.GetEntryOffset(uId)

	for j := da.Index(0); j < uInDeg; j++ {

		stallingOffset := uInDeg*uEntryPoint + j
		bui := math.Max(0, uEntryIdTravelTime+
			bs.engine.metrics.GetEntryStallingTableCost(uId, stallingOffset))

		if val := bs.stallingEntry[otherUEntryId]; da.Eq(val, pkg.INF_WEIGHT) {
			bs.stallingEntry[otherUEntryId] = bui
		} else {
			bs.stallingEntry[otherUEntryId] = math.Min(bs.stallingEntry[otherUEntryId], bui)
		}
		otherUEntryId++
	}

	// traverse outEdges of u
	bs.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(outArc *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {
		vId := outArc.GetHead()

		edgeWeight := bs.engine.metrics.GetWeight(outArc)

		turnCost := bs.engine.metrics.GetTurnCost(turnType)
		if uId == source {
			turnCost = 0
		}

		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newTravelTime := uEntryIdTravelTime + edgeWeight + turnCost

		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}
		vEntryId := bs.engine.graph.GetEntryOffset(vId) + da.Index(outArc.GetEntryPoint())

		// if query level of v is 0, then v is in the same cell as s or t in the lowest level
		// then, we just do edge relaxation as usual in turn-aware dijkstra

		// relax edge
		oldVEntryIdTravelTime := bs.forwardPq.GetPriority(vEntryId)
		vAlreadyLabelled := da.Lt(oldVEntryIdTravelTime, pkg.INF_WEIGHT)
		if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, oldVEntryIdTravelTime)) {
			if bvi := bs.stallingEntry[vEntryId]; da.Lt(bvi, pkg.INF_WEIGHT) && da.Gt(newTravelTime, bvi) {
				// stalled
				return
			}

			if vAlreadyLabelled {
				// newTravelTime is better, update the forwardInfo8
				// is key already in the priority queue, decrease its key

				newPar := da.NewVertexEdgePair(uId, uEntryId, false)
				bs.forwardPq.DecreaseKey(vEntryId, newTravelTime, newTravelTime, newPar)
			} else if !vAlreadyLabelled {

				vertexInfo := da.NewVertexInfo(newTravelTime,
					da.NewVertexEdgePair(uId, uEntryId, false))
				queryKey := da.NewCRPQueryKey(vId, vEntryId, false)
				// is key not in the priority queue, insert it
				bs.forwardPq.Insert(vEntryId, newTravelTime, vertexInfo, queryKey)
			}
		}

		// check wether we already scanned an exit point of vId

		exitOffset := bs.engine.graph.GetExitOffset(vId)

		vExitId := exitOffset

		newVEntryIdTravelTime := bs.forwardPq.GetPriority(vEntryId)
		// traverse outEdges of v
		bs.engine.graph.ForOutEdgesOf(vId, da.Index(outArc.GetEntryPoint()), func(e2 *da.OutEdge,
			exitPoint da.Index, turnType2 pkg.TurnType) {

			//  check if forward and backward search already scanned entry and exit point of v. if so, check whether we can improve the shortest path
			scannedByBackwardSearch := bs.backwardPq.IsScanned(vExitId)
			labelledByBackwardSearch := bs.backwardPq.IsLabelled(vExitId)

			vExitIdTravelTime := bs.backwardPq.GetPriority(vExitId)
			if scannedByBackwardSearch && da.Lt(newVEntryIdTravelTime+bs.engine.metrics.GetTurnCost(turnType2)+
				vExitIdTravelTime, bs.shortestTravelTime) {

				bs.shortestTravelTime = newVEntryIdTravelTime + bs.engine.metrics.GetTurnCost(turnType2) +
					vExitIdTravelTime

				bs.forwardMid = da.NewVertexEdgePair(vId, vEntryId, false)
				bs.backwardMid = da.NewVertexEdgePair(vId, vExitId, true)
				bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(vId, vEntryId, vExitId, vId, false))
			} else if labelledByBackwardSearch {
				bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(vId, vEntryId, vExitId, vId, false))
			}
			vExitId++
		})

	})
}

func (bs *BidirectionalDijkstra) backwardGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
	// search backward on graph level 1
	// basically same as forward search, but using inEdges and exitPoint instead of outEdges and entryPoint

	uId := uItem.GetNode()
	uExitId := uItem.GetEntryExitPoint() // index of outEdge that have endpoint from vertex uId
	uExitIdTravelTime := bs.backwardPq.GetPriority(uExitId)

	uExitPoint := uExitId - bs.engine.graph.GetExitOffset(uId)

	// stalling
	uOutDeg := bs.engine.graph.GetOutDegree(uId)
	otherUExitId := bs.engine.graph.GetExitOffset(uId)

	for j := da.Index(0); j < uOutDeg; j++ {

		stallingOffset := uOutDeg*uExitPoint + j
		bui := math.Max(0, uExitIdTravelTime+
			bs.engine.metrics.GetExitStallingTableCost(uId, stallingOffset))

		if val := bs.stallingExit[otherUExitId]; da.Eq(val, pkg.INF_WEIGHT) {
			bs.stallingExit[otherUExitId] = bui
		} else {
			bs.stallingExit[otherUExitId] = math.Min(bs.stallingExit[otherUExitId], bui)
		}
		otherUExitId++
	}

	bs.engine.graph.ForInEdgesOf(uId, uExitPoint, func(inArc *da.InEdge, entryPoint da.Index, turnType pkg.TurnType) {
		vId := inArc.GetTail()

		edgeWeight := bs.engine.metrics.GetWeight(inArc)

		turnCost := bs.engine.metrics.GetTurnCost(turnType)

		if uId == target {
			turnCost = 0
		}

		newTravelTime := uExitIdTravelTime + edgeWeight + turnCost
		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}

		vExitId := bs.engine.graph.GetExitOffset(vId) + da.Index(inArc.GetExitPoint())

		// relax edge
		oldVExitIdTravelTime := bs.backwardPq.GetPriority(vExitId)
		vAlreadyLabelled := da.Lt(oldVExitIdTravelTime, pkg.INF_WEIGHT)
		if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, oldVExitIdTravelTime)) {

			if bvi := bs.stallingExit[vExitId]; da.Lt(bvi, pkg.INF_WEIGHT) && da.Gt(newTravelTime, bvi) {
				// stalled
				return
			}

			if vAlreadyLabelled {
				newPar := da.NewVertexEdgePair(uId, uExitId, true)
				bs.backwardPq.DecreaseKey(vExitId, newTravelTime, newTravelTime, newPar)
			} else {
				vertexInfo := da.NewVertexInfo(newTravelTime,
					da.NewVertexEdgePair(uId, uExitId, false))
				queryKey := da.NewCRPQueryKey(vId, vExitId, false)
				bs.backwardPq.Insert(vExitId, newTravelTime, vertexInfo, queryKey)
			}
		}

		// check wether we already Labelled an entry point
		entryOffset := bs.engine.graph.GetEntryOffset(vId)

		vEntryId := entryOffset

		newVExitIdTravelTime := bs.backwardPq.GetPriority(vExitId)
		bs.engine.graph.ForInEdgesOf(vId, da.Index(inArc.GetExitPoint()), func(inArc2 *da.InEdge,
			entryPoint2 da.Index, turnType2 pkg.TurnType) {
			scannedByForwardSearch := bs.forwardPq.IsScanned(vEntryId)
			labelledByForwardSearch := bs.forwardPq.IsLabelled(vEntryId)

			vEntryIdTravelTime := bs.forwardPq.GetPriority(vEntryId)
			if scannedByForwardSearch && da.Lt(vEntryIdTravelTime+bs.engine.metrics.GetTurnCost(turnType2)+
				newVExitIdTravelTime, bs.shortestTravelTime) {

				bs.shortestTravelTime = vEntryIdTravelTime + bs.engine.metrics.GetTurnCost(turnType2) +
					newVExitIdTravelTime

				bs.forwardMid = da.NewVertexEdgePair(vId, vEntryId, false)
				bs.backwardMid = da.NewVertexEdgePair(vId, vExitId, true)

				bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(vId, vEntryId, vExitId, vId, false))
			} else if labelledByForwardSearch {
				bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(vId, vEntryId, vExitId, vId, false))
			}
			vEntryId++
		})

	})
}

func (bs *BidirectionalDijkstra) Preallocate() {
	maxSearch := bs.engine.graph.NumberOfEdges()

	bs.stallingEntry = make([]float64, maxSearch)
	bs.stallingExit = make([]float64, maxSearch)
	initInfWeight(bs.stallingEntry)
	initInfWeight(bs.stallingExit)

	bs.forwardPq = da.NewQueryHeap[da.CRPQueryKey](maxSearch, maxSearch, da.ARRAY_STORAGE)
	bs.backwardPq = da.NewQueryHeap[da.CRPQueryKey](maxSearch, maxSearch, da.ARRAY_STORAGE)
}
