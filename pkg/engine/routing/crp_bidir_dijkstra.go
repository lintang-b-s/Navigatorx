package routing

import (
	"math"

	"github.com/lintang-b-s/navigatorx-crp/pkg"
	"github.com/lintang-b-s/navigatorx-crp/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx-crp/pkg/metrics"
)

type CRPRoutingEngine struct {
	graph        *datastructure.Graph
	overlayGraph *datastructure.OverlayGraph
	metrics      *metrics.Metric
}

func NewCRPRoutingEngine(graph *datastructure.Graph,
	overlayGraph *datastructure.OverlayGraph, metrics *metrics.Metric) *CRPRoutingEngine {
	return &CRPRoutingEngine{
		graph:        graph,
		metrics:      metrics,
		overlayGraph: overlayGraph,
	}
}

type CRPBidirectionalSearch struct {
	engine       *CRPRoutingEngine
	shortestPath float64
	forwardMid   vertexEdgePair
	backwardMid  vertexEdgePair

	forwardInfo  map[datastructure.Index]VertexInfo
	backwardInfo map[datastructure.Index]VertexInfo

	forwardPq         *datastructure.MinHeap[datastructure.CRPQueryKey]
	backwardPq        *datastructure.MinHeap[datastructure.CRPQueryKey]
	forwardOverlayPq  *datastructure.MinHeap[datastructure.CRPQueryKey]
	backwardOverlayPq *datastructure.MinHeap[datastructure.CRPQueryKey]

	maxEdgesInCell                   datastructure.Index
	forwardInSameSCellOffset         datastructure.Index
	forwardDifferentFromSCellOffset  datastructure.Index
	backwardInSameSCellOffset        datastructure.Index
	backwardDifferentFromSCellOffset datastructure.Index
	sCellNumber                      datastructure.Pv
	tCellNumber                      datastructure.Pv
}

func NewCRPBidirectionalSearch(engine *CRPRoutingEngine) *CRPBidirectionalSearch {
	return &CRPBidirectionalSearch{
		engine:            engine,
		forwardInfo:       make(map[datastructure.Index]VertexInfo),
		backwardInfo:      make(map[datastructure.Index]VertexInfo),
		forwardPq:         datastructure.NewMinHeap[datastructure.CRPQueryKey](),
		backwardPq:        datastructure.NewMinHeap[datastructure.CRPQueryKey](),
		forwardOverlayPq:  datastructure.NewMinHeap[datastructure.CRPQueryKey](),
		backwardOverlayPq: datastructure.NewMinHeap[datastructure.CRPQueryKey](),
		forwardMid:        newVertexEdgePair(0, 0),
		backwardMid:       newVertexEdgePair(0, 0),
	}
}

func (bs *CRPBidirectionalSearch) Search(source, target datastructure.Index) (float64, []datastructure.Index, bool) {

	// Our query algorithm takes as input a source arc as , a target arc at, the original graph G, the overlay graph
	// H = ∪i Hi , and computes the shortest path between the head vertex s of as and the tail vertex t of at.

	// get outEdge that enters head at source
	// u -> s
	// first get inEdge that enters head at source
	inEdgeToSource := bs.engine.graph.GetInEdge(bs.engine.graph.GetEntryOffset(source))
	// get outEdge index that exits tail at u to source
	// every inEdge s<-u has corresponding exitPoint that point to index of outEdge u->s in outEdges of u.
	u := inEdgeToSource.GetTail()
	// as = outEdge (u->s) offset
	asId := bs.engine.graph.GetExitOffset(u) + datastructure.Index(inEdgeToSource.GetExitPoint())

	// get inEdge that exits tail at target
	// v <- t
	// first get outEdge that exits tail at target
	outEdgeFromTarget := bs.engine.graph.GetOutEdge(bs.engine.graph.GetExitOffset(target))
	// get inEdge index that enters head at v from target
	// every outEdge t->v has corresponding entryPoint that point to index of inEdge v<-t in inEdges of v.
	// atId = inEdge (t->v) offset
	v := outEdgeFromTarget.GetHead()
	atId := bs.engine.graph.GetEntryOffset(v) + datastructure.Index(outEdgeFromTarget.GetEntryPoint())

	s := bs.engine.graph.GetOutEdge(asId).GetHead()
	t := bs.engine.graph.GetInEdge(atId).GetTail()

	bs.sCellNumber = bs.engine.graph.GetCellNumber(s)
	bs.tCellNumber = bs.engine.graph.GetCellNumber(t)

	// strategy: use outEdges for forward search, use inEdges for backward search
	// for iterating outEdges, we need entryOffset. for iterating inEdges, we need exitOffset.

	sEntryOffset := bs.engine.graph.GetEntryOffset(s) + datastructure.Index(bs.engine.graph.GetOutEdge(asId).GetEntryPoint())
	tExitOffset := bs.engine.graph.GetExitOffset(t) + datastructure.Index(bs.engine.graph.GetInEdge(atId).GetExitPoint())

	// remember, we store the inEdges of each vertex in each cell in each level to the inEdges field of the graph.

	bs.maxEdgesInCell = bs.engine.graph.GetMaxEdgesInCell()

	// we have to get the index of the first in/out edge of  source and target cell
	bs.forwardInSameSCellOffset = bs.engine.graph.GetInEdgeCellOffset(source)
	bs.backwardInSameSCellOffset = bs.engine.graph.GetOutEdgeCellOffset(source)

	bs.forwardDifferentFromSCellOffset = forwardDifferentFromSCellConst - bs.maxEdgesInCell
	bs.backwardDifferentFromSCellOffset = backwardDifferentFromTCellConst - bs.maxEdgesInCell

	overlayOffset := 100 * bs.maxEdgesInCell

	sForwardId := sEntryOffset - bs.forwardInSameSCellOffset
	tBackwardId := tExitOffset

	// to mark whether the target is in the same cell as the source
	if bs.engine.graph.GetCellNumber(source) == bs.engine.graph.GetCellNumber(target) {
		tBackwardId -= bs.backwardInSameSCellOffset
	} else {
		tBackwardId -= bs.backwardDifferentFromSCellOffset
	}

	bs.shortestPath = 2 * pkg.INF_WEIGHT

	bs.forwardInfo[sForwardId] = NewVertexInfo(0, newVertexEdgePair(s, asId))
	bs.backwardInfo[tBackwardId] = NewVertexInfo(0, newVertexEdgePair(t, atId))

	bs.forwardPq.Insert(datastructure.NewPriorityQueueNode(0, datastructure.NewCRPQueryKey(s, sForwardId)))
	bs.backwardPq.Insert(datastructure.NewPriorityQueueNode(0, datastructure.NewCRPQueryKey(t, tBackwardId)))

	for bs.forwardPq.Size()+bs.forwardOverlayPq.Size() > 0 && bs.backwardPq.Size()+bs.backwardOverlayPq.Size() > 0 {
		if math.Min(bs.forwardPq.GetMinrank(), bs.forwardOverlayPq.GetMinrank())+
			math.Min(bs.backwardPq.GetMinrank(), bs.backwardOverlayPq.GetMinrank()) > bs.shortestPath {
			break
		}

		minGraph := math.Min(bs.forwardPq.GetMinrank(), bs.backwardPq.GetMinrank())
		minOverlay := math.Min(bs.forwardOverlayPq.GetMinrank(), bs.backwardOverlayPq.GetMinrank())

		// Customizable Route Planning In Road Networks, Daniel Delling, page 14:
		// Each iteration of the algorithm takes the minimum-distance entry from the queue, representing either an
		// overlay vertex u or a pair (u, i) from the original graph. If the entry is a pair, we scan it using the turn-aware
		// version of Dijkstra’s algorithm (and look at its neighbors in G). Otherwise, we use the overlay graph at level
		// lst (u), which does not have turns. In either case, the neighbors v of u are added to the priority queue with
		// the appropriate distance labels. Note that a level transition occurs when u and v have different query levels;
		if minGraph < minOverlay {
			// search on graph level 1
			bs.graphSearch(source, target, overlayOffset)
		} else {
			// search on overlay graph
			bs.overlayGraphSearch(overlayOffset)
		}
	}

	if bs.shortestPath == 2*pkg.INF_WEIGHT {
		return pkg.INF_WEIGHT, []datastructure.Index{}, false
	}

	// unpackOverlayOffset := bs.engine.graph.NumberOfEdges()

	// idPath := make([]vertexEdgePair, 0)
	// curId := bs.forwardMid.edge
	// for curId != asId {
	// 	curInfo := bs.forwardInfo[curId]
	// 	parent := curInfo.GetParent()
	// 	parentCopy := parent

	// 	if parentCopy.getEdge() < overlayOffset {
	// 		// original entry/exit edge
	// 		if parentCopy.getEdge() < bs.maxEdgesInCell {
	// 			// same cell as source
	// 			parentCopy.setEdge(parentCopy.getEdge() + bs.forwardInSameSCellOffset)
	// 		} else {
	// 			parentCopy.setEdge(parentCopy.getEdge() + bs.forwardDifferentFromSCellOffset)
	// 		}
	// 	} else {

	// 		// overlay vertex
	// 		parentCopy.setEdge(parentCopy.getEdge() - overlayOffset + datastructure.Index(unpackOverlayOffset))
	// 	}

	// 	idPath = append(idPath, parentCopy)

	// 	curId = parent.getEdge()
	// }

	// idPath = append(idPath, newVertexEdgePair(s, asId))
	// idPath = util.ReverseG[vertexEdgePair](idPath)

	// mid := bs.backwardMid
	// if mid.getEdge() < overlayOffset {
	// 	// original entry/exit edge
	// 	if mid.getEdge() < bs.maxEdgesInCell {
	// 		// same cell as source
	// 		mid.setEdge(mid.getEdge() + bs.backwardInSameSCellOffset)
	// 	} else {
	// 		mid.setEdge(mid.getEdge() + bs.backwardDifferentFromSCellOffset)
	// 	}
	// } else {
	// 	// overlay vertex
	// 	mid.setEdge(mid.getEdge() - overlayOffset + datastructure.Index(unpackOverlayOffset))
	// }

	// idPath = append(idPath, mid)

	// curId = bs.backwardMid.edge
	// for curId != atId {
	// 	curInfo := bs.backwardInfo[curId]
	// 	parent := curInfo.GetParent()
	// 	parentCopy := parent

	// 	if parentCopy.getEdge() < overlayOffset {
	// 		// original entry/exit edge
	// 		if parentCopy.getEdge() < bs.maxEdgesInCell {
	// 			// same cell as source
	// 			parentCopy.setEdge(parentCopy.getEdge() + bs.backwardInSameSCellOffset)
	// 		} else {
	// 			parentCopy.setEdge(parentCopy.getEdge() + bs.backwardDifferentFromSCellOffset)
	// 		}
	// 	} else {
	// 		// overlay vertex
	// 		parentCopy.setEdge(parentCopy.getEdge() - overlayOffset + datastructure.Index(unpackOverlayOffset))
	// 	}
	// 	idPath = append(idPath, parentCopy)

	// 	curId = parent.getEdge()
	// }

	// idPath = append(idPath, newVertexEdgePair(t, atId))

	// unpacker := NewPathUnpacker(bs.engine.graph, bs.engine.overlayGraph, bs.engine.metrics)
	// finalPath := unpacker.unpackPath(idPath, bs.sCellNumber, bs.tCellNumber)
	return bs.shortestPath, []datastructure.Index{}, true
}

/*
	graphSearch. turn-aware bidirectional dijkstra search on graph level 1.

Customizable Route Planning In Road Networks, Daniel Delling, page 7-8:

We implement this by main-taining triples (v, i, d) in the heap, where v is a vertex, i the order of an entry point at v, and d a distance
label. The algorithm is initialized by (s, i, 0) indicating that we start the query from entry point i at vertex
s. (Note that one can generalize this to allow queries starting anywhere along an arc by inserting s, i with
an offset into the queue.) The distance value d of a label (v, i, d) then indicates the cost of the best path
seen so far from the source to the entry point i at vertex v.

To implement bidirectional search on the compact model, we maintain a tentative
shortest path distance µ, initialized by ∞. Then, we perform a forward search from an entry point at s,
operating as described above, and a backward search from an exit point of t that operates on the exit points
of the vertices.

Whenever we scan a vertex that has been seen from the other side, we evaluate all possible turns between all entry and exit points of the intersection and check
whether we can improve µ. We can stop the search as soon as the sum of the minimum keys in both priority
queues exceeds µ. Note that this algorithm basically performs a search from the head vertex (s) of an arc to
the tail (t) of another.
*/
func (bs *CRPBidirectionalSearch) graphSearch(source, target, overlayOffset datastructure.Index) {
	minrankForward := bs.forwardPq.GetMinrank()
	minrankBackward := bs.backwardPq.GetMinrank()
	if minrankForward < minrankBackward {
		//The query algorithm maintains a distance label d(u) for each entry u which can either be a vertex on the overlay or a pair (u, i) corresponding to the i-th entry point of u in the original graph.
		// for forward search, we traverse outEdges of the graph and store (u, entryPoint of outEdge) to represent the key of the priority queue.
		// we need to store entryPoint because we need to know turnType & turn cost when traversing from inEdge to outEdge of vertex u.
		// forward search  on graph level 1
		queryKey, _ := bs.forwardPq.ExtractMin()
		uItem := queryKey.GetItem()
		uId := uItem.GetNode()
		uEntryPoint := uItem.GetEntryExitPoint()

		uEntryPointAdjusted := bs.getEntryPointForward(uEntryPoint, uId)

		// traverse outEdges of u
		bs.engine.graph.ForOutEdgesOf(uId, uEntryPointAdjusted, func(outArc *datastructure.OutEdge, exitPoint datastructure.Index, turnType pkg.TurnType) {
			vId := outArc.GetHead()

			// get query level of v l_st(v)
			vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
				bs.engine.graph.GetCellNumber(vId))

			edgeWeight := bs.engine.metrics.GetWeight(outArc)
			turnCost := bs.engine.metrics.GetTurnCost(turnType)
			if uId == source {
				turnCost = 0
			}
			// get cost to reach v through u + turn cost from inEdge to outEdge of u
			newEta := bs.forwardInfo[uEntryPoint].GetEta() + edgeWeight + turnCost

			if newEta >= pkg.INF_WEIGHT {
				return
			}

			if vQueryLevel == 0 {
				// if query level of v is 0, then v is in the same cell as s or t
				// then, we just do edge relaxation as usual in turn-aware dijkstra

				vEntryPoint := bs.engine.graph.GetEntryOffset(vId) + datastructure.Index(outArc.GetEntryPoint())
				vEntryPoint = bs.adjustForwardVertexId(vEntryPoint, vId)

				_, vAlreadyVisited := bs.forwardInfo[vEntryPoint]
				if newEta > bs.forwardInfo[vEntryPoint].GetEta() && vAlreadyVisited {
					// newEta is not better, do nothing
					return
				}

				// newEta is better, update the forwardInfo
				bs.forwardInfo[vEntryPoint] = NewVertexInfo(newEta,
					newVertexEdgePair(uId, uEntryPoint))

				if vAlreadyVisited {
					// is key already in the priority queue, decrease its key
					bs.forwardPq.DecreaseKey(datastructure.NewPriorityQueueNode(
						newEta, datastructure.NewCRPQueryKey(vId, vEntryPoint)),
					)
				} else if !vAlreadyVisited {
					// is key not in the priority queue, insert it
					bs.forwardPq.Insert(datastructure.NewPriorityQueueNode(
						newEta, datastructure.NewCRPQueryKey(vId, vEntryPoint)),
					)
				}

				// check wether we already visited an exit point

				exitOffset := bs.engine.graph.GetExitOffset(vId)
				vInSCell := bs.engine.graph.GetCellNumber(vId) == bs.sCellNumber
				if vInSCell {
					exitOffset -= bs.backwardInSameSCellOffset
				} else {
					exitOffset -= bs.backwardDifferentFromSCellOffset
				}
				vExitPoint := exitOffset

				// traverse outEdges of v
				bs.engine.graph.ForOutEdgesOf(vId, datastructure.Index(outArc.GetEntryPoint()), func(e2 *datastructure.OutEdge,
					exitPoint datastructure.Index, turnType2 pkg.TurnType) {
					// Customizable Route Planning In Road Networks, Page 8:  Whenever we scan a vertex that has been seen from
					// the other side, we evaluate all possible turns between all entry and exit points of the intersection and check
					// whether we can improve µ.
					// basically: check if forward and backward search already visited entry and exit point of v. if so, check whether we can improve the shortest path
					_, visitedByBackwardSearch := bs.backwardInfo[vExitPoint]
					if visitedByBackwardSearch {
						// if head of outEdge v->w already visited by backward search, and its forwardEta + backwardEta is better than shortestPath, then update shortestPath
						newPathEta := bs.forwardInfo[vEntryPoint].GetEta() + bs.engine.metrics.GetTurnCost(turnType2) +
							bs.backwardInfo[vExitPoint].GetEta()

						if newPathEta < bs.shortestPath {
							bs.shortestPath = newPathEta
							bs.forwardMid = newVertexEdgePair(vId, vEntryPoint)
							bs.backwardMid = newVertexEdgePair(vId, vExitPoint)
						}
					}
					vExitPoint++
				})

			} else {
				// v is in another cell on higher level
				// update the forward info of overlay vertex v
				// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
				v, _ := bs.engine.graph.GetOverlayVertex(vId, outArc.GetEntryPoint(), false)
				vId = v + overlayOffset
				_, vAlreadyVisited := bs.forwardInfo[vId]
				if !vAlreadyVisited || newEta < bs.forwardInfo[vId].GetEta() {

					vertexInfo := NewVertexInfo(newEta,
						newVertexEdgePair(uId, uEntryPoint))

					bs.forwardInfo[vId] = vertexInfo
					if !vAlreadyVisited {
						bs.forwardOverlayPq.Insert(datastructure.NewPriorityQueueNode(
							newEta, datastructure.NewCRPQueryKey(v, datastructure.Index(vQueryLevel))),
						)
					} else {
						bs.forwardOverlayPq.DecreaseKey(datastructure.NewPriorityQueueNode(
							newEta, datastructure.NewCRPQueryKey(v, datastructure.Index(vQueryLevel))),
						)
					}

					_, visitedByBackwardSearch := bs.backwardInfo[vId]
					// if v visited by backward search, check whether we can improve the shortestPath
					if visitedByBackwardSearch && bs.forwardInfo[vId].GetEta()+bs.backwardInfo[vId].GetEta() < bs.shortestPath {
						bs.shortestPath = bs.forwardInfo[vId].GetEta() + bs.backwardInfo[vId].GetEta()
						bs.forwardMid = newVertexEdgePair(v, vId)
						bs.backwardMid = newVertexEdgePair(v, vId)
					}
				}
			}
		})

	} else {
		// search backward on graph level 1
		// basically same as forward search, but using inEdges and exitPoint instead of outEdges and entryPoint
		queryKey, _ := bs.backwardPq.ExtractMin()
		uItem := queryKey.GetItem()
		uId := uItem.GetNode()
		uExitPoint := uItem.GetEntryExitPoint()

		uExitPointAdjusted := bs.getExitPointBackward(uExitPoint, uId)

		bs.engine.graph.ForInEdgesOf(uId, uExitPointAdjusted, func(inArc *datastructure.InEdge, exitPoint datastructure.Index, turnType pkg.TurnType) {
			vId := inArc.GetTail()

			vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
				bs.engine.graph.GetCellNumber(vId))

			edgeWeight := bs.engine.metrics.GetWeight(inArc)
			turnCost := bs.engine.metrics.GetTurnCost(turnType)

			if uId == target {
				turnCost = 0
			}

			newEta := bs.backwardInfo[uExitPoint].GetEta() + edgeWeight + turnCost

			if newEta >= pkg.INF_WEIGHT {
				return
			}

			if vQueryLevel == 0 {
				vExitPoint := bs.engine.graph.GetExitOffset(vId) + datastructure.Index(inArc.GetExitPoint())
				vExitPoint = bs.adjustBackwardVertexId(vExitPoint, vId)

				_, vAlreadyVisited := bs.backwardInfo[vExitPoint]

				if newEta > bs.backwardInfo[vExitPoint].GetEta() && vAlreadyVisited {
					return
				}

				bs.backwardInfo[vExitPoint] = NewVertexInfo(newEta,
					newVertexEdgePair(uId, uExitPoint))
				if newEta < bs.backwardInfo[vExitPoint].GetEta() && vAlreadyVisited {
					bs.backwardPq.DecreaseKey(datastructure.NewPriorityQueueNode(
						newEta, datastructure.NewCRPQueryKey(vId, vExitPoint)),
					)
				} else if !vAlreadyVisited {
					bs.backwardPq.Insert(datastructure.NewPriorityQueueNode(
						newEta, datastructure.NewCRPQueryKey(vId, vExitPoint)),
					)
				}

				// check wether we already visited an entry point
				entryOffset := bs.engine.graph.GetEntryOffset(vId)
				if bs.engine.graph.GetCellNumber(vId) == bs.sCellNumber {
					entryOffset -= bs.forwardInSameSCellOffset
				} else {
					entryOffset -= bs.forwardDifferentFromSCellOffset
				}

				entryId := entryOffset

				bs.engine.graph.ForInEdgesOf(vId, datastructure.Index(inArc.GetExitPoint()), func(inArc2 *datastructure.InEdge,
					entryPoint2 datastructure.Index, turnType2 pkg.TurnType) {
					_, visitedByForwardSearch := bs.forwardInfo[entryId]

					if visitedByForwardSearch {
						newPathEta := bs.forwardInfo[entryId].GetEta() + bs.engine.metrics.GetTurnCost(turnType2) +
							bs.backwardInfo[vExitPoint].GetEta()
						if newPathEta < bs.shortestPath {
							bs.shortestPath = newPathEta
							bs.forwardMid = newVertexEdgePair(vId, entryId)
							bs.backwardMid = newVertexEdgePair(vId, vExitPoint)
						}
					}
					entryId++
				})

			} else {
				// v is in another cell on higher level
				v, _ := bs.engine.graph.GetOverlayVertex(vId, inArc.GetExitPoint(), true)
				vId := v + overlayOffset
				if bs.backwardInfo[uExitPoint].GetEta()+edgeWeight < bs.backwardInfo[vId].GetEta() {
					vertexInfo := NewVertexInfo(bs.backwardInfo[uExitPoint].GetEta()+edgeWeight,
						newVertexEdgePair(uId, uExitPoint))

					_, vAlreadyVisited := bs.backwardInfo[vId]
					bs.backwardInfo[vId] = vertexInfo

					if !vAlreadyVisited {
						bs.backwardOverlayPq.Insert(datastructure.NewPriorityQueueNode(
							bs.backwardInfo[vId].GetEta(), datastructure.NewCRPQueryKey(v, datastructure.Index(vQueryLevel))),
						)
					} else {
						bs.backwardOverlayPq.DecreaseKey(datastructure.NewPriorityQueueNode(
							bs.backwardInfo[vId].GetEta(), datastructure.NewCRPQueryKey(v, datastructure.Index(vQueryLevel))),
						)
					}

					_, visitedByForwardSearch := bs.forwardInfo[vId]

					if visitedByForwardSearch && bs.forwardInfo[vId].GetEta()+bs.backwardInfo[vId].GetEta() < bs.shortestPath {
						bs.shortestPath = bs.forwardInfo[vId].GetEta() + bs.backwardInfo[vId].GetEta()
						bs.forwardMid = newVertexEdgePair(v, vId)
						bs.backwardMid = newVertexEdgePair(v, vId)
					}

				}
			}
		})
	}
}

/*
Customizable Route Planning In Road Networks, Daniel Delling, page 14:
Each iteration of the algorithm takes the minimum-distance entry from the queue, representing either an
overlay vertex u or a pair (u, i) from the original graph. If the entry is a pair, we scan it using the turn-aware
version of Dijkstra’s algorithm (and look at its neighbors in G). Otherwise, we use the overlay graph at level
lst (u), which does not have turns. In either case, the neighbors v of u are added to the priority queue with
the appropriate distance labels. Note that a level transition occurs when u and v have different query levels;
*/
func (bs *CRPBidirectionalSearch) overlayGraphSearch(overlayOffset datastructure.Index) {
	// search on overlay graph
	minrankForward := bs.forwardOverlayPq.GetMinrank()
	minrankBackward := bs.backwardOverlayPq.GetMinrank()
	if minrankForward < minrankBackward {
		queryKey, _ := bs.forwardOverlayPq.ExtractMin()
		uItem := queryKey.GetItem()
		u := uItem.GetNode() // overlay vertex id

		uId := u + overlayOffset // overlay vertex id + overlayOffset to get unique id in forwardInfo & backwardInfo
		uVertex := bs.engine.overlayGraph.GetVertex(u)
		uQueryLevel := int(uItem.GetEntryExitPoint())

		// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
		// for each out neighbors of u in level l, check if v already visited by backward search. if so, check whether we can improve shortestPath
		// then if v not already visited or newEta to v is better, traverse to the next cell entry vertex w using outEdge of v.
		bs.engine.overlayGraph.ForOutNeighborsOf(u, uQueryLevel, func(v datastructure.Index, wOffset datastructure.Index) {
			newEta := bs.forwardInfo[uId].GetEta() + bs.engine.metrics.GetShortcutWeight(wOffset)
			if newEta >= pkg.INF_WEIGHT {
				return
			}
			vId := v + overlayOffset
			_, vAlreadyVisited := bs.forwardInfo[vId]
			if !vAlreadyVisited || newEta < bs.forwardInfo[vId].GetEta() {
				bs.forwardInfo[vId] = NewVertexInfo(newEta,
					newVertexEdgePair(uVertex.GetOriginalVertex(), uId))

				vVertex := bs.engine.overlayGraph.GetVertex(v)

				_, vVisitedByBackwardSearch := bs.backwardInfo[vId]
				if vVisitedByBackwardSearch && bs.forwardInfo[vId].GetEta()+bs.backwardInfo[vId].GetEta() < bs.shortestPath {
					bs.shortestPath = bs.forwardInfo[vId].GetEta() + bs.backwardInfo[vId].GetEta()
					bs.forwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), vId)
					bs.backwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), vId)
				}

				// traverse edge to next cell
				outEdge := bs.engine.graph.GetOutEdge(vVertex.GetOriginalEdge())
				edgeWeight := bs.engine.metrics.GetWeight(outEdge)
				newEta := bs.forwardInfo[vId].GetEta() + edgeWeight

				if newEta >= pkg.INF_WEIGHT {
					return
				}

				// w is in the next cell from v cell
				w := vVertex.GetNeighborOverlayVertex()
				wVertex := bs.engine.overlayGraph.GetVertex(w)
				wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
					wVertex.GetCellNumber())

				if wQueryLevel == 0 {
					// w is in the same cell as s or t

					wInSCell := wVertex.GetCellNumber() == bs.sCellNumber
					originalW := wVertex.GetOriginalVertex()
					originalWEntryPoint := bs.engine.graph.GetEntryOffset(originalW) + datastructure.Index(outEdge.GetEntryPoint())
					if wInSCell {
						originalWEntryPoint -= bs.forwardInSameSCellOffset
					} else {
						originalWEntryPoint -= bs.forwardDifferentFromSCellOffset
					}

					// relax entry Edge of w
					// update eta to reach entry point of w and insert entryPoint of w to forwardPq
					_, wAlreadyVisited := bs.forwardInfo[originalWEntryPoint]
					if wAlreadyVisited && newEta > bs.forwardInfo[originalWEntryPoint].GetEta() {
						return
					}

					bs.forwardInfo[originalWEntryPoint] = NewVertexInfo(newEta,
						newVertexEdgePair(vVertex.GetOriginalVertex(), vId))

					if wAlreadyVisited {
						bs.forwardPq.DecreaseKey(datastructure.NewPriorityQueueNode(
							newEta, datastructure.NewCRPQueryKey(originalW, originalWEntryPoint)),
						)
					} else {
						bs.forwardPq.Insert(datastructure.NewPriorityQueueNode(
							newEta, datastructure.NewCRPQueryKey(originalW, originalWEntryPoint)),
						)
					}

					// check whether we already visited an exit point
					exitOffset := bs.engine.graph.GetExitOffset(originalW)
					if wInSCell {
						exitOffset -= bs.backwardInSameSCellOffset
					} else {
						exitOffset -= bs.backwardDifferentFromSCellOffset
					}

					wExitId := exitOffset
					bs.engine.graph.ForOutEdgesOf(originalW, datastructure.Index(outEdge.GetEntryPoint()), func(e *datastructure.OutEdge, exitPoint datastructure.Index, turn pkg.TurnType) {
						// basically: check if forward and backward search already visited entry and exit point of w. if so, check whether we can improve the shortest path
						_, wVisitedByBackwardSearch := bs.backwardInfo[wExitId]
						if wVisitedByBackwardSearch {

							newPathEta := bs.forwardInfo[originalWEntryPoint].GetEta() + bs.engine.metrics.GetTurnCost(turn) +
								bs.backwardInfo[wExitId].GetEta()
							if newPathEta < bs.shortestPath {
								bs.shortestPath = newPathEta
								bs.forwardMid = newVertexEdgePair(originalW, originalWEntryPoint)
								bs.backwardMid = newVertexEdgePair(originalW, wExitId)
							}
						}
						wExitId++
					})
				} else {
					// w is in another cell on higher level
					// update new eta to reach overlay vertex w
					// insert item overlay vertex w and its query level to forwardOverlayPq, because we need to traverse & relax shortcut edges in overlay graph
					wId := w + overlayOffset
					_, wAlreadyVisited := bs.forwardInfo[wId]
					if !wAlreadyVisited || newEta < bs.forwardInfo[wId].GetEta() {
						bs.forwardInfo[wId] = NewVertexInfo(newEta,
							newVertexEdgePair(vVertex.GetOriginalVertex(), vId))

						if !wAlreadyVisited {
							bs.forwardOverlayPq.Insert(datastructure.NewPriorityQueueNode(
								newEta, datastructure.NewCRPQueryKey(w, datastructure.Index(wQueryLevel))),
							)
						} else {
							bs.forwardOverlayPq.DecreaseKey(datastructure.NewPriorityQueueNode(
								newEta, datastructure.NewCRPQueryKey(w, datastructure.Index(wQueryLevel))),
							)
						}

						_, wVisitedByBackwardSearch := bs.backwardInfo[wId]
						if wVisitedByBackwardSearch && bs.forwardInfo[wId].GetEta()+bs.backwardInfo[wId].GetEta() < bs.shortestPath {
							// if overlay vertex w visited by backward search, check whether we can improve the shortestPath
							bs.shortestPath = bs.forwardInfo[wId].GetEta() + bs.backwardInfo[wId].GetEta()
							bs.forwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), wId)
							bs.backwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), wId)
						}
					}
				}
			}
		})
	} else { // search backward on overlay graph
		// basically same as forward search on overlayGraph, but using inEdges and exitPoint instead of outEdges and entryPoint

		queryKey, _ := bs.backwardOverlayPq.ExtractMin()
		uItem := queryKey.GetItem()
		u := uItem.GetNode()

		uId := u + overlayOffset
		uVertex := bs.engine.overlayGraph.GetVertex(u)

		uQueryLevel := uItem.GetEntryExitPoint()

		bs.engine.overlayGraph.ForInNeighborsOf(u, int(uQueryLevel), func(v datastructure.Index,
			wOffset datastructure.Index) {
			newEta := bs.backwardInfo[uId].GetEta() + bs.engine.metrics.GetShortcutWeight(wOffset)
			if newEta >= pkg.INF_WEIGHT {
				return
			}

			vId := v + overlayOffset
			_, vAlreadyVisited := bs.backwardInfo[vId]
			if !vAlreadyVisited || newEta < bs.backwardInfo[vId].GetEta() {
				bs.backwardInfo[vId] = NewVertexInfo(newEta,
					newVertexEdgePair(uVertex.GetOriginalVertex(), uId))
				vVertex := bs.engine.overlayGraph.GetVertex(v)

				_, vVisitedByForwardSearch := bs.forwardInfo[vId]
				if vVisitedByForwardSearch && bs.backwardInfo[vId].GetEta()+bs.forwardInfo[vId].GetEta() < bs.shortestPath {
					bs.shortestPath = bs.backwardInfo[vId].GetEta() + bs.forwardInfo[vId].GetEta()
					bs.forwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), vId)
					bs.backwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), vId)
				}

				// traverse edge to next cell
				inEdge := bs.engine.graph.GetInEdge(vVertex.GetOriginalEdge())
				newEta := bs.backwardInfo[vId].GetEta() + bs.engine.metrics.GetWeight(inEdge)

				if newEta >= pkg.INF_WEIGHT {
					return
				}

				w := vVertex.GetNeighborOverlayVertex()
				wVertex := bs.engine.overlayGraph.GetVertex(w)
				wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
					wVertex.GetCellNumber())
				if wQueryLevel == 0 {
					wInSCell := wVertex.GetCellNumber() == bs.sCellNumber
					originalW := wVertex.GetOriginalVertex()
					originalWExitPoint := bs.engine.graph.GetExitOffset(originalW) + datastructure.Index(inEdge.GetExitPoint())
					if wInSCell {
						originalWExitPoint -= bs.backwardInSameSCellOffset
					} else {
						originalWExitPoint -= bs.backwardDifferentFromSCellOffset
					}

					_, wAlreadyVisited := bs.backwardInfo[originalWExitPoint]
					if newEta > bs.backwardInfo[originalWExitPoint].GetEta() && wAlreadyVisited {
						return
					}

					bs.backwardInfo[originalWExitPoint] = NewVertexInfo(newEta,
						newVertexEdgePair(vVertex.GetOriginalVertex(), vId))

					if wAlreadyVisited {

						bs.backwardPq.DecreaseKey(datastructure.NewPriorityQueueNode(
							newEta, datastructure.NewCRPQueryKey(originalW, originalWExitPoint)),
						)
					} else {

						bs.backwardPq.Insert(datastructure.NewPriorityQueueNode(
							newEta, datastructure.NewCRPQueryKey(originalW, originalWExitPoint)),
						)
					}

					// check whether we already visited an entry point
					entryOffset := bs.engine.graph.GetEntryOffset(originalW)
					if wInSCell {
						entryOffset -= bs.forwardInSameSCellOffset
					} else {
						entryOffset -= bs.forwardDifferentFromSCellOffset
					}
					wEntryId := entryOffset
					bs.engine.graph.ForInEdgesOf(originalW, datastructure.Index(inEdge.GetExitPoint()), func(e *datastructure.InEdge,
						entryPoint datastructure.Index, turn pkg.TurnType) {
						_, wVisitedByForwardSearch := bs.forwardInfo[wEntryId]
						if wVisitedByForwardSearch {
							newPathEta := bs.forwardInfo[wEntryId].GetEta() + bs.engine.metrics.GetTurnCost(turn) +
								bs.backwardInfo[originalWExitPoint].GetEta()
							if newPathEta < bs.shortestPath {
								bs.shortestPath = newPathEta
								bs.forwardMid = newVertexEdgePair(originalW, wEntryId)
								bs.backwardMid = newVertexEdgePair(originalW, originalWExitPoint)
							}
						}
						wEntryId++
					})
				} else {
					wId := w + overlayOffset
					_, wAlreadyvisited := bs.backwardInfo[wId]
					if !wAlreadyvisited || newEta < bs.backwardInfo[wId].GetEta() {
						bs.backwardInfo[wId] = NewVertexInfo(newEta,
							newVertexEdgePair(vVertex.GetOriginalVertex(), vId))

						if !wAlreadyvisited {
							bs.backwardOverlayPq.Insert(datastructure.NewPriorityQueueNode(
								newEta, datastructure.NewCRPQueryKey(w, datastructure.Index(wQueryLevel))),
							)
						} else {
							bs.backwardOverlayPq.DecreaseKey(datastructure.NewPriorityQueueNode(
								newEta, datastructure.NewCRPQueryKey(w, datastructure.Index(wQueryLevel))),
							)
						}

						_, wVisitedByForwardSearch := bs.forwardInfo[wId]
						if wVisitedByForwardSearch && bs.forwardInfo[wId].GetEta()+bs.backwardInfo[wId].GetEta() < bs.shortestPath {
							bs.shortestPath = bs.backwardInfo[wId].GetEta() + bs.forwardInfo[wId].GetEta()
							bs.forwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), wId)
							bs.backwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), wId)
						}
					}
				}
			}
		})
	}
}

func (bs *CRPBidirectionalSearch) getEntryPointForward(uEntryPoint, uId datastructure.Index) datastructure.Index {
	if uEntryPoint < bs.maxEdgesInCell {
		return uEntryPoint + bs.forwardInSameSCellOffset - bs.engine.graph.GetEntryOffset(uId)
	} else {
		return uEntryPoint + bs.forwardDifferentFromSCellOffset - bs.engine.graph.GetEntryOffset(uId)
	}
}

func (bs *CRPBidirectionalSearch) getExitPointBackward(uExitPoint, uId datastructure.Index) datastructure.Index {
	if uExitPoint < bs.maxEdgesInCell {
		return uExitPoint + bs.backwardInSameSCellOffset - bs.engine.graph.GetExitOffset(uId)
	} else {
		return uExitPoint + bs.backwardDifferentFromSCellOffset - bs.engine.graph.GetExitOffset(uId)
	}
}

// note when traversing vertice v in forward/backward, v can be in different cell than s/t
// we need to adjust the offset accordingly
// if v not in same cell as s, we have to subtract the value from forwardDifferentFromSCellOffset
// if v not in same cell as t, we have to subtract the value from backwardDifferentFromSCellOffset
func (bs *CRPBidirectionalSearch) adjustForwardVertexId(vEntryPoint, vId datastructure.Index) datastructure.Index {
	vInSCell := bs.engine.graph.GetCellNumber(vId) == bs.sCellNumber
	if vInSCell {
		return vEntryPoint - bs.forwardInSameSCellOffset
	} else {
		return vEntryPoint - bs.forwardDifferentFromSCellOffset
	}
}

func (bs *CRPBidirectionalSearch) adjustBackwardVertexId(vExitPoint, vId datastructure.Index) datastructure.Index {
	vInTCell := bs.engine.graph.GetCellNumber(vId) == bs.sCellNumber
	if vInTCell {
		return vExitPoint - bs.backwardInSameSCellOffset
	} else {
		return vExitPoint - bs.backwardDifferentFromSCellOffset
	}
}
