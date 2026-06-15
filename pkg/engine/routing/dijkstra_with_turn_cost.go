package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type DijkstraWithTurnCost[W util.RoutingNumber] struct {
	engine *CRPRoutingEngine[W]

	finalCost           []W
	finalQueryKey       []da.VertexInfo[W]
	finalEdge           []da.Index
	shortestTravelTimes []W

	pq *da.QueryHeap[da.CRPQueryKey, W]

	sForwardId da.Index

	numSettledNodes  int
	useReversedEdges bool
}

func NewDijkstraWithTurnCost[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
	useReversedEdges bool,
) DijkstraWithTurnCost[W] {
	dj := DijkstraWithTurnCost[W]{
		engine:        engine,
		finalQueryKey: make([]da.VertexInfo[W], 0),

		numSettledNodes:     0,
		shortestTravelTimes: make([]W, 0),
		useReversedEdges:    useReversedEdges,
	}

	dj.Preallocate()
	return dj
}

/*
single-source shortest paths, from s to all other vertices
edge-based graph, support turn-costs..
see section 4.2 (Dijkstra's Algorithm on the Compact Graph): https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

useReversedEdges = true -> buat cari sssp dari every vertices in graph to s

karena di multilevel-dijkstra/multilevel-alt kita query dari origin phantom node ke destination phantom node (lihat phantom_node.go):

q -originPhantomEdge->s->....................... -t-destPhantomEdge->z

originPhantomEdge:						destPhantomEdge:
q------originPhantomNode----->s    t------destPhantomNode----->z

shortest path di multilevel-dijkstra/multilevel alt adalah shortest path dari s ke t + turn cost dari originPhantomEdge ke outEdge dari s +
turn cost dari inEdge dari t ke destPhantomEdge ..
kita udah include turn cost dari originPhantomEdge ke other outEdge dari s di awal forward search dan cost dari inEdge dari t ke destPhantomEdge di awal backward search
sebenarnya setelah multilevel-dijkstra selesai (di routing.go), kita tambahin sp cost nya dengan travelTime(originPhantomNode, s) + travelTime(t, destPhantomNode)

untuk referensi lain implementasi routing with turn cost di road network dapat dilihat di:
1. https://dl.acm.org/doi/10.5555/2008623.2008634
2. https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
2. multilevel-dijkstranya OSRM: https://github.com/Project-OSRM/osrm-backend/blob/master/include/engine/routing_algorithms/routing_base_mld.hpp
*/
func (us *DijkstraWithTurnCost[W]) ShortestPath(s da.Index) ([]W, [][]da.Index) {
	var (
		sForwardId da.Index
	)

	if !us.useReversedEdges {
		sForwardId = us.engine.graph.GetDummyInEdgeId(s)
	} else {
		sForwardId = us.engine.graph.GetExitOffset(s) + us.engine.graph.GetOutDegree(s) - 1
	}

	n := us.engine.graph.NumberOfVertices()
	us.finalCost = make([]W, n)
	for v := 0; v < n; v++ {
		us.finalCost[v] = util.Infinity[W]()
	}
	us.sForwardId = sForwardId

	sVertexInfo := da.NewVertexInfo(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))

	djKey := da.NewDijkstraKey(s, sForwardId)
	us.pq.Insert(sForwardId, 0, sVertexInfo, djKey)

	for !us.pq.IsEmpty() {

		us.graphSearchUni(s)
		us.numSettledNodes++
	}

	spEdges := make([][]da.Index, n)
	sps := make([]W, n)
	if !us.useReversedEdges {

		for t := da.Index(0); t < da.Index(n); t++ {
			curInfo := us.finalQueryKey[t]
			tEntryId := us.finalEdge[t]
			sp := us.finalCost[t]

			if s == t {
				continue // sp == 0
			}

			sps[t] = sp
			if util.Ge(sp, util.Infinity[W]()) {
				continue
			}

			// jadiin outEdge semua
			inEdge := us.engine.graph.GetInEdge(tEntryId)
			_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
			spEdges[t] = append(spEdges[t], outEdge)

			for curInfo.GetParent().GetEdge() != sForwardId {
				parent := curInfo.GetParent()
				parentEdge := parent.GetEdge()

				// jadiin outEdge semua
				inEdge := us.engine.graph.GetInEdge(parentEdge)
				_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
				spEdges[t] = append(spEdges[t], outEdge)

				curInfo = us.pq.Get(parentEdge)
			}

			util.ReverseG(spEdges[t])
		}

	} else {
		//  use reversed edges
		for t := da.Index(0); t < da.Index(n); t++ {
			curInfo := us.finalQueryKey[t]
			tExitId := us.finalEdge[t]
			sp := us.finalCost[t]

			if s == t {
				continue // sp == 0
			}
			sps[t] = sp
			if util.Ge(sp, util.Infinity[W]()) {
				continue
			}

			// jadiin outEdge semua
			outEdge := us.engine.graph.GetOutEdge(tExitId)
			spEdges[t] = append(spEdges[t], outEdge.GetEdgeId())

			for curInfo.GetParent().GetEdge() != sForwardId {
				parent := curInfo.GetParent()
				parentEdge := parent.GetEdge()

				// jadiin outEdge semua
				outEdge := us.engine.graph.GetOutEdge(parentEdge)
				spEdges[t] = append(spEdges[t], outEdge.GetEdgeId())

				curInfo = us.pq.Get(parentEdge)
			}
		}

		// kita mau cari
		// dari v -> .... -> landmark, for all vertices v
		// karena pakai reversed edges: landmark -> .... -> v ,
		// setiap reversed edge (v,u) punya weight sama dengan weight edge (u,v).
		// shortest path dari landmark ke v pakai reversed edges equivalent to shortest path dari v ke landmark pakai original edges
		// kita dapet spEdges berupa list of outEdges dari v ,... ke landmark
		// gak perlu direverse
	}

	return sps, spEdges
}

func (us *DijkstraWithTurnCost[W]) graphSearchUni(source da.Index) bool {
	// taken from Delling, D. et al. (2015) “Customizable Route Planning in Road Networks,” Transportation Science [Preprint]. Available at: https://doi.org/10.1287/trsc.2014.0579 :
	// The query algorithm maintains a distance label d(u) for each entry u which can either be a vertex on the overlay or a pair (u, i) corresponding to the i-th entry point of u in the original graph.
	// for forward search, we traverse outEdges of the graph and store (u, entryPoint of outEdge) to represent the key of the priority queue.
	// we need to store entryPoint because we need to know turnType & turn cost when traversing from inEdge to outEdge of vertex u.
	// forward search  on graph level 1
	queryKey := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	uId := uItem.GetNode()
	uCost := queryKey.GetRank()

	if !us.useReversedEdges {
		uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId
		// -uEntry> u

		uEntryPoint := uEntryId - us.engine.graph.GetEntryOffset(uId)

		/*
			karena di multilevel-dijkstra/multilevel-alt kita query dari origin phantom node ke destination phantom node (lihat phantom_node.go):

			q -originPhantomEdge->s->....................... -t-destPhantomEdge->z

			originPhantomEdge:						destPhantomEdge:
			q------originPhantomNode----->s    t------destPhantomNode----->z

			shortest path di multilevel-dijkstra/multilevel alt adalah shortest path dari s ke t + turn cost dari originPhantomEdge ke outEdge dari s +
			turn cost dari inEdge dari t ke destPhantomEdge ..
			kita udah include turn cost dari originPhantomEdge ke other outEdge dari s di awal, tapi belum turn cost dari inEdge dari t ke destPhantomEdge
			jadi kode dibawah untuk tambahin turn cost dari inEdge dari t ke destPhantomEdge..

			sebenarnya setelah multilevel-dijkstra selesai (di routing.go), kita tambahin sp cost nya dengan travelTime(originPhantomNode, s) + travelTime(t, destPhantomNode)

		*/

		exitIdFromTarget := us.engine.graph.GetDummyOutEdgeId(uId)
		_, at := us.engine.graph.GetTailOfOutedgeWithInEdge(exitIdFromTarget)

		us.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(eId, head da.Index, exitPoint, entryPoint, turnTableId da.Index, turnType pkg.TurnType, hwType pkg.OsmHighwayType) {
			turnCost := us.engine.metrics.GetTurnCost(turnTableId)
			newCost := uCost + turnCost

			headEntryId := us.engine.graph.GetEntryOffset(head) + da.Index(entryPoint)
			if headEntryId != at {
				return
			}

			if util.Eq(us.finalCost[uId], util.Infinity[W]()) || util.Lt(newCost, us.finalCost[uId]) {
				us.finalQueryKey[uId] = us.pq.Get(uEntryId)
				us.finalEdge[uId] = uEntryId
				us.finalCost[uId] = newCost
			}
		})

		// traverse outEdges of u
		us.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(eId, head da.Index, exitPoint, entryPoint, turnTableId da.Index, turnType pkg.TurnType,
			hwType pkg.OsmHighwayType) {

			vId := head

			edgeWeight := us.engine.getWeight(eId, true)

			turnCost := us.engine.metrics.GetTurnCost(turnTableId)

			// get cost to reach v through u + turn cost from inEdge to outEdge of u
			newTravelTime := uCost + edgeWeight + turnCost

			if util.Ge(newTravelTime, util.Infinity[W]()) {
				return
			}

			vEntryId := us.engine.graph.GetEntryOffset(vId) + da.Index(entryPoint)

			vAlreadyLabelled := util.Lt(us.pq.GetPriority(vEntryId), util.Infinity[W]())
			if vAlreadyLabelled && util.Ge(newTravelTime, us.pq.GetPriority(vEntryId)) {
				// newTravelTime is not better, do nothing

				return
			}

			// newTravelTime is better, update the forwardInfo

			if vAlreadyLabelled {
				newPar := da.NewVertexEdgePair(uId, uEntryId, false)
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vEntryId, newTravelTime, newTravelTime, newPar)

			} else if !vAlreadyLabelled {
				queryKey := da.NewDijkstraKey(vId, vEntryId)
				vertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(uId, uEntryId, false))

				// is key not in the priority queue, insert it
				us.pq.Insert(vEntryId, newTravelTime, vertexInfo, queryKey)
			}
		})
	} else {
		// use reversed edges
		uExitId := uItem.GetEntryExitPoint()

		// // u-uExit->

		uExitPoint := uExitId - us.engine.graph.GetExitOffset(uId)
		us.engine.graph.ForInEdgesOf(uId, uExitPoint, func(eId, tail da.Index, exitPoint, entryPoint, turnTableId da.Index, turnType pkg.TurnType, hwType pkg.OsmHighwayType) {
			turnCost := us.engine.metrics.GetTurnCost(turnTableId)
			newCost := uCost + turnCost

			if util.Eq(us.finalCost[uId], util.Infinity[W]()) || util.Lt(newCost, us.finalCost[uId]) {
				us.finalQueryKey[uId] = us.pq.Get(uExitId)
				us.finalEdge[uId] = uExitId
				us.finalCost[uId] = newCost
			}
		})

		// traverse inEdges of u
		us.engine.graph.ForInEdgesOf(uId, uExitPoint, func(eId, tail da.Index, exitPoint, entryPoint, turnTableId da.Index,
			turnType pkg.TurnType, hwType pkg.OsmHighwayType) {

			vId := tail

			edgeWeight := us.engine.getWeight(eId, false)

			turnCost := us.engine.metrics.GetTurnCost(turnTableId)

			newTravelTime := us.pq.GetPriority(uExitId) + edgeWeight + turnCost

			if util.Ge(newTravelTime, util.Infinity[W]()) {
				return
			}

			vExitId := us.engine.graph.GetExitOffset(vId) + da.Index(exitPoint)

			vAlreadyLabelled := util.Lt(us.pq.GetPriority(vExitId), util.Infinity[W]())
			if vAlreadyLabelled && util.Ge(newTravelTime, us.pq.GetPriority(vExitId)) {
				// newTravelTime is not better, do nothing
				return
			}

			// newTravelTime is better, update the forwardInfo

			if vAlreadyLabelled {
				newPar := da.NewVertexEdgePair(uId, uExitId, false)
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vExitId, newTravelTime, newTravelTime, newPar)

			} else if !vAlreadyLabelled {
				queryKey := da.NewDijkstraKey(vId, vExitId)
				vertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(uId, uExitId, false))

				// is key not in the priority queue, insert it
				us.pq.Insert(vExitId, newTravelTime, vertexInfo, queryKey)
			}
		})
	}

	return false
}

func (us *DijkstraWithTurnCost[W]) Preallocate() {
	numberOfEdges := us.engine.graph.NumberOfEdges()
	maxSearchSize := numberOfEdges
	numberOfVerties := us.engine.graph.NumberOfVertices()
	us.finalQueryKey = make([]da.VertexInfo[W], numberOfVerties)
	for i := 0; i < numberOfVerties; i++ {
		us.finalQueryKey[i] = da.NewVertexInfo(util.Infinity[W](), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	}
	us.finalEdge = make([]da.Index, numberOfVerties)
	maxEdgesInCell := us.engine.graph.GetMaxEdgesInCell()
	us.pq = da.NewQueryHeap[da.CRPQueryKey, W](uint32(maxSearchSize), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
	us.pq.PreallocateHeap(maxSearchSize)
}
