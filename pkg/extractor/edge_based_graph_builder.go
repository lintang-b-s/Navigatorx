package extractor

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"

	"github.com/spf13/viper"
)

/*
buildEdgeBasedGraph. build edge-based (or expanded) graph following:  https://github.com/Project-OSRM/osrm-backend/wiki/Graph-representation
https://github.com/Project-OSRM/osrm-backend/blob/master/src/extractor/edge_based_graph_factory.cpp
another reference:  https://drops.dagstuhl.de/storage/01oasics/oasics-vol085-atmos2020/OASIcs.ATMOS.2020.9/OASIcs.ATMOS.2020.9.pdf
https://i11www.iti.kit.edu/_media/teaching/theses/ba-zuendorf-19.pdf
parameter:
graph: node-based graph datastructure
weightFunction: weight (duration/travel time) of each edges of node-based graph

only for road network OpenStreetMap input file.
*/
func BuildEdgeBasedGraph[W util.Numeric](g *da.Graph, wf *met.TimeFunction[W],
	vTurnTableIds []da.Index, turnMatrix []pkg.TurnType, rn *da.RoadNetworkDataContainer) (*da.Graph, *met.TimeFunction[W], []uint64) {

	ebgvNum := da.Index(g.NumberOfEdges())
	ebgVertices := make([]da.Vertex, ebgvNum+1)
	ebgEdgeList := make([]ebgEdge, 0, ebgvNum)
	ebgMapping := make([]uint64, ebgvNum)

	speedFromWeight := func(length uint32, weight W) uint32 {
		return uint32(math.Round(float64(length) / float64(weight)))
	}

	eSpeedLimit := make([]uint32, ebgvNum)
	wf.ForWeights(func(eId da.Index, weight W, length uint32) {
		if weight == 0 {
			return
		}
		sLength := wf.GetSegmentLength(eId)
		eSpeedLimit[eId] = speedFromWeight(sLength, weight)
	})

	turnTable := makeTurnTable(turnMatrix, wf, g, rn, eSpeedLimit, vTurnTableIds)
	ebgWeights := make([]W, 0, ebgvNum)
	turnCosts := make([]uint16, 0, ebgvNum)
	ebgeId := da.Index(0)
	emPerm := make([]int, g.NumberOfEdges())
	// T(n) = \sum_{u in V} \sum_{v in edges(u,v)} outDeg(u)*outDeg(v) = O(n). if we let inDeg(v)=outDeg(v)=O(1) (for any vertex v) like in road networks.
	g.ForVertices(func(_ da.Vertex, u da.Index) {
		g.ForOutEdgesOf(u, func(ueId, v da.Index, i uint8) {

			uc := rn.GetSegmentTailCoord(ueId)
			vc := rn.GetSegmentHeadCoord(ueId)
			mpLat := (uc.GetLat() + vc.GetLat()) / 2
			mpLon := (uc.GetLon() + vc.GetLon()) / 2
			ebgVertices[ueId] = da.NewVertex(mpLat, mpLon, ueId)

			g.ForOutEdgesOfWithTurn(v, i, func(veId, w da.Index, j, _ uint8) {
				turnTableId := vTurnTableIds[v] + da.Index(i)*g.GetOutDegree(v) + da.Index(j)
				tc := turnTable[turnTableId] // turn costs
				eWeight := wf.GetWeight(ueId) + W(tc)

				if tc == util.TurnCostForbidden {
					return
				}

				ebgEdgeList = append(ebgEdgeList, ebgEdge{ueId, veId})
				ebgWeights = append(ebgWeights, eWeight)
				turnCosts = append(turnCosts, tc)

				ebgeId++
			})

			emPerm[ueId] = int(ueId)
			ebgMapping[ueId] = uint64(u) | (uint64(v) << 32)
		})
	})

	// build the ebgVertices[w].firstOut, ebgVertices[w].firstIn, ebgHeads, ebgTails like in counting sort
	// we need to store the edges (u,v) in ebgHeads sorted by tail vertex u. and store the reversed edges (v,u) in ebgTails sorted by head vertex v.
	// we can do this by counting sort, sort by tail vertex u for ebgHeads and ebgVertices[w].firstOut.
	// sort by head vertex v for ebgTails and ebgVertices[w].firstIn.
	// count the frequency of each tail vertex u & count frequency of head vertex v.
	// for example:
	// edges (u,v):  [(1,2), (3,8), (2,4), (1,3), (2,5), (1,4)]
	// build the frequency/occurence table of  each tail vertex u: [(1: 3), (2: 2), (3: 1)]
	// build the prefix-sum: [(1: 3), (2: 5), (3: 6)]
	// place the edges sorted by u from right to left of edges array:
	// place the edge (1, 4) to index 3-1=2, decrement (1: 3) -> (1: 2)
	// place the edge (2, 5) to index 5-1=4, decrement (2: 5) -> (2: 4)
	// place the edge (1, 3) to index 2-1=1, decrement (1: 2) -> (1: 1)
	// etc...
	// final edges array (u,v) sorted by its tail vertex u: [(1, 2), (1, 3), (1, 4), (2, 4), (2, 5), (3, 8)]
	for _, e := range ebgEdgeList {
		ebgVertices[e.u].IncrementFirstOut()
		ebgVertices[e.v].IncrementFirstIn()
	}

	m := da.Index(len(ebgEdgeList))

	// prefix-sum the vertices[v].firstOut and vertices[v].firstIn offset
	for v := da.Index(1); v < ebgvNum; v++ {
		ebgVertices[v].AddFirstOut(ebgVertices[v-1].GetFirstOut())
		ebgVertices[v].AddFirstIn(ebgVertices[v-1].GetFirstIn())
	}

	// dummy last edge-based graph ebg vertex
	ebgVertices[ebgvNum] = da.NewVertex(0, 0, ebgvNum)
	ebgVertices[ebgvNum].SetFirstOut(m)
	ebgVertices[ebgvNum].SetFirstIn(m)

	ebgHeads := make([]da.Index, m)
	ebgTails := make([]da.Index, m)
	ebgEntryPoints := make([]uint8, m)
	ebgExitPoints := make([]uint8, m)

	eids := make([]da.Index, m)
	reids := make([]da.Index, m)

	sortedWeights := make([]W, m)
	sortedTurnCosts := make([]uint16, m)

	// place the edges (u,v) from right to left by its prefix-sum result sorted by tail vertex u.
	// place the reversed edges (v,u) from right to left by its prefix-sum result sorted by head vertex v.
	for i := int(m) - 1; i >= 0; i-- {
		e := ebgEdgeList[i]
		ebgVertices[e.u].DecrementFirstOut()
		ebgVertices[e.v].DecrementFirstIn()
		outEdgeID := ebgVertices[e.u].GetFirstOut()
		inEdgeID := ebgVertices[e.v].GetFirstIn()
		ebgHeads[outEdgeID] = e.v
		ebgTails[inEdgeID] = e.u
		eids[i] = outEdgeID
		reids[i] = inEdgeID
		sortedWeights[outEdgeID] = ebgWeights[i]
		sortedTurnCosts[outEdgeID] = turnCosts[i]
	}

	for i := 0; i < int(m); i++ {
		e := ebgEdgeList[i]
		off := eids[i]
		revOff := reids[i]

		exitPoint := uint8(off - ebgVertices[e.u].GetFirstOut())
		entryPoint := uint8(revOff - ebgVertices[e.v].GetFirstIn())

		ebgEntryPoints[off] = entryPoint
		ebgExitPoints[revOff] = exitPoint
	}

	ebg := da.NewGraph(ebgVertices, ebgHeads, ebgTails, true, ebgEntryPoints, ebgExitPoints)

	sl := wf.GetSegmentLengths()
	sl = append(sl, 0) // dummy last ebg vertex segment length
	segDur := wf.GetSegmentDurations()
	segDur = append(segDur, 0)
	ebgWf := met.NewTimeCostFunction(true, sortedWeights, sl, segDur)
	ebgWf.SetTurnCosts(sortedTurnCosts)

	rn.ApplySegmentsPermutation(emPerm)

	// dummy last ebg vertex annotation
	rn.AppendSegmentData(-1, 1, 1, 0, pkg.INVALID_HIGHWAY, pkg.INVALID_HIGHWAY, 0, da.NewEmptyTurnLanesData())
	rn.SetSegmentBit(ebgvNum, 0, false)

	return ebg, ebgWf, ebgMapping
}

// makeTurnTable builds turn costs using edge speeds converted to meters per second.
// only for road network OpenStreetMap input file.
func makeTurnTable[W util.Numeric](
	turnMatrix []pkg.TurnType, wf *met.TimeFunction[W],
	g *da.Graph, rn *da.RoadNetworkDataContainer, eSpeedLimit []uint32,
	vTurnTableIds []da.Index,
) []uint16 {
	mapTurnCosts := viper.GetStringMap("turncosts")
	turnTypesCost := make([]float64, 6)
	for turnTypeStr, cost := range mapTurnCosts {
		if turnTypeStr == "traffic_light" || turnTypeStr == "max_turn_cost_based_on_angle_between_edges" {
			continue
		}
		turnType := getTurnTableId(turnTypeStr)
		switch v := cost.(type) {
		case int:
			turnTypesCost[turnType] = float64(v)
		case float64:
			turnTypesCost[turnType] = float64(v)
		default:
			panic("unsupported type")
		}
	}

	trafficLightPenalty := viper.GetFloat64("turncosts.traffic_light") // in seconds
	turnCostByAngleThreshold := viper.GetFloat64("turncosts.max_turn_cost_based_on_angle_between_edges")

	turnTypesCost[pkg.NONE] = 0
	turnTypesCost[pkg.NO_ENTRY] = math.Inf(1)

	n := len(turnMatrix)
	turnTableSeconds := make([]float64, n)
	for id := 0; id < n; id++ {
		turnType := turnMatrix[id]
		turnTableSeconds[id] += turnTypesCost[turnType]
	}

	minResolution := g.GetMinResolution()

	// T(n) = \sum_{u in V} \sum_{v in edges(u,v)} outDeg(u)*outDeg(v) = O(n). if we let inDeg(v)=outDeg(v)=O(1) (for any vertex v) like in road networks.
	g.ForOutEdges(func(_ uint8, v, u da.Index, i uint8, percentage float64, eIdFrom da.Index) {

		vLimitFrom := float64(eSpeedLimit[eIdFrom])
		g.ForOutEdgesOfWithTurn(v, i, func(eIdTo, w da.Index, j, _ uint8) {
			turnTableId := vTurnTableIds[v] + da.Index(i)*g.GetOutDegree(v) + da.Index(j)
			turnType := turnMatrix[turnTableId]
			ftl := rn.IsSegmentFlagBitOn(eIdFrom, da.FlagContainsTrafficLight)
			ttl := rn.IsSegmentFlagBitOn(eIdTo, da.FlagContainsTrafficLight)

			containsTrafficLight := ftl || ttl
			if containsTrafficLight {
				turnTableSeconds[turnTableId] += trafficLightPenalty
			}
			vLimitTo := float64(eSpeedLimit[eIdTo])
			currentTurnCost := turnTableSeconds[turnTableId]

			tail := g.GetVertex(v)
			prev := GetPrevPoint(eIdFrom, tail.GetCoordinate(), 15, rn)
			head := GetHeadPoint(eIdTo, tail.GetCoordinate(), 15, rn)
			prevInitialBearing := geo.ComputeInitialBearing(prev.GetLat(), prev.GetLon(), tail.GetLat(),
				tail.GetLon())
			relBearing := geo.ComputeRelativeBearing(tail.GetLat(), tail.GetLon(), head.GetLat(),
				head.GetLon(), prevInitialBearing)
			absRelativeBearing := math.Abs(relBearing)
			turnAngleDeg := util.RadiansToDegree(absRelativeBearing)

			l := util.DistanceToMeters(wf.GetSegmentLength(eIdFrom))
			lPrime := util.DistanceToMeters(wf.GetSegmentLength(eIdTo))
			turningSpeed := pkg.CalcTurningSpeed(l, lPrime, minResolution, turnAngleDeg)

			turnSign := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon(), prevInitialBearing)

			if turnType == pkg.NO_ENTRY || math.IsInf(currentTurnCost, 1) || (turnSign == da.CONTINUE_ON_STREET && !containsTrafficLight) {
				// gak ada turn penalty (pkg.NewTurnRest())
				// in this section 3.2 Turning Speed: https://ae.iti.kit.edu/download/turn_ch.pdf model
				// we assume deceleration a_{dec} from v down to max_v at the latest point possible and while traversing edge e′ we assume immediate start of acceleration a_{acc} from max_v to v′.
				// if the turnSign is CONTINUE, the possible scenario we decelerate from v down to max_v (turning speed) at the latest point possible in the intersection is when this intersection has a traffic light.
				return
			}

			// kalau u-turn jangan diskip, karena turning speed ~0 -> turn cost nya tinggi
			tcByAngle := pkg.CalcTurningCost(turningSpeed, vLimitFrom, vLimitTo)
			tcByAngle = min(tcByAngle, turnCostByAngleThreshold)
			turnTableSeconds[turnTableId] += tcByAngle
		})
	})

	turnTable := make([]uint16, n)
	for i, seconds := range turnTableSeconds {
		turnTable[i] = util.TurnCostCentiSeconds(seconds, math.IsInf(seconds, 1))
	}
	return turnTable
}

func GetPrevPoint(segmentId da.Index, tailCoord da.Coordinate, atLeastDist float64, rn *da.RoadNetworkDataContainer) da.Coordinate {

	curved := rn.IsCurved(segmentId)
	n := rn.GetSegmentGeometryLength(segmentId)
	if !curved || n == 2 {
		return rn.GetSegmentGeometryPoint(segmentId, 0)
	}

	v := rn.GetSegmentGeometryPoint(segmentId, int(n-2))
	for i := int(n - 2); int(i) >= 0; i-- {
		p := rn.GetSegmentGeometryPoint(segmentId, int(i))
		dist := geo.CalculateEuclideanDistMercatorProj(tailCoord.GetLat(), tailCoord.GetLon(), p.GetLat(), p.GetLon())
		if util.Ge(dist, atLeastDist) {
			v = p
			break
		}
	}

	return v
}

func GetHeadPoint(eId da.Index, tailCoord da.Coordinate, atLeastDist float64, rn *da.RoadNetworkDataContainer) da.Coordinate {

	curved := rn.IsCurved(eId)
	n := rn.GetSegmentGeometryLength(eId)
	if !curved || n == 2 {
		return rn.GetSegmentGeometryPoint(eId, 1)
	}

	v := rn.GetSegmentGeometryPoint(eId, 1)
	for i := 1; i < int(n); i++ {
		p := rn.GetSegmentGeometryPoint(eId, i)
		dist := geo.CalculateEuclideanDistMercatorProj(tailCoord.GetLat(), tailCoord.GetLon(), p.GetLat(), p.GetLon())
		if util.Ge(dist, atLeastDist) {
			v = p
			break
		}
	}

	return v
}

func getTurnTableId(turnTypeStr string) pkg.TurnType {
	var turnType pkg.TurnType
	switch turnTypeStr {
	case "left_turn":
		turnType = pkg.LEFT_TURN
	case "right_turn":
		turnType = pkg.RIGHT_TURN
	case "straight_on":
		turnType = pkg.STRAIGHT_ON
	case "u_turn":
		turnType = pkg.U_TURN
	case strNoEntry:
		turnType = pkg.NO_ENTRY
	case strNone:
		turnType = pkg.NONE
	default:
		panic("unsupported turn type")
	}
	return turnType
}

type ebgEdge struct {
	u, v da.Index
}
