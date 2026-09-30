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

// todo: yg query pakai turn cost ganti pakai pendekatan OSRM aja https://github.com/Project-OSRM/osrm-backend/wiki/Graph-representation  atau
// atau edge-based model (sama aja) disini: https://drops.dagstuhl.de/storage/01oasics/oasics-vol085-atmos2020/OASIcs.ATMOS.2020.9/OASIcs.ATMOS.2020.9.pdf
// ntar query with turn cost bisa pakai kode multilevel_dijkstra_without_turn_cost.go kalau pakai edge-based model
// this compact model buat support query with turn cost (& turn restrictions) ribet bgt gokil
// biar gak sama kaya paten ini juga, CRP yg dijelasin disini pakai compact representation: https://patents.google.com/patent/US20130231862A1/en
// banyak yang diganti terutama driving direction and map matching. tapi harusnya gak susah..
// referensi lain buat bikin edge-based graph (or expanded graph whatever): https://i11www.iti.kit.edu/_media/teaching/theses/ba-zuendorf-19.pdf

/*
buildEdgeBasedGraph. build edge-based (or expanded) graph following:  https://github.com/Project-OSRM/osrm-backend/wiki/Graph-representation
another reference:  https://drops.dagstuhl.de/storage/01oasics/oasics-vol085-atmos2020/OASIcs.ATMOS.2020.9/OASIcs.ATMOS.2020.9.pdf
https://i11www.iti.kit.edu/_media/teaching/theses/ba-zuendorf-19.pdf
https://github.com/Project-OSRM/osrm-backend/blob/master/src/extractor/edge_based_graph_factory.cpp
parameter:
graph: node-based graph datastructure
weightFunction: weight (duration/travel time) of each edges of node-based graph

only for road network OpenStreetMap input file.
*/
func BuildEdgeBasedGraph[W util.RoutingNumber](g *da.Graph, wf *met.TimeFunction[W], segmentDataIds [][]da.Index,
	vTurnTableIds []da.Index, turnMatrix []pkg.TurnType, rn *da.RoadNetworkDataContainer) (*da.Graph, *met.TimeFunction[W]) {
	ebgvNum := da.Index(g.NumberOfEdges())
	ebgVertices := make([]da.Vertex, ebgvNum+1)
	ebgAdjList := make([][]da.Index, ebgvNum)
	ebgRevAdjList := make([][]da.Index, ebgvNum)
	ebgEntryPoints := make([][]da.Index, ebgvNum)
	ebgExitPoints := make([][]da.Index, ebgvNum)

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
	ebgvId := da.Index(0)
	ebgeId := da.Index(0)
	emPerm := make([]int, g.NumberOfEdges())
	// T(n) = \sum_{u in V} \sum_{v in edges(u,v)} outDeg(u)*outDeg(v) = O(n). if we let inDeg(v)=outDeg(v)=O(1) (for any vertex v) like in road networks.
	g.ForVertices(func(_ da.Vertex, u da.Index) {
		g.ForOutEdgesOf(u, func(ueId, v, i da.Index) {
			uep := g.GetExitOrder(u, ueId)
			uoemId := segmentDataIds[u][uep]
			uc := rn.GetSegmentTailCoord(uoemId)
			vc := rn.GetSegmentHeadCoord(uoemId)
			mpLat := (uc.GetLat() + vc.GetLat()) / 2
			mpLon := (uc.GetLon() + vc.GetLon()) / 2
			ebgVertices[ebgvId] = da.NewVertex(mpLat, mpLon, ueId)

			g.ForOutEdgesOfWithTurn(v, i, func(veId, w, j, _ da.Index) {
				turnTableId := vTurnTableIds[v] + da.Index(i)*g.GetOutDegree(v) + da.Index(j)
				tc := turnTable[turnTableId] // turn costs
				eWeight := wf.GetWeight(ueId) + W(tc)

				if tc == util.TurnCostForbidden {
					return
				}

				uvEntryPoint := da.Index(len(ebgRevAdjList[veId]))
				uvExitPoint := da.Index(len(ebgAdjList[ueId]))

				ebgAdjList[ueId] = append(ebgAdjList[ueId], veId)
				ebgRevAdjList[veId] = append(ebgRevAdjList[veId], ueId)

				ebgEntryPoints[ueId] = append(ebgEntryPoints[ueId], uvEntryPoint)
				ebgExitPoints[veId] = append(ebgExitPoints[veId], uvExitPoint)
				ebgWeights = append(ebgWeights, eWeight)

				ebgeId++
			})

			emPerm[ebgvId] = int(uoemId)
			ebgvId++
		})
	})

	oEdgeOffset := da.Index(0)
	iEdgeOffset := da.Index(0)
	for u := 0; u < int(ebgvNum); u++ {
		ebgVertices[u].SetFirstOut(oEdgeOffset) // index of the first outEdge of vertex i in the flattened outEdges array
		ebgVertices[u].SetFirstIn(iEdgeOffset)
		oEdgeOffset += da.Index(len(ebgAdjList[u]))
		iEdgeOffset += da.Index(len(ebgRevAdjList[u]))
	}

	// dummy last edge-based graph ebg vertex
	ebgVertices[ebgvNum] = da.NewVertex(0, 0, ebgvNum)
	ebgVertices[ebgvNum].SetFirstOut(oEdgeOffset)
	ebgVertices[ebgvNum].SetFirstIn(iEdgeOffset)

	ebgHeads := util.Flatten(ebgAdjList)
	ebgTails := util.Flatten(ebgRevAdjList)
	ebgEntryPointsFlat := util.Flatten(ebgEntryPoints)
	ebgExitPointsFlat := util.Flatten(ebgExitPoints)

	ebg := da.NewGraph(ebgVertices, ebgHeads, ebgTails, true, ebgEntryPointsFlat, ebgExitPointsFlat)

	sl := wf.GetSegmentLengths()
	sl = append(sl, 0) // dummy last ebg vertex segment length
	segDur := wf.GetSegmentDurations()
	segDur = append(segDur, 0)
	ebgWf := met.NewTimeCostFunction(true, ebgWeights, sl, segDur)

	rn.ApplySegmentsPermutation(emPerm)

	// dummy last ebg vertex annotation
	rn.AppendSegmentData(-1, 1, 1, 0, pkg.INVALID_HIGHWAY, pkg.INVALID_HIGHWAY, 0, da.NewEmptyTurnLanesData())
	rn.SetSegmentBit(ebgvNum, 0, false)

	return ebg, ebgWf
}

// makeTurnTable builds turn costs using edge speeds converted to meters per second.
// only for road network OpenStreetMap input file.
func makeTurnTable[W util.RoutingNumber](
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

	// T(n) = \sum_{u in V} outDeg(u)*outDeg(v) = O(n). if we let inDeg(v)=outDeg(v)=O(1) (for any vertex v) like in road networks.
	g.ForOutEdges(func(_, v, u, i da.Index, percentage float64, eIdFrom da.Index) {

		vLimitFrom := util.SpeedToMetersPerSecond(eSpeedLimit[eIdFrom])
		g.ForOutEdgesOfWithTurn(v, i, func(eIdTo, w da.Index, j, _ da.Index) {
			turnTableId := vTurnTableIds[v] + da.Index(i)*g.GetOutDegree(v) + da.Index(j)
			turnType := turnMatrix[turnTableId]
			ftl := rn.IsSegmentFlagBitOn(eIdFrom, da.FlagContainsTrafficLight)
			ttl := rn.IsSegmentFlagBitOn(eIdTo, da.FlagContainsTrafficLight)

			containsTrafficLight := ftl || ttl
			if containsTrafficLight {
				turnTableSeconds[turnTableId] += trafficLightPenalty
			}

			fSegmentName := rn.GetStreetName(eIdFrom)
			tSegmentName := rn.GetStreetName(eIdTo)

			if !isTurnCostByAngleBetweenEdgesAllowed(turnType) || isSameName(fSegmentName, tSegmentName) {
				// skip kalau turnType bukan LEFT_TURN dan bukan RIGHT_TURN. skip juga kalau gak pindah jalan.
				// soale kalau di jalan tol (example: https://www.openstreetmap.org/way/1301675709#map=15/-7.63705/110.66151)
				// sering dipisah jadi beberaapa osm ways -> yang mana jadi beberapa graph edges. padahal masih bisa ngebut dan gak perlu turn costs di jalan tol??...
				return
			}

			vLimitTo := util.SpeedToMetersPerSecond(eSpeedLimit[eIdTo])
			currentTurnCost := turnTableSeconds[turnTableId]

			prev := g.GetVertex(u)
			tail := g.GetVertex(v)
			head := g.GetVertex(w)

			prevInitialBearing := geo.ComputeInitialBearing(prev.GetLat(), prev.GetLon(), tail.GetLat(),
				tail.GetLon())
			relBearing := geo.ComputeRelativeBearing(tail.GetLat(), tail.GetLon(), head.GetLat(),
				head.GetLon(), prevInitialBearing)
			absRelativeBearing := math.Abs(relBearing)
			turnAngleDeg := util.RadiansToDegree(absRelativeBearing)

			l := util.DistanceToMeters(wf.GetSegmentLength(eIdFrom))
			lPrime := util.DistanceToMeters(wf.GetSegmentLength(eIdTo))
			turningSpeed := pkg.CalcTurningSpeed(l, lPrime, minResolution, turnAngleDeg)

			if util.Eq(turningSpeed, 0) || turnType == pkg.NO_ENTRY || math.IsInf(currentTurnCost, 1) {
				// gak ada turn penalty (pkg.NewTurnRest())
				return
			}

			tcByAngle := pkg.CalcTurningCost(turningSpeed, vLimitFrom, vLimitTo)
			tcByAngle = min(tcByAngle, turnCostByAngleThreshold)
			turnTableSeconds[turnTableId] += tcByAngle
		})
	})

	turnTable := make([]uint16, n)
	for i, seconds := range turnTableSeconds {
		turnTable[i] = util.QuantizeTurnCost(seconds, math.IsInf(seconds, 1))
	}
	return turnTable
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
	case "no_entry":
		turnType = pkg.NO_ENTRY
	case "none":
		turnType = pkg.NONE
	default:
		panic("unsupported turn type")
	}
	return turnType
}

func isTurnCostByAngleBetweenEdgesAllowed(turnType pkg.TurnType) bool {
	return turnType == pkg.LEFT_TURN || turnType == pkg.RIGHT_TURN
}

func isSameName(name1, name2 string) bool {
	if name1 == "" || name2 == "" {
		// seringkali di osm, nama street kosong "" (terutama di residential/living street/tertiary osm ways), better dianggap false
		// biar kalo belok masih ada turn instructionnya
		// contoh tertiary osm way yang gak ada namanya:  https://www.openstreetmap.org/way/332233207#map=17/-7.555473/110.769728
		return false
	}
	return name1 == name2
}
