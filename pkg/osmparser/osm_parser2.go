package osmparser

import (
	"context"
	"io"
	"log"
	"os"
	"strconv"
	"strings"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
	"go.uber.org/zap"
)

type node struct {
	id    int64
	coord nodeCoord
}

type nodeCoord struct {
	lat float64
	lon float64
}

type restriction struct {
	via             datastructure.Index
	to              int64
	turnRestriction TurnRestriction
}

type osmWay struct {
	nodes  []datastructure.Index
	oneWay bool
}

type OsmParser struct {
	wayNodeMap        map[int64]NodeType
	relationMemberMap map[int64]struct{}
	acceptedNodeMap   map[int64]nodeCoord
	barrierNodes      map[int64]bool
	nodeTag           map[int64]map[int]int
	tagStringIdMap    util.IDMap
	nodeIDMap         map[int64]datastructure.Index
	nodeToOsmId       map[datastructure.Index]int64
	maxNodeID         int64
	restrictions      map[int64][]restriction // wayId -> list of restrictions
	ways              map[int64]osmWay
	trafficEdges      []datastructure.Index
}

func NewOSMParserV2() *OsmParser {
	return &OsmParser{
		wayNodeMap:        make(map[int64]NodeType),
		relationMemberMap: make(map[int64]struct{}),
		acceptedNodeMap:   make(map[int64]nodeCoord),
		barrierNodes:      make(map[int64]bool),
		nodeTag:           make(map[int64]map[int]int),
		tagStringIdMap:    util.NewIdMap(),
		nodeIDMap:         make(map[int64]datastructure.Index),
		nodeToOsmId:       make(map[datastructure.Index]int64),
		trafficEdges:      make([]datastructure.Index, 0),
	}
}
func (o *OsmParser) GetTagStringIdMap() util.IDMap {
	return o.tagStringIdMap
}

func (p *OsmParser) Parse(mapFile string, logger *zap.Logger) *datastructure.Graph {

	f, err := os.Open(mapFile)

	if err != nil {
		log.Fatal(err)
	}
	defer f.Close()

	restrictions := make(map[int64][]struct {
		via             int64
		turnRestriction TurnRestriction
		to              int64
	})
	scanner := osmpbf.New(context.Background(), f, 0)
	// must not be parallel
	countWays := 0
	for scanner.Scan() {
		o := scanner.Object()

		tipe := o.ObjectID().Type()

		switch tipe {
		case osm.TypeWay:
			{
				way := o.(*osm.Way)
				if len(way.Nodes) < 2 {
					continue
				}

				if !acceptOsmWay(way) {
					continue
				}
				if (countWays+1)%50000 == 0 {
					logger.Sugar().Infof("scanning openstreetmap ways: %d...", countWays+1)
				}
				countWays++

				for i, node := range way.Nodes {
					if _, ok := p.wayNodeMap[int64(node.ID)]; !ok {
						if i == 0 || i == len(way.Nodes)-1 {
							p.wayNodeMap[int64(node.ID)] = END_NODE
						} else {
							p.wayNodeMap[int64(node.ID)] = BETWEEN_NODE
						}
					} else {
						p.wayNodeMap[int64(node.ID)] = JUNCTION_NODE
					}
				}
			}
		case osm.TypeNode:
			{
			}
		case osm.TypeRelation:
			{
				relation := o.(*osm.Relation)
				for _, member := range relation.Members {
					if member.Type == osm.TypeRelation {
						continue
					}
				}

				if relation.Tags.Find("type") == "route" {
					for _, member := range relation.Members {
						p.relationMemberMap[member.Ref] = struct{}{}
					}
				}

				tagVal := relation.Tags.Find("restriction")
				if tagVal != "" {
					from := int64(0)
					via := int64(0)
					to := int64(0)
					// https://www.openstreetmap.org/api/0.6/relation/5710500
					for _, member := range relation.Members {
						if member.Role == "from" {
							from = member.Ref
						} else if member.Role == "to" {
							to = member.Ref
						} else {
							via = int64(member.Ref)
						}
					}
					rest := struct {
						via             int64
						turnRestriction TurnRestriction
						to              int64
					}{
						via:             via,
						to:              to,
						turnRestriction: parseTurnRestriction(tagVal),
					}
					restrictions[from] = append(restrictions[from], rest)
				}
			}
		}
	}
	scanner.Close()
	graphStorage := datastructure.NewGraphStorage()

	edgeSet := make(map[datastructure.Index]map[datastructure.Index]struct{})

	f.Seek(0, io.SeekStart)
	if err != nil {
		log.Fatal(err)
	}
	scanner = osmpbf.New(context.Background(), f, 0)
	//must not be parallel
	defer scanner.Close()

	scannedEdges := make([]Edge, 0)
	p.ways = make(map[int64]osmWay, countWays)

	streetDirection := make(map[int64][2]bool)

	countWays = 0
	countNodes := 0
	for scanner.Scan() {
		o := scanner.Object()

		tipe := o.ObjectID().Type()

		switch tipe {
		case osm.TypeWay:
			{
				way := o.(*osm.Way)
				if len(way.Nodes) < 2 {
					continue
				}

				if !acceptOsmWay(way) {
					continue
				}
				if (countWays+1)%50000 == 0 {
					logger.Sugar().Infof("processing openstreetmap ways: %d...", countWays+1)
				}
				countWays++

				p.processWay(way, graphStorage, streetDirection, edgeSet, &scannedEdges)

				wayExtraInfoData := wayExtraInfo{}
				okvf, okmvf, okvb, okmvb := getReversedOneWay(way)
				if val := way.Tags.Find("oneway"); val == "yes" || val == "-1" || okvf || okmvf || okvb || okmvb {
					wayExtraInfoData.oneWay = true
				}

				if way.Tags.Find("oneway") == "-1" || okvf || okmvf {
					// okvf / omvf = restricted/not allowed forward.
					wayExtraInfoData.forward = false

				} else {
					wayExtraInfoData.forward = true
				}
				wNodes := make([]datastructure.Index, 0, len(way.Nodes))
				for _, node := range way.Nodes {
					nodeID := p.nodeIDMap[int64(node.ID)]
					wNodes = append(wNodes, nodeID)
				}
				p.ways[int64(way.ID)] = osmWay{
					nodes:  wNodes,
					oneWay: wayExtraInfoData.oneWay,
				}
			}
		case osm.TypeNode:
			{

				if (countNodes+1)%50000 == 0 {
					logger.Sugar().Infof("processing openstreetmap nodes: %d...", countNodes+1)
				}
				countNodes++
				node := o.(*osm.Node)

				p.maxNodeID = max(p.maxNodeID, int64(node.ID))

				if _, ok := p.wayNodeMap[int64(node.ID)]; ok {
					p.acceptedNodeMap[int64(node.ID)] = nodeCoord{
						lat: node.Lat,
						lon: node.Lon,
					}
				}
				accessType := node.Tags.Find("access")
				barrierType := node.Tags.Find("barrier")

				if _, ok := acceptedBarrierType[barrierType]; ok && accessType == "no" && barrierType != "" {
					p.barrierNodes[int64(node.ID)] = true
				}

				for _, tag := range node.Tags {
					if strings.Contains(tag.Key, "created_by") ||
						strings.Contains(tag.Key, "source") ||
						strings.Contains(tag.Key, "note") ||
						strings.Contains(tag.Key, "fixme") {
						continue
					}
					tagID := p.tagStringIdMap.GetID(tag.Key)
					if _, ok := p.nodeTag[int64(node.ID)]; !ok {
						p.nodeTag[int64(node.ID)] = make(map[int]int)
					}
					p.nodeTag[int64(node.ID)][tagID] = p.tagStringIdMap.GetID(tag.Value)
					if strings.Contains(tag.Value, "traffic_signals") {
						p.nodeTag[int64(node.ID)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] = 1
					}
				}

			}
		case osm.TypeRelation:
			{

			}
		}
	}

	graphStorage.SetStreetDirection(streetDirection)
	graphStorage.SetTagStringIdMap(p.tagStringIdMap)

	for nodeID, nodeIDX := range p.nodeIDMap {
		if val, ok := p.nodeTag[int64(nodeID)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)]; ok && val == 1 {
			graphStorage.SetTrafficLight(nodeIDX)
		}
	}

	p.restrictions = make(map[int64][]restriction, len(restrictions))
	for key, val := range restrictions {
		savedRest := make([]restriction, len(val))
		for i := range val {
			savedRest[i] = restriction{
				via:             p.nodeIDMap[val[i].via],
				to:              val[i].to,
				turnRestriction: val[i].turnRestriction,
			}
		}
		p.restrictions[key] = savedRest
	}

	graph := p.BuildGraph(scannedEdges, graphStorage)
	graph.SetGraphStorage(graphStorage)

	// adjust travel time of edges that contain traffic light
	for _, edgeId := range p.trafficEdges {
		edge := graph.GetOutEdge(edgeId)
		edge.SetWeight(edge.GetWeight() + util.SecondsToMinutes(pkg.TRAFFIC_LIGHT_ADDITIONAL_WEIGHT_SECOND))
		edgeWayId := graph.GetOsmWayId(edgeId)

		tail := graph.GetTailOfOutedge(edgeId)
		backtrack := true
		visited := make(map[datastructure.Index]struct{})
		for backtrack {
			if _, already := visited[tail]; already {
				break
			}

			visited[tail] = struct{}{}
			backEdgeHaveSameWayId := false
			graph.ForInEdgesOfWithId(tail, func(e *datastructure.InEdge, id datastructure.Index) {
				if graph.GetOsmWayId(id) == edgeWayId {
					e.SetWeight(e.GetWeight() + util.SecondsToMinutes(pkg.TRAFFIC_LIGHT_ADDITIONAL_WEIGHT_SECOND))
					tail = e.GetTail()
					_, outEdge := graph.GetHeadOfInedgeWithOutEdge(e.GetEdgeId())
					outEdge.SetWeight(outEdge.GetWeight() + util.SecondsToMinutes(pkg.TRAFFIC_LIGHT_ADDITIONAL_WEIGHT_SECOND))
					backEdgeHaveSameWayId = true
					return
				}
			})
			if !backEdgeHaveSameWayId {
				backtrack = false
			}
		}
	}

	return graph
}

type wayExtraInfo struct {
	oneWay  bool
	forward bool
}

func (p *OsmParser) processWay(way *osm.Way, graphStorage *datastructure.GraphStorage,
	streetDirection map[int64][2]bool, edgeSet map[datastructure.Index]map[datastructure.Index]struct{}, scannedEdges *[]Edge) error {
	tempMap := make(map[string]string)
	name := way.Tags.Find("name")

	tempMap[STREET_NAME] = name

	refName := way.Tags.Find("ref")
	tempMap[STREET_REF] = refName

	maxSpeed := 0.0
	highwayTypeSpeed := 0.0

	wayExtraInfoData := wayExtraInfo{}
	okvf, okmvf, okvb, okmvb := getReversedOneWay(way)
	if val := way.Tags.Find("oneway"); val == "yes" || val == "-1" || okvf || okmvf || okvb || okmvb {
		wayExtraInfoData.oneWay = true
	}

	if way.Tags.Find("oneway") == "-1" || okvf || okmvf {
		// okvf / omvf = restricted/not allowed forward.
		wayExtraInfoData.forward = false

	} else {
		wayExtraInfoData.forward = true
	}

	if wayExtraInfoData.oneWay {
		if wayExtraInfoData.forward {
			streetDirection[int64(way.ID)] = [2]bool{true, false} // {forward, backward}
		} else {
			streetDirection[int64(way.ID)] = [2]bool{false, true}
		}
	} else {
		streetDirection[int64(way.ID)] = [2]bool{true, true}
	}

	for _, tag := range way.Tags {
		switch tag.Key {
		case "junction":
			{
				tempMap[JUNCTION] = tag.Value
			}
		case "highway":
			{
				highwayTypeSpeed = roadTypeMaxSpeed2(tag.Value)

				if strings.Contains(tag.Value, "link") {
					tempMap[ROAD_CLASS_LINK] = tag.Value
				} else {
					tempMap[ROAD_CLASS] = tag.Value
				}
			}
		case "lanes":
			{
				tempMap[LANES] = tag.Value
			}
		case "maxspeed":
			{
				if strings.Contains(tag.Value, "mph") {

					currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " mph", "", -1), 64)
					if err != nil {
						return err
					}
					maxSpeed = currSpeed * 1.60934
				} else if strings.Contains(tag.Value, "km/h") {
					currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " km/h", "", -1), 64)
					if err != nil {
						return err
					}
					maxSpeed = currSpeed
				} else if strings.Contains(tag.Value, "knots") {
					currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " knots", "", -1), 64)
					if err != nil {
						return err
					}
					maxSpeed = currSpeed * 1.852
				} else {
					// without unit
					// dont use this
				}
			}

		}
	}

	if maxSpeed == 0 {
		maxSpeed = highwayTypeSpeed
	}
	if maxSpeed == 0 {
		maxSpeed = 30
	}

	waySegment := []node{}
	for _, wayNode := range way.Nodes {
		nodeCoord := p.acceptedNodeMap[int64(wayNode.ID)]
		nodeData := node{
			id:    int64(wayNode.ID),
			coord: nodeCoord,
		}
		if p.isJunctionNode(int64(nodeData.id)) {

			waySegment = append(waySegment, nodeData)
			p.processSegment(waySegment, tempMap, maxSpeed, graphStorage, wayExtraInfoData,
				edgeSet, scannedEdges, int64(way.ID))
			waySegment = []node{}

			waySegment = append(waySegment, nodeData)

		} else {
			waySegment = append(waySegment, nodeData)
		}
	}
	if len(waySegment) > 1 {
		p.processSegment(waySegment, tempMap, maxSpeed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges,
			int64(way.ID))
	}

	return nil
}

func isRestricted(value string) bool {
	if value == "no" || value == "restricted" {
		return true
	}
	return false
}

func getReversedOneWay(way *osm.Way) (bool, bool, bool, bool) {
	vehicleForward := way.Tags.Find("vehicle:forward")
	motorVehicleForward := way.Tags.Find("motor_vehicle:forward")
	vehicleBackward := way.Tags.Find("vehicle:backward")
	motorVehicleBackward := way.Tags.Find("motor_vehicle:backward")
	return isRestricted(vehicleForward), isRestricted(motorVehicleForward), isRestricted(vehicleBackward), isRestricted(motorVehicleBackward)
}

func (p *OsmParser) processSegment(segment []node, tempMap map[string]string, speed float64, graphStorage *datastructure.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[datastructure.Index]map[datastructure.Index]struct{}, scannedEdges *[]Edge, id int64) {

	if len(segment) == 2 && segment[0].id == segment[1].id {
		// skip
		return
	} else if len(segment) > 2 && segment[0].id == segment[len(segment)-1].id {
		// loop
		p.processSegment2(segment[0:len(segment)-1], tempMap, speed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges, id)
		p.processSegment2(segment[len(segment)-2:], tempMap, speed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges, id)
	} else {
		p.processSegment2(segment, tempMap, speed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges, id)
	}
}

func (p *OsmParser) processSegment2(segment []node, tempMap map[string]string, speed float64, graphStorage *datastructure.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[datastructure.Index]map[datastructure.Index]struct{}, scannedEdges *[]Edge, id int64) {
	waySegment := []node{}
	for i := 0; i < len(segment); i++ {
		nodeData := segment[i]
		if _, ok := p.barrierNodes[int64(nodeData.id)]; ok {

			if len(waySegment) != 0 {
				// if current node is a barrier
				// add the barrier node and process the segment (add edge)
				waySegment = append(waySegment, nodeData)
				p.addEdge(waySegment, tempMap, speed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges, id)
				waySegment = []node{}
			}
			// copy the barrier node but with different id so that previous edge (with barrier) not connected with the new edge

			nodeData = p.copyNode(nodeData)
			waySegment = append(waySegment, nodeData)

		} else {
			waySegment = append(waySegment, nodeData)
		}
	}
	if len(waySegment) > 1 {
		p.addEdge(waySegment, tempMap, speed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges, id)
	}
}

func (p *OsmParser) copyNode(nodeData node) node {
	// use the same coordinate but different id & and the newID is not used
	newMaxID := p.maxNodeID + 1
	p.acceptedNodeMap[newMaxID] = nodeCoord{
		lat: nodeData.coord.lat,
		lon: nodeData.coord.lon,
	}
	p.maxNodeID++
	return node{
		id: newMaxID,
		coord: nodeCoord{
			lat: nodeData.coord.lat,
			lon: nodeData.coord.lon,
		},
	}
}

func (p *OsmParser) addEdge(segment []node, tempMap map[string]string, speed float64, graphStorage *datastructure.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[datastructure.Index]map[datastructure.Index]struct{}, scannedEdges *[]Edge, id int64) {
	from := segment[0]

	to := segment[len(segment)-1]

	if from == to {
		return
	}

	if _, ok := p.nodeIDMap[from.id]; !ok {
		p.nodeIDMap[from.id] = datastructure.Index(len(p.nodeIDMap))
		p.nodeToOsmId[p.nodeIDMap[from.id]] = from.id
	}
	if _, ok := p.nodeIDMap[to.id]; !ok {
		p.nodeIDMap[to.id] = datastructure.Index(len(p.nodeIDMap))
		p.nodeToOsmId[p.nodeIDMap[to.id]] = to.id
	}

	edgePoints := []datastructure.Coordinate{}
	distance := 0.0
	for i := 0; i < len(segment); i++ {
		if p.nodeTag[int64(segment[i].id)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] == 1 {
			p.trafficEdges = append(p.trafficEdges, datastructure.Index(len(*scannedEdges)))
		}
		edgePoints = append(edgePoints, datastructure.NewCoordinate(
			segment[i].coord.lat,
			segment[i].coord.lon,
		))
		if i > 0 {
			distance += geo.CalculateHaversineDistance(segment[i-1].coord.lat, segment[i-1].coord.lon, segment[i].coord.lat, segment[i].coord.lon)
		}
	}

	geoEdgePoints := geo.RamerDouglasPeucker(datastructure.NewGeoCoordinates(edgePoints)) // simplify edge geometry
	for i, coord := range geoEdgePoints {
		edgePoints[i] = datastructure.NewCoordinate(coord.GetLat(), coord.GetLon())
	}

	isRoundabout := false
	if val, ok := tempMap[JUNCTION]; ok {
		if val == "roundabout" {
			isRoundabout = true
		}
		if val == "circular" {
			isRoundabout = true
		}
	}

	distanceInMeter := distance * 1000
	etaWeight := distanceInMeter / (speed * 1000 / 60) // in minutes

	lanes, err := strconv.Atoi(tempMap[LANES])
	if err != nil {
		lanes = 1 // assume
	}

	if _, ok := edgeSet[p.nodeIDMap[from.id]]; !ok {
		edgeSet[p.nodeIDMap[from.id]] = make(map[datastructure.Index]struct{})
	}
	if _, ok := edgeSet[p.nodeIDMap[to.id]]; !ok {
		edgeSet[p.nodeIDMap[to.id]] = make(map[datastructure.Index]struct{})
	}

	if wayExtraInfoData.oneWay {
		if wayExtraInfoData.forward {

			if _, ok := edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]]; ok {
				return
			}

			edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]] = struct{}{}

			startPointsIndex := graphStorage.GetGlobalPointsCount()

			graphStorage.AppendGlobalPoints(edgePoints)
			endPointsIndex := graphStorage.GetGlobalPointsCount()

			graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
				uint8(lanes),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
				datastructure.Index(startPointsIndex), datastructure.Index(endPointsIndex),
				id,
			))

			graphStorage.SetRoundabout(datastructure.Index(len(*scannedEdges)), isRoundabout)

			*scannedEdges = append(*scannedEdges, NewEdge(
				uint32(p.nodeIDMap[from.id]),
				uint32(p.nodeIDMap[to.id]),
				etaWeight,
				distanceInMeter,
				uint32(len(*scannedEdges)),
			))

		} else {

			if _, ok := edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]]; ok {
				return
			}
			edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]] = struct{}{}

			edgePoints = util.ReverseG(edgePoints)

			startPointsIndex := graphStorage.GetGlobalPointsCount()

			graphStorage.AppendGlobalPoints(edgePoints)
			endPointsIndex := graphStorage.GetGlobalPointsCount()

			graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
				uint8(lanes),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
				datastructure.Index(startPointsIndex), datastructure.Index(endPointsIndex),
				id,
			),
			)

			graphStorage.SetRoundabout(datastructure.Index(len(*scannedEdges)), isRoundabout)

			*scannedEdges = append(*scannedEdges, NewEdge(
				uint32(p.nodeIDMap[to.id]),
				uint32(p.nodeIDMap[from.id]),
				etaWeight,
				distanceInMeter,
				uint32(len(*scannedEdges)),
			))
		}
	} else {
		if _, ok := edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]]; ok {
			return
		}
		edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]] = struct{}{}
		edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]] = struct{}{}

		startPointsIndex := graphStorage.GetGlobalPointsCount()

		graphStorage.AppendGlobalPoints(edgePoints)
		endPointsIndex := graphStorage.GetGlobalPointsCount()

		graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
			uint8(lanes),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
			datastructure.Index(startPointsIndex), datastructure.Index(endPointsIndex),
			id,
		),
		)

		graphStorage.SetRoundabout(datastructure.Index(len(*scannedEdges)), isRoundabout)

		*scannedEdges = append(*scannedEdges, NewEdge(
			uint32(p.nodeIDMap[from.id]),
			uint32(p.nodeIDMap[to.id]),
			etaWeight,
			distanceInMeter,
			uint32(len(*scannedEdges)),
		))

		graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
			uint8(lanes),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
			datastructure.Index(endPointsIndex), datastructure.Index(startPointsIndex),
			id,
		))

		graphStorage.SetRoundabout(datastructure.Index(len(*scannedEdges)), isRoundabout)

		*scannedEdges = append(*scannedEdges, NewEdge(
			uint32(p.nodeIDMap[to.id]),
			uint32(p.nodeIDMap[from.id]),
			etaWeight,
			distanceInMeter,
			uint32(len(*scannedEdges)),
		))

	}
}

func roadTypeMaxSpeed2(roadType string) float64 {
	switch roadType {
	case "motorway":
		return 100
	case "trunk":
		return 70
	case "primary":
		return 65
	case "secondary":
		return 60
	case "tertiary":
		return 50
	case "unclassified":
		return 40
	case "residential":
		return 30
	case "service":
		return 20
	case "motorway_link":
		return 70
	case "trunk_link":
		return 65
	case "primary_link":
		return 60
	case "secondary_link":
		return 50
	case "tertiary_link":
		return 40
	case "living_street":
		return 5
	case "road":
		return 20
	case "track":
		return 15
	case "motorroad":
		return 90
	default:
		return 30
	}
}

func (p *OsmParser) isJunctionNode(nodeID int64) bool {
	return p.wayNodeMap[int64(nodeID)] == JUNCTION_NODE
}

func acceptOsmWay(way *osm.Way) bool {
	highway := way.Tags.Find("highway")
	junction := way.Tags.Find("junction")
	if highway != "" {
		if _, ok := acceptedHighway[highway]; ok {
			return true
		}
	} else if junction != "" {
		return true
	}
	return false
}

func max(a, b int64) int64 {
	if a > b {
		return a
	}
	return b
}
