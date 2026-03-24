package osmparser

import (
	"context"
	"io"
	"math"
	"os"
	"strconv"
	"strings"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
	"go.uber.org/zap"
)

type node struct {
	id    int64
	coord NodeCoord
}

type NodeCoord struct {
	lat float64
	lon float64
}

func NewNodeCoord(lat, lon float64) NodeCoord {
	return NodeCoord{lat, lon}
}

type restriction struct {
	via             da.Index
	to              int64
	turnRestriction TurnRestriction
}

type osmWay struct {
	nodes  []da.Index
	oneWay bool
	hwTag  string
}

type OsmParser struct {
	wayNodeMap         map[int64]NodeType
	relationMemberMap  map[int64]struct{}
	acceptedNodeMap    map[int64]NodeCoord
	barrierNodes       map[int64]bool
	nodeTag            map[int64]map[int]int
	tagStringIdMap     util.IDMap
	nodeIDMap          map[int64]da.Index
	nodeToOsmId        map[da.Index]int64
	maxNodeID          int64
	restrictions       map[int64][]restriction // wayId -> list of restrictions
	ways               map[int64]osmWay
	trafficEdges       []da.Index
	osmWayDefaultSpeed map[int64]float64
	bb                 *da.BoundingBox
}

func NewOSMParserV2() *OsmParser {
	return &OsmParser{
		wayNodeMap:         make(map[int64]NodeType),
		relationMemberMap:  make(map[int64]struct{}),
		acceptedNodeMap:    make(map[int64]NodeCoord),
		barrierNodes:       make(map[int64]bool),
		nodeTag:            make(map[int64]map[int]int),
		tagStringIdMap:     util.NewIdMap(),
		nodeIDMap:          make(map[int64]da.Index),
		nodeToOsmId:        make(map[da.Index]int64),
		trafficEdges:       make([]da.Index, 0),
		osmWayDefaultSpeed: make(map[int64]float64),
		bb:                 da.NewBoundingBoxEmpty(),
	}
}

func (o *OsmParser) SetAcceptedNodeMap(acceptedNodeMap map[int64]NodeCoord) {
	o.acceptedNodeMap = acceptedNodeMap
}

func (o *OsmParser) SetNodeToOsmId(nodeToOsmId map[da.Index]int64) {
	o.nodeToOsmId = nodeToOsmId
}

func (o *OsmParser) GetTagStringIdMap() util.IDMap {
	return o.tagStringIdMap
}

func (p *OsmParser) Parse(mapFile string, logger *zap.Logger, useMaxSpeed bool) (*da.Graph, error) {

	f, err := os.Open(mapFile)

	if err != nil {
		return nil, err
	}
	defer f.Close()

	restrictions := make(map[int64][]struct {
		via             int64
		turnRestriction TurnRestriction
		to              int64
	})
	scanner := osmpbf.New(context.Background(), f, 0)
	// must not be parallel
	scannedWays := 0

	logger.Sugar().Infof("parsing openstreetmap .pbf file......")

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

				scannedWays++

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
	graphStorage := da.NewGraphStorage()

	edgeSet := make(map[da.Index]map[da.Index]struct{})

	_, err = f.Seek(0, io.SeekStart)
	if err != nil {
		return nil, err
	}
	scanner = osmpbf.New(context.Background(), f, 0)
	//must not be parallel
	defer scanner.Close()

	scannedEdges := make([]Edge, 0)
	p.ways = make(map[int64]osmWay, scannedWays)

	streetDirection := make(map[int64][2]bool)

	countWays := 0
	countNodes := 0

	logger.Sugar().Infof("processing openstreetmap .pbf file: 0%.... ")
	outputEvery := 5
	pg := make([]bool, 101)

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

				progress := int(((float64(countWays) + 1) / float64(scannedWays)) * 100)
				if progress > 0 && progress%outputEvery == 0 && !pg[progress] {
					pg[progress] = true
					logger.Sugar().Infof("processing openstreetmap .pbf file: %v %% .....  ", int(progress))
				}

				countWays++

				hwTag, err := p.processWay(way, graphStorage, streetDirection, edgeSet, &scannedEdges, useMaxSpeed)
				if err != nil {
					continue
				}

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
				wNodes := make([]da.Index, 0, len(way.Nodes))
				for _, node := range way.Nodes {
					nodeID := p.nodeIDMap[int64(node.ID)]
					wNodes = append(wNodes, nodeID)
				}
				p.ways[int64(way.ID)] = osmWay{
					nodes:  wNodes,
					oneWay: wayExtraInfoData.oneWay,
					hwTag:  hwTag,
				}
			}
		case osm.TypeNode:
			{
				countNodes++
				node := o.(*osm.Node)

				p.maxNodeID = max(p.maxNodeID, int64(node.ID))

				if _, ok := p.wayNodeMap[int64(node.ID)]; ok {
					p.acceptedNodeMap[int64(node.ID)] = NodeCoord{
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
			graphStorage.SetTrafficLight(nodeIDX, true)
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

	logger.Sugar().Infof("building road network graph.... ")

	graph := p.BuildGraph(scannedEdges, graphStorage, uint32(len(p.nodeIDMap)), false)
	graph.SetGraphStorage(graphStorage)
	graph.SetBoundingBox(p.bb)

	logger.Sugar().Infof("number of vertices: %v\n", graph.NumberOfVertices())
	logger.Sugar().Infof("number of edges: %v\n", graph.NumberOfEdges())

	return graph, nil
}

type wayExtraInfo struct {
	oneWay  bool
	forward bool
}

func (p *OsmParser) processWay(way *osm.Way, graphStorage *da.GraphStorage,
	streetDirection map[int64][2]bool, edgeSet map[da.Index]map[da.Index]struct{},
	scannedEdges *[]Edge, useMaxSpeed bool) (string, error) {
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
				if !useMaxSpeed {
					highwayTypeSpeed = roadTypeSpeed(tag.Value)
				} else {
					highwayTypeSpeed = roadTypeMaxSpeedOsm(tag.Value) * pkg.NERF_MAXSPEED_OSM
				}

				if strings.Contains(tag.Value, "link") {
					tempMap[ROAD_CLASS_LINK] = tag.Value
				} else {
					tempMap[ROAD_CLASS] = tag.Value
				}
				p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])

			}
		case "lanes":
			{
				tempMap[LANES] = tag.Value
			}
		case "maxspeed":
			{
				if useMaxSpeed {
					if strings.Contains(tag.Value, "mph") {

						currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " mph", "", -1), 64)
						if err != nil {
							return "", err
						}
						maxSpeed = currSpeed * 1.60934
					} else if strings.Contains(tag.Value, "km/h") {
						currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " km/h", "", -1), 64)
						if err != nil {
							return "", err
						}
						maxSpeed = currSpeed
					} else if strings.Contains(tag.Value, "knots") {
						currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " knots", "", -1), 64)
						if err != nil {
							return "", err
						}
						maxSpeed = currSpeed * 1.852
					} else {
						// without unit
						// dont use this
					}
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

	hwTag := tempMap[ROAD_CLASS]
	return hwTag, nil
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

func (p *OsmParser) processSegment(segment []node, tempMap map[string]string, speed float64, graphStorage *da.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[da.Index]map[da.Index]struct{}, scannedEdges *[]Edge, id int64) {

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

func (p *OsmParser) processSegment2(segment []node, tempMap map[string]string, speed float64, graphStorage *da.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[da.Index]map[da.Index]struct{}, scannedEdges *[]Edge, id int64,
) {
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
	p.acceptedNodeMap[newMaxID] = NodeCoord{
		lat: nodeData.coord.lat,
		lon: nodeData.coord.lon,
	}
	p.maxNodeID++
	return node{
		id: newMaxID,
		coord: NodeCoord{
			lat: nodeData.coord.lat,
			lon: nodeData.coord.lon,
		},
	}
}

func (p *OsmParser) addEdge(segment []node, tempMap map[string]string, speed float64, graphStorage *da.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[da.Index]map[da.Index]struct{}, scannedEdges *[]Edge, id int64) {
	from := segment[0]

	to := segment[len(segment)-1]

	if from == to {
		return
	}
	maxLonft := math.Max(from.coord.lon, to.coord.lon)
	maxLatft := math.Max(from.coord.lat, to.coord.lat)

	minLonft := math.Min(from.coord.lon, to.coord.lon)
	minLatft := math.Min(from.coord.lat, to.coord.lat)

	p.bb.SetMaxLat(math.Max(p.bb.GetMaxLat(), maxLatft))
	p.bb.SetMaxLon(math.Max(p.bb.GetMaxLon(), maxLonft))
	p.bb.SetMinLat(math.Min(p.bb.GetMinLat(), minLatft))
	p.bb.SetMinLon(math.Min(p.bb.GetMinLon(), minLonft))

	if _, ok := p.nodeIDMap[from.id]; !ok {
		p.nodeIDMap[from.id] = da.Index(len(p.nodeIDMap))
		p.nodeToOsmId[p.nodeIDMap[from.id]] = from.id
	}
	if _, ok := p.nodeIDMap[to.id]; !ok {
		p.nodeIDMap[to.id] = da.Index(len(p.nodeIDMap))
		p.nodeToOsmId[p.nodeIDMap[to.id]] = to.id
	}

	edgePoints := make([]da.Coordinate, 0)
	distance := 0.0
	for i := 0; i < len(segment); i++ {
		if p.nodeTag[int64(segment[i].id)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] == 1 {

			p.trafficEdges = append(p.trafficEdges, da.Index(len(*scannedEdges)))
		}
		edgePoints = append(edgePoints, da.NewCoordinate(
			segment[i].coord.lat,
			segment[i].coord.lon,
		))
		if i > 0 {
			distance += geo.CalculateHaversineDistance(segment[i-1].coord.lat, segment[i-1].coord.lon, segment[i].coord.lat, segment[i].coord.lon)
		}
	}

	geoEdgePoints := geo.RamerDouglasPeucker(da.NewGeoCoordinates(edgePoints)) // simplify edge geometry
	simplifiedEdgePoints := make([]da.Coordinate, len(geoEdgePoints))
	simplifiedDistance := 0.0
	for i, coord := range geoEdgePoints {
		simplifiedEdgePoints[i] = da.NewCoordinate(coord.GetLat(), coord.GetLon())
		if i > 0 {
			simplifiedDistance += geo.CalculateHaversineDistance(geoEdgePoints[i-1].GetLat(), geoEdgePoints[i-1].GetLon(), geoEdgePoints[i].GetLat(), geoEdgePoints[i].GetLon())
		}
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

	distanceInMeter := util.KilometerToMeter(distance)
	simplifiedDistanceInMeter := util.KilometerToMeter(simplifiedDistance)

	travelTimeWeight := distanceInMeter / util.KMHToMMin(speed) // in minutes

	p.osmWayDefaultSpeed[id] = speed

	lanes, err := strconv.Atoi(tempMap[LANES])
	if err != nil {
		lanes = 1
	}

	fromNId := p.nodeIDMap[from.id]
	if _, ok := edgeSet[fromNId]; !ok {
		edgeSet[fromNId] = make(map[da.Index]struct{})
	}
	toNId := p.nodeIDMap[to.id]
	if _, ok := edgeSet[toNId]; !ok {
		edgeSet[toNId] = make(map[da.Index]struct{})
	}

	roadClass := tempMap[ROAD_CLASS]
	if roadClass == "" {
		roadClass = tempMap[ROAD_CLASS_LINK]
	}

	hwType := pkg.GetHighwayType(roadClass)

	if wayExtraInfoData.oneWay {
		if wayExtraInfoData.forward {

			if _, ok := edgeSet[fromNId][toNId]; ok {
				return
			}

			edgeSet[fromNId][toNId] = struct{}{}

			startPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendOsmNodePoints(simplifiedEdgePoints)
			endPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendMapEdgeInfo(da.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
				p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
				uint8(lanes),

				da.Index(startPointsIndex), da.Index(endPointsIndex),
				id,
			))

			graphStorage.SetRoundabout(da.Index(len(*scannedEdges)), isRoundabout)

			e := NewEdge(
				uint32(fromNId),
				uint32(toNId),
				travelTimeWeight,
				distanceInMeter,
				simplifiedDistanceInMeter,
				hwType,
			)

			e.SetFromOSMId(uint64(from.id))
			e.SetToOSMId(uint64(to.id))

			*scannedEdges = append(*scannedEdges, e)

		} else {

			if _, ok := edgeSet[toNId][fromNId]; ok {
				return
			}
			edgeSet[toNId][fromNId] = struct{}{}

			simplifiedEdgePoints = util.ReverseG(simplifiedEdgePoints)

			startPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendOsmNodePoints(simplifiedEdgePoints)
			endPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendMapEdgeInfo(da.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				p.tagStringIdMap.GetID(tempMap[ROAD_CLASS]),
				p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
				uint8(lanes),

				da.Index(startPointsIndex), da.Index(endPointsIndex),
				id,
			),
			)

			graphStorage.SetRoundabout(da.Index(len(*scannedEdges)), isRoundabout)
			e := NewEdge(
				uint32(toNId),
				uint32(fromNId),
				travelTimeWeight,
				distanceInMeter,
				simplifiedDistanceInMeter,
				hwType,
			)

			e.SetFromOSMId(uint64(to.id))
			e.SetToOSMId(uint64(from.id))

			*scannedEdges = append(*scannedEdges, e)
		}
	} else {
		if _, ok := edgeSet[fromNId][toNId]; ok {
			return
		}
		edgeSet[fromNId][toNId] = struct{}{}
		edgeSet[toNId][fromNId] = struct{}{}

		startPointsIndex := graphStorage.GetOsmNodePointsCount()

		graphStorage.AppendOsmNodePoints(simplifiedEdgePoints)
		endPointsIndex := graphStorage.GetOsmNodePointsCount()

		graphStorage.AppendMapEdgeInfo(da.NewEdgeExtraInfo(
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			p.tagStringIdMap.GetID(tempMap[ROAD_CLASS]),
			p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
			uint8(lanes),

			da.Index(startPointsIndex), da.Index(endPointsIndex),
			id,
		),
		)

		graphStorage.SetRoundabout(da.Index(len(*scannedEdges)), isRoundabout)

		e := NewEdge(
			uint32(fromNId),
			uint32(toNId),
			travelTimeWeight,
			distanceInMeter,
			simplifiedDistanceInMeter,
			hwType,
		)

		e.SetFromOSMId(uint64(from.id))
		e.SetToOSMId(uint64(to.id))

		*scannedEdges = append(*scannedEdges, e)

		graphStorage.AppendMapEdgeInfo(da.NewEdgeExtraInfo(
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			p.tagStringIdMap.GetID(tempMap[ROAD_CLASS]),
			p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK]),
			uint8(lanes),
			da.Index(endPointsIndex), da.Index(startPointsIndex),
			id,
		))

		graphStorage.SetRoundabout(da.Index(len(*scannedEdges)), isRoundabout)

		e = NewEdge(
			uint32(toNId),
			uint32(fromNId),
			travelTimeWeight,
			distanceInMeter,
			simplifiedDistanceInMeter,
			hwType,
		)

		e.SetFromOSMId(uint64(to.id))
		e.SetToOSMId(uint64(from.id))

		*scannedEdges = append(*scannedEdges, e)
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

// return osm way length in meter
func (p *OsmParser) GetRoadSpeed(osmWayId int64) float64 {
	return roadTypeSpeed(p.ways[osmWayId].hwTag)
}

// return osm way length in meter
func (p *OsmParser) GetOldRoadSpeed(osmWayId int64) float64 {
	return p.osmWayDefaultSpeed[osmWayId]
}

func (p *OsmParser) GetNodeIdMap() map[int64]da.Index {
	return p.nodeIDMap
}
