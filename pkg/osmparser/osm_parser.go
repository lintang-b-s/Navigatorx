// Package osmparser provides tools for parsing OpenStreetMap (OSM) .osm.pbf data and building the road network graphs.
package osmparser

import (
	"context"
	"fmt"
	"io"
	"math"
	"os"
	"strings"
	"time"

	"github.com/bits-and-blooms/bitset"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

// note: di road network graph osm, wajar ada parallel edge,
// lihat tail dari parallel edge: https://www.openstreetmap.org/node/8445770719#map=19/-7.568382/110.816289
// lihat head dari parallel edge: https://www.openstreetmap.org/node/8445759311#map=19/-7.568314/110.815814

type OsmParser[W util.RoutingNumber] struct {
	wayNodeMap        map[int64]nodeWithCoord // osm nodeId -> tipe dari node (JUNCTION,BETWEEN, END),node coordinate
	relationMemberMap map[int64]struct{}
	barrierNodes      map[int64]bool // osm node Id -> barrier node flag
	nodeTag           map[int64]map[uint32]uint32
	tagStringIdMap    util.IDMap
	nodeIDMap         map[int64]da.Index // osm node Id -> graph nodeId
	nodeToOsmId       map[da.Index]int64 // graph nodeId -> osm node Id
	maxNodeID         int64
	restrictions      map[int64][]restriction // wayId -> list of restrictions
	ways              map[int64]osmWay        // wayId -> osm way

	osmWayDefaultSpeed           map[int64]float64 // wayId -> default max speed
	bb                           *da.BoundingBox
	reversibleOsmWay             map[int64]struct{}
	maxspeeds                    []float64
	currentTime                  time.Time
	highwayWhitelist             map[string]struct{}
	conditionalBarrierNodes      []da.ConditionalBarrierNode
	conditionalReversibleWayVals map[int64]string
	conditionalSpeedLimits       map[int64]string
	conditionalTrafficModesVal   map[int64]string
}

func NewOSMParserV2[W util.RoutingNumber]() *OsmParser[W] {
	p := &OsmParser[W]{
		wayNodeMap:        make(map[int64]nodeWithCoord),
		relationMemberMap: make(map[int64]struct{}),
		barrierNodes:      make(map[int64]bool),
		nodeTag:           make(map[int64]map[uint32]uint32),
		tagStringIdMap:    util.NewIdMap(),
		nodeIDMap:         make(map[int64]da.Index),
		nodeToOsmId:       make(map[da.Index]int64),

		osmWayDefaultSpeed:           make(map[int64]float64),
		bb:                           da.NewBoundingBoxEmpty(),
		currentTime:                  time.Now(),
		highwayWhitelist:             initializeHighwayWhitelist(), // https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Telenav
		ways:                         make(map[int64]osmWay),
		restrictions:                 make(map[int64][]restriction),
		reversibleOsmWay:             make(map[int64]struct{}),
		conditionalBarrierNodes:      make([]da.ConditionalBarrierNode, 0),
		conditionalReversibleWayVals: make(map[int64]string),
		conditionalSpeedLimits:       make(map[int64]string, 0),
		conditionalTrafficModesVal:   make(map[int64]string),
	}
	p.initializeMaxSpeed()
	return p
}

func (p *OsmParser[W]) initializeMaxSpeed() {
	mapMaxSpeeds := viper.GetStringMap("maxspeeds")
	maxspeeds := make([]float64, 18)
	for roadType, speed := range mapMaxSpeeds {
		highwayType := pkg.GetHighwayType(roadType)
		switch v := speed.(type) {
		case int:
			maxspeeds[highwayType] = float64(v)
		case float64:
			maxspeeds[highwayType] = v
		default:
			panic("unsupported type")
		}
	}
	p.maxspeeds = maxspeeds
}

func (p *OsmParser[W]) SetAcceptedNodeMap(acceptedNodeMap map[int64]NodeCoord) {
	wayNodeMap := make(map[int64]nodeWithCoord)
	for id, val := range acceptedNodeMap {
		wayNodeMap[id] = nodeWithCoord{JUNCTION_NODE, val}
	}
	p.wayNodeMap = wayNodeMap
}

func (p *OsmParser[W]) SetNodeToOsmId(nodeToOsmId map[da.Index]int64) {
	p.nodeToOsmId = nodeToOsmId
}

func (p *OsmParser[W]) GetTagStringIdMap() util.IDMap {
	return p.tagStringIdMap
}

func (p *OsmParser[W]) Parse(mapFile string, logger *zap.Logger) (*da.Graph, *costfunction.TimeFunction[W], [][]da.Index, error) {

	f, err := os.Open(mapFile)

	if err != nil {
		return nil, nil, make([][]da.Index, 0), fmt.Errorf("osmParser.Parse: failed to Open file: %s: %w", mapFile, err)
	}

	defer f.Close()

	restrictions := make(map[int64][]struct {
		via             int64
		viaWays         []int64
		turnRestriction TurnRestriction
		timeRangeVal    string
		to              int64
		isWay           bool
		conditional     bool
	})
	scanner := osmpbf.New(context.Background(), f, 0)
	scannedWays := 0
	logger.Sugar().Infof("parsing openstreetmap .pbf file......")
	// scan osm ways and relations
	// store relations and way.nodes

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

				if !p.acceptOsmWay(way) {
					continue
				}

				scannedWays++

				for i, node := range way.Nodes {
					if _, ok := p.wayNodeMap[int64(node.ID)]; !ok {
						if i == 0 || i == len(way.Nodes)-1 {

							p.wayNodeMap[int64(node.ID)] = nodeWithCoord{END_NODE, NewNodeCoord(node.Lat, node.Lon)}
						} else {
							p.wayNodeMap[int64(node.ID)] = nodeWithCoord{BETWEEN_NODE, NewNodeCoord(node.Lat, node.Lon)}
						}
					} else {
						// ini junction node dari osm way
						// jadi ada setidaknya 2 osm way yang punya node yang sama
						// nah node yang sama ini ditandain JUNCTION_NODE
						p.wayNodeMap[int64(node.ID)] = nodeWithCoord{JUNCTION_NODE, NewNodeCoord(node.Lat, node.Lon)}
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
				conditionalTurnRestriction := false
				if tagVal == "" {
					tagVal = relation.Tags.Find("restriction:conditional")
					if tagVal != "" {
						conditionalTurnRestriction = true
					}
				}
				if tagVal != "" {
					// https://wiki.openstreetmap.org/wiki/Relation:restriction

					from := int64(0)
					via := int64(math.MaxInt64)
					to := int64(0)
					// https://www.openstreetmap.org/api/0.6/relation/5710500
					// example: https://www.openstreetmap.org/relation/10732316#map=19/-7.566370/110.775455  or https://www.openstreetmap.org/api/0.6/relation/10732316

					viaways := make([]int64, 0)
					// turn restriction dengan via-way bisa aja via-way nya lebih dari 1, example:  example: https://www.openstreetmap.org/relation/4763182#map=19/-7.783071/110.360767

					isWay := false
					for _, member := range relation.Members {
						if member.Role == "from" {
							from = member.Ref
						} else if member.Role == "to" {
							to = member.Ref
						} else if member.Type == "node" && member.Role == "via" {
							via = member.Ref
							isWay = false
						} else if member.Type == "way" && member.Role == "via" {
							// example: https://www.openstreetmap.org/relation/13427535
							viaways = append(viaways, member.Ref)
							isWay = true
						}
					}

					if via == math.MaxInt64 && len(viaways) == 0 {
						// skip adding restriction yang gak sesuai rule restriction osm:
						// example: https://www.openstreetmap.org/relation/12868204/history/2 (version 2) gak ada via nya...
						// padahal di version 1 bener (ada via nya): https://www.openstreetmap.org/relation/12868204/history/1
						// https://wiki.openstreetmap.org/wiki/Relation:restriction
						continue
					}

					timeRangeVal := ""
					if conditionalTurnRestriction {
						oldTagVal := tagVal
						tagVal = GetAccessValConditionalRestriction(oldTagVal)
						timeRangeVal = GetTimeRangeValConditionalRestriction(oldTagVal)
					}

					rest := struct {
						via             int64
						viaWays         []int64
						turnRestriction TurnRestriction
						timeRangeVal    string
						to              int64
						isWay           bool
						conditional     bool
					}{
						via:             via,
						to:              to,
						turnRestriction: parseTurnRestriction(tagVal),
						timeRangeVal:    timeRangeVal,
						isWay:           isWay,
						viaWays:         viaways,
						conditional:     conditionalTurnRestriction,
					}
					restrictions[from] = append(restrictions[from], rest)
				}
			}
		}
	}

	err = scanner.Close()
	if err != nil {
		return nil, nil, make([][]da.Index, 0), fmt.Errorf("osmParser.Parse: failed to close scanner: %s: %w", mapFile, err)
	}

	_, err = f.Seek(0, io.SeekStart)
	if err != nil {
		return nil, nil, make([][]da.Index, 0), fmt.Errorf("osmParser.Parse: failed to Seek scanner: %s: %w", mapFile, err)
	}

	scanner = osmpbf.New(context.Background(), f, 0)

	// scan osm nodes
	for scanner.Scan() {
		o := scanner.Object()

		tipe := o.ObjectID().Type()

		switch tipe {
		case osm.TypeNode:
			{
				node := o.(*osm.Node)

				p.maxNodeID = max(p.maxNodeID, int64(node.ID))

				_, isUsedNode := p.wayNodeMap[int64(node.ID)]
				if !isUsedNode {
					continue
				}
				oldNode := p.wayNodeMap[int64(node.ID)]
				p.wayNodeMap[int64(node.ID)] = nodeWithCoord{tipe: oldNode.tipe, coord: NewNodeCoord(node.Lat, node.Lon)}

				isBarrierAccessible, err := p.isBarrierNodeAccessible(node)
				if err != nil {
					return nil, nil, make([][]da.Index, 0), fmt.Errorf("osmParser.Parse: isBarrierNodeAccessible() failed to parse conditional accees node barrier: %s: %w", mapFile, err)
				}

				if !isBarrierAccessible {
					p.barrierNodes[int64(node.ID)] = true
				}

				for _, tag := range node.Tags {

					tagID := p.tagStringIdMap.GetID(tag.Key)
					if _, ok := p.nodeTag[int64(node.ID)]; !ok {
						p.nodeTag[int64(node.ID)] = make(map[uint32]uint32)
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

	err = scanner.Close()
	if err != nil {
		return nil, nil, make([][]da.Index, 0), fmt.Errorf("osmParser.Parse: failed to close scanner: %s: %w", mapFile, err)
	}

	_, err = f.Seek(0, io.SeekStart)
	if err != nil {
		return nil, nil, make([][]da.Index, 0), fmt.Errorf("osmParser.Parse: failed to Seek scanner: %s: %w", mapFile, err)
	}

	// scan osm way and store graph edges
	scanner = osmpbf.New(context.Background(), f, 0)
	scannedEdges := make([]Edge[W], 0)
	p.ways = make(map[int64]osmWay, scannedWays)
	streetDirection := make(map[int64][2]bool)
	countWays := 0
	logger.Sugar().Infof("processing openstreetmap .pbf file: 0%.... ")
	outputEvery := 5
	pg := make([]bool, 101)
	graphStorage := da.NewGraphStorage(da.DEFAULT_BIT_SIZE_OSM_WAY_ID)

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

				if !p.acceptOsmWay(way) {
					continue
				}

				progress := int(((float64(countWays) + 1) / float64(scannedWays)) * 100)
				if progress > 0 && progress%outputEvery == 0 && !pg[progress] {
					pg[progress] = true
					logger.Sugar().Infof("processing openstreetmap .pbf file: %v %% .....  ", int(progress))
				}

				countWays++
				tempMap := make(map[string]string)

				hwTag, err := p.processWay(way, graphStorage, streetDirection, &scannedEdges, tempMap)
				if err != nil {
					continue
				}

				oneWay := true
				if streetDirection[int64(way.ID)][1] && streetDirection[int64(way.ID)][0] {
					// (true,true)=two way road, not a one way road
					oneWay = false
				}

				wNodes := make([]int64, 0, len(way.Nodes))
				for _, node := range way.Nodes {

					wNodes = append(wNodes, int64(node.ID))
				}
				p.ways[int64(way.ID)] = osmWay{
					nodes:  wNodes,
					oneWay: oneWay,
					hwTag:  hwTag,
				}
			}

		}
	}

	for nodeID, nodeIDX := range p.nodeIDMap {
		if val, ok := p.nodeTag[int64(nodeID)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)]; ok && val == 1 {
			graphStorage.SetTrafficLight(nodeIDX, true)
		}
	}

	p.restrictions = make(map[int64][]restriction, len(restrictions))
	for from, val := range restrictions {
		savedRest := make([]restriction, len(val))
		for i := range val {

			if !val[i].isWay {
				savedRest[i] = restriction{
					via:             p.nodeIDMap[val[i].via],
					to:              val[i].to,
					turnRestriction: val[i].turnRestriction,
					isWay:           val[i].isWay,
					conditional:     val[i].conditional,
					timeRangeVal:    val[i].timeRangeVal,
				}
			} else {
				savedRest[i] = restriction{
					viaWays:         val[i].viaWays,
					to:              val[i].to,
					turnRestriction: val[i].turnRestriction,
					isWay:           val[i].isWay,
					conditional:     val[i].conditional,
					timeRangeVal:    val[i].timeRangeVal,
				}
			}

		}

		p.restrictions[from] = savedRest
	}

	logger.Sugar().Infof("building road network graph.... ")

	for wayId, way := range p.ways {
		graphNodes := make([]da.Index, 0)
		for _, node := range way.nodes {
			graphNodeId, exists := p.nodeIDMap[node]
			if !exists {
				continue
			}
			graphNodes = append(graphNodes, graphNodeId)
		}

		way := p.ways[wayId]
		way.graphNodes = graphNodes
		p.ways[wayId] = way
	}

	scannedEdges, graphStorage, numVertices := p.compressOSMGraph(
		scannedEdges, graphStorage, streetDirection,
	)

	graph, timeFunction, edgeInfoIds := p.BuildGraph(
		scannedEdges, graphStorage, numVertices, true,
	)

	graph.SetBoundingBox(p.bb)

	streetDirectionForward := bitset.New(uint(graph.NumberOfEdges()))
	streetDirectionBackward := bitset.New(uint(graph.NumberOfEdges()))

	graph.ForOutEdges(func(exitPoint, head, tail, entryId, entryPoint da.Index, percentage float64, eId da.Index) {
		edgeInfoId := edgeInfoIds[tail][exitPoint]
		if edgeInfoId == da.INVALID_EDGE_INFO_ID { // skip dummy edges
			return
		}
		eOsmWayId := graph.GetOsmWayId(edgeInfoId)
		direction := streetDirection[eOsmWayId]
		if direction[0] {
			streetDirectionForward.Set(uint(eId))
		}

		if direction[1] {
			streetDirectionBackward.Set(uint(eId))
		}
	})

	graphStorage.SetStreetDirection(streetDirectionForward, streetDirectionBackward)

	logger.Sugar().Infof("number of vertices: %v", graph.NumberOfVertices())
	logger.Sugar().Infof("number of edges: %v", graph.NumberOfEdges())

	if err = scanner.Close(); err != nil {
		return nil, nil, make([][]da.Index, 0), fmt.Errorf("osmParser.Parse: failed to Close scanner: %s: %w", mapFile, err)
	}

	return graph, timeFunction, edgeInfoIds, nil
}

type wayExtraInfo struct {
	oneWay  bool
	forward bool
}

func (p *OsmParser[W]) processWay(way *osm.Way, graphStorage *da.GraphStorage,
	streetDirection map[int64][2]bool,
	scannedEdges *[]Edge[W], tempMap map[string]string) (string, error) {
	var err error

	getName(way, tempMap)

	maxSpeed := 0.0
	highwayTypeSpeed := 0.0

	wayExtraInfoData := wayExtraInfo{}

	for _, tag := range way.Tags {
		switch tag.Key {
		case "junction":
			{
				tempMap[JUNCTION] = tag.Value
			}
		case "highway":
			{
				highwayTypeSpeed = p.roadTypeSpeed(tag.Value)

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
				maxSpeed, err = parseOsmWayMaxSpeedVal(tag.Value)
				if err != nil {
					return "", fmt.Errorf("processWay: failed to parseOsmWayMaxSpeedVal(), wayId: %v: %w", way.ID, err)
				}
			}
		case "maxspeed:conditional":
			p.conditionalSpeedLimits[int64(way.ID)] = tag.Value

		}
	}

	if maxSpeed == 0 {
		maxSpeed = highwayTypeSpeed
	}
	if maxSpeed == 0 {
		maxSpeed = 30
	}

	// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing
	// https://wiki.openstreetmap.org/wiki/Forward_%26_backward,_left_%26_right#Identifying_the_direction_of_a_way
	// https://wiki.openstreetmap.org/wiki/Key:oneway
	restrictedForward, restrictedBackward := getReversedOneWay(way)
	// see Land-based transportation: https://wiki.openstreetmap.org/wiki/Key:access
	// vehicle is all type of vehicle (bicycle, car, bus, motorcycle, etc)
	/*
		taken from: https://wiki.openstreetmap.org/wiki/Forward_%26_backward,_left_%26_right#Identifying_the_direction_of_a_way
			On a street, such as highway=unclassified, highway=residential or highway=pedestrian, oneway=* can safely be interpreted as applying only to vehicles. It should not affect pedestrian routing.
			, and is effectively a synonym for vehicle:backward=no.
	*/

	// pedestrian
	/*
		taken from: https://wiki.openstreetmap.org/wiki/Forward_%26_backward,_left_%26_right#Identifying_the_direction_of_a_way
		On highway=steps and highway=via_ferrata, the tag oneway=* can safely be interpreted as applying to pedestrians.
		(the tag oneway=no has precedence over highway=motorway)

	*/

	vehicleType, ok := pkg.VehicleTypeTag[pkg.VehicleType]
	vehicleTypeOneway := way.Tags.Find(fmt.Sprintf("oneway:%s", vehicleType))
	val := way.Tags.Find("oneway")
	if val == "yes" || val == "-1" || restrictedForward || restrictedBackward || (pkg.IsVehicleEnabled &&
		(tempMap[ROAD_CLASS] == "unclassified" || tempMap[ROAD_CLASS] == "residential") && (val == "yes" || val == "-1")) ||
		(!pkg.IsVehicleEnabled && (tempMap[ROAD_CLASS] == "via_ferrata" || tempMap[ROAD_CLASS] == "steps") && (val == "yes" || val == "-1")) ||
		tempMap[JUNCTION] == "roundabout" || (tempMap[ROAD_CLASS] == "motorway" && val != "no" && val != "") || (ok && (vehicleTypeOneway == "yes" || vehicleTypeOneway == "-1")) {
		wayExtraInfoData.oneWay = true

		if way.Tags.Find("oneway") == "-1" || restrictedForward {
			// restrictedForward = restricted/not allowed forward.
			wayExtraInfoData.forward = false

		} else {
			// kalau gak restrictedForward berarti forward directionnya
			// https://www.openstreetmap.org/way/141318123
			wayExtraInfoData.forward = true
		}
	} else if val == "reversible" {
		//  https://wiki.openstreetmap.org/wiki/Tag:oneway%3Dreversible
		wayExtraInfoData.oneWay = true
		reversibleVal := way.Tags.Find("oneway:conditional")
		if reversibleVal != "" {

			p.conditionalReversibleWayVals[int64(way.ID)] = reversibleVal
		} else {
			// https://wiki.openstreetmap.org/wiki/Tag:oneway=reversible?uselang=en
			// If a vehicle travels a oneway=reversible in any direction, routing engines should infer u-turn restriction
			wayExtraInfoData.oneWay = false
			// tandain osm way ini dengan u-turn restriction
			p.reversibleOsmWay[int64(way.ID)] = struct{}{}
		}
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

	waySegment := make([]node, 0, 16)
	for _, wayNode := range way.Nodes {
		nodeId := int64(wayNode.ID)
		wayNodeData := p.wayNodeMap[nodeId]

		nodeCoord := wayNodeData.coord
		nodeData := node{
			id:    nodeId,
			coord: nodeCoord,
		}
		if p.isJunctionNode(nodeId) {
			waySegment = append(waySegment, nodeData)
			p.processSegment(waySegment, tempMap, maxSpeed, graphStorage, wayExtraInfoData,
				scannedEdges, int64(way.ID))
			waySegment = make([]node, 0, 16)

			waySegment = append(waySegment, nodeData)
		} else {
			waySegment = append(waySegment, nodeData)
		}
	}
	if len(waySegment) > 1 {
		p.processSegment(waySegment, tempMap, maxSpeed, graphStorage, wayExtraInfoData, scannedEdges,
			int64(way.ID))
	}

	hwTag := tempMap[ROAD_CLASS]
	return hwTag, nil
}

func (p *OsmParser[W]) processSegment(segment []node, tempMap map[string]string, speed float64, graphStorage *da.GraphStorage,
	wayExtraInfoData wayExtraInfo, scannedEdges *[]Edge[W], id int64) {

	if len(segment) == 2 && segment[0].id == segment[1].id {
		// skip loop edge
		return
	} else if len(segment) > 2 && segment[0].id == segment[len(segment)-1].id {
		// loop
		p.processSegment2(segment[0:len(segment)-1], tempMap, speed, graphStorage, wayExtraInfoData, scannedEdges, id)
		p.processSegment2(segment[len(segment)-2:], tempMap, speed, graphStorage, wayExtraInfoData, scannedEdges, id)
	} else {
		p.processSegment2(segment, tempMap, speed, graphStorage, wayExtraInfoData, scannedEdges, id)
	}
}

func (p *OsmParser[W]) processSegment2(segment []node, tempMap map[string]string, speed float64, graphStorage *da.GraphStorage,
	wayExtraInfoData wayExtraInfo, scannedEdges *[]Edge[W], id int64,
) {
	waySegment := []node{}
	for i := 0; i < len(segment); i++ {
		nodeData := segment[i]
		if _, ok := p.barrierNodes[int64(nodeData.id)]; ok {

			if len(waySegment) != 0 {
				// if current node is a barrier
				// add the barrier node and process the segment (add edge)
				waySegment = append(waySegment, nodeData)
				p.addEdge(waySegment, tempMap, speed, graphStorage, wayExtraInfoData, scannedEdges, id)
				waySegment = []node{}
			}
			// copy the barrier node but with different id so that previous edge (with barrier) not connected with the new edge

			nodeDataCopy := p.copyNode(nodeData)
			waySegment = append(waySegment, nodeDataCopy)

		} else {
			waySegment = append(waySegment, nodeData)
		}
	}
	if len(waySegment) > 1 {
		p.addEdge(waySegment, tempMap, speed, graphStorage, wayExtraInfoData, scannedEdges, id)
	}
}

func (p *OsmParser[W]) copyNode(nodeData node) node {
	// use the same coordinate but different id & and the newID is not used
	newMaxID := p.maxNodeID + 1
	p.wayNodeMap[newMaxID] = nodeWithCoord{
		tipe: JUNCTION_NODE, coord: NodeCoord{lat: nodeData.coord.lat,
			lon: nodeData.coord.lon},
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

func (p *OsmParser[W]) addEdge(segment []node, tempMap map[string]string, speed float64, graphStorage *da.GraphStorage,
	wayExtraInfoData wayExtraInfo, scannedEdges *[]Edge[W], id int64) {
	var (
		lanes int
	)

	from := segment[0]

	to := segment[len(segment)-1]

	if from.id == to.id {
		return
	}

	maxLonft := util.MaxFloat(from.coord.lon, to.coord.lon)
	maxLatft := util.MaxFloat(from.coord.lat, to.coord.lat)

	minLonft := util.MinFloat(from.coord.lon, to.coord.lon)
	minLatft := util.MinFloat(from.coord.lat, to.coord.lat)

	p.bb.SetMaxLat(util.MaxFloat(p.bb.GetMaxLat(), maxLatft))
	p.bb.SetMaxLon(util.MaxFloat(p.bb.GetMaxLon(), maxLonft))
	p.bb.SetMinLat(util.MinFloat(p.bb.GetMinLat(), minLatft))
	p.bb.SetMinLon(util.MinFloat(p.bb.GetMinLon(), minLonft))

	if _, ok := p.nodeIDMap[from.id]; !ok {
		p.nodeIDMap[from.id] = da.Index(len(p.nodeIDMap))
		p.nodeToOsmId[p.nodeIDMap[from.id]] = from.id
	}
	if _, ok := p.nodeIDMap[to.id]; !ok {
		p.nodeIDMap[to.id] = da.Index(len(p.nodeIDMap))
		p.nodeToOsmId[p.nodeIDMap[to.id]] = to.id
	}

	containTrafficLight := false
	edgePoints := make([]da.Coordinate, 0)
	distance := 0.0
	for i := 0; i < len(segment); i++ {
		if p.nodeTag[int64(segment[i].id)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] == 1 {
			containTrafficLight = true
		}
		edgePoints = append(edgePoints, da.NewCoordinate(
			segment[i].coord.lat,
			segment[i].coord.lon,
		))
		if i > 0 {
			distance += geo.CalculateGreatCircleDistance(segment[i-1].coord.lat, segment[i-1].coord.lon, segment[i].coord.lat, segment[i].coord.lon)
		}
	}

	isRoundabout := false
	streetName := tempMap[STREET_NAME]
	if val, ok := tempMap[JUNCTION]; ok {
		if val == "roundabout" {
			isRoundabout = true
		}
		if val == "circular" {
			isRoundabout = true
		}
	} else if p.isRoundaboutByName(streetName) {
		isRoundabout = true
	}

	distanceInMeter := util.KilometerToMeter(distance)

	travelTimeWeight := distanceInMeter / util.KMHToMSeconds(speed) // in seconds
	fixedWeight := W(util.RoundCentiseconds(travelTimeWeight))
	fixedDistance := uint32(util.RoundCentimeters(distanceInMeter))

	p.osmWayDefaultSpeed[id] = speed

	lanesString, exists := tempMap[LANES]
	if !exists {
		lanes = 1
	} else {
		lanes, _ = util.ParseTextInt(lanesString)
	}

	fromNId := p.nodeIDMap[from.id]

	toNId := p.nodeIDMap[to.id]

	roadClass := tempMap[ROAD_CLASS]
	if roadClass == "" {
		roadClass = tempMap[ROAD_CLASS_LINK]
	}

	hwType := pkg.GetHighwayType(roadClass)
	isCurved := geo.IsPolylineCurved(edgePoints)

	if wayExtraInfoData.oneWay {
		if wayExtraInfoData.forward {

			startPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendOsmNodePoints(edgePoints)
			endPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendEdgeMetadata(
				id,
				da.Index(startPointsIndex), da.Index(endPointsIndex),
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				pkg.GetHighwayType(tempMap[ROAD_CLASS]),
				pkg.GetHighwayType(tempMap[ROAD_CLASS_LINK]),
				uint8(lanes),
			)

			graphStorage.SetRoundabout(da.Index(len(*scannedEdges)), isRoundabout)

			e := NewEdge(
				uint32(fromNId),
				uint32(toNId),
				fixedWeight,
				fixedDistance,
				containTrafficLight,
				hwType,
			)

			e.SetFromOSMId(uint64(from.id))
			e.SetToOSMId(uint64(to.id))
			e.SetOsmWayId(id)

			if p.isJunctionNode(from.id) {
				e.SetJunctionTail()
			}
			if p.isJunctionNode(to.id) {
				e.SetJunctionHead()
			}

			graphStorage.SetIsCurved(da.Index(len(*scannedEdges)), isCurved)

			*scannedEdges = append(*scannedEdges, e)

		} else {

			util.ReverseG(edgePoints)

			startPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendOsmNodePoints(edgePoints)
			endPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendEdgeMetadata(
				id,
				da.Index(startPointsIndex), da.Index(endPointsIndex),
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				pkg.GetHighwayType(tempMap[ROAD_CLASS]),
				pkg.GetHighwayType(tempMap[ROAD_CLASS_LINK]),
				uint8(lanes),
			)

			graphStorage.SetRoundabout(da.Index(len(*scannedEdges)), isRoundabout)
			e := NewEdge[W](
				uint32(toNId),
				uint32(fromNId),
				fixedWeight,
				fixedDistance,
				containTrafficLight,
				hwType,
			)

			e.SetFromOSMId(uint64(to.id))
			e.SetToOSMId(uint64(from.id))
			e.SetOsmWayId(id)

			if p.isJunctionNode(to.id) {
				e.SetJunctionTail()
			}

			if p.isJunctionNode(from.id) {
				e.SetJunctionHead()
			}

			graphStorage.SetIsCurved(da.Index(len(*scannedEdges)), isCurved)

			*scannedEdges = append(*scannedEdges, e)
		}
	} else {

		// add forward edge
		startPointsIndex := graphStorage.GetOsmNodePointsCount()

		graphStorage.AppendOsmNodePoints(edgePoints)
		endPointsIndex := graphStorage.GetOsmNodePointsCount()

		graphStorage.AppendEdgeMetadata(
			id,
			da.Index(startPointsIndex), da.Index(endPointsIndex),
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			pkg.GetHighwayType(tempMap[ROAD_CLASS]),
			pkg.GetHighwayType(tempMap[ROAD_CLASS_LINK]),
			uint8(lanes),
		)

		graphStorage.SetRoundabout(da.Index(len(*scannedEdges)), isRoundabout)

		e := NewEdge(
			uint32(fromNId),
			uint32(toNId),
			fixedWeight,
			fixedDistance,
			containTrafficLight,
			hwType,
		)

		e.SetFromOSMId(uint64(from.id))
		e.SetToOSMId(uint64(to.id))
		e.SetOsmWayId(id)

		if p.isJunctionNode(from.id) {
			e.SetJunctionTail()
		}
		if p.isJunctionNode(to.id) {
			e.SetJunctionHead()
		}

		graphStorage.SetIsCurved(da.Index(len(*scannedEdges)), isCurved)

		*scannedEdges = append(*scannedEdges, e)

		// add reversed edge

		graphStorage.AppendEdgeMetadata(
			id,
			da.Index(endPointsIndex), da.Index(startPointsIndex),
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			pkg.GetHighwayType(tempMap[ROAD_CLASS]),
			pkg.GetHighwayType(tempMap[ROAD_CLASS_LINK]),
			uint8(lanes),
		)

		graphStorage.SetRoundabout(da.Index(len(*scannedEdges)), isRoundabout)

		e = NewEdge(
			uint32(toNId),
			uint32(fromNId),
			fixedWeight,
			fixedDistance,
			containTrafficLight,
			hwType,
		)

		e.SetFromOSMId(uint64(to.id))
		e.SetToOSMId(uint64(from.id))
		e.SetOsmWayId(id)

		if p.isJunctionNode(to.id) {
			e.SetJunctionTail()
		}

		if p.isJunctionNode(from.id) {
			e.SetJunctionHead()
		}

		graphStorage.SetIsCurved(da.Index(len(*scannedEdges)), isCurved)

		*scannedEdges = append(*scannedEdges, e)
	}
}

func (p *OsmParser[W]) isJunctionNode(nodeID int64) bool {
	return p.wayNodeMap[int64(nodeID)].tipe == JUNCTION_NODE
}

func (p *OsmParser[W]) GetNodeIdMap() map[int64]da.Index {
	return p.nodeIDMap
}
