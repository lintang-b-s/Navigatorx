package datastructure

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type GraphStorage struct {
	globalPoints []Coordinate

	/*
		32 bit -> 32 boolean flag for roundabout & trafficlight

		idx in flag array = floor(edgeID/32)
		idx in flag = edgeID % 32
	*/
	roundaboutFlag   []Index
	nodeTrafficLight []Index

	mapEdgeInfo []EdgeExtraInfo
	trafficWay  map[int64]struct{}

	tagStringIDMap  util.IDMap
	streetDirection map[int64][2]bool // osm way id -> [forward,backward]
}

func NewGraphStorage() *GraphStorage {
	return &GraphStorage{trafficWay: make(map[int64]struct{}),
		streetDirection:  make(map[int64][2]bool),
		mapEdgeInfo:      make([]EdgeExtraInfo, 0),
		tagStringIDMap:   util.NewIdMap(),
		roundaboutFlag:   make([]Index, 0),
		nodeTrafficLight: make([]Index, 0),
		globalPoints:     make([]Coordinate, 0),
	}
}

func BuildGraphStorage(globalPoints []Coordinate, roundaboutFlag []Index, nodeTrafficLight []Index,
	mapEdgeInfo []EdgeExtraInfo, tagStringIDMap util.IDMap, streetDirection map[int64][2]bool) *GraphStorage {
	return &GraphStorage{globalPoints: globalPoints, roundaboutFlag: roundaboutFlag,
		nodeTrafficLight: nodeTrafficLight, mapEdgeInfo: mapEdgeInfo, tagStringIDMap: tagStringIDMap,
		streetDirection: streetDirection}
}

func NewGraphStorageWithSize(numberOfEdges int, numberOfVertices int) *GraphStorage {
	return &GraphStorage{trafficWay: make(map[int64]struct{}),
		streetDirection:  make(map[int64][2]bool),
		mapEdgeInfo:      make([]EdgeExtraInfo, numberOfEdges),
		tagStringIDMap:   util.NewIdMap(),
		roundaboutFlag:   make([]Index, numberOfEdges),
		nodeTrafficLight: make([]Index, numberOfVertices),
		globalPoints:     make([]Coordinate, 1),
	}
}

func (gs *GraphStorage) SetTrafficWayMap(trafficWay map[int64]struct{}) {
	gs.trafficWay = trafficWay
}

func (gs *GraphStorage) SetRoundabout(edgeID Index, isRoundabout bool) {
	index := int(math.Floor(float64(edgeID) / 32))
	if len(gs.roundaboutFlag) <= int(index) {
		gs.roundaboutFlag = append(gs.roundaboutFlag, make([]Index, edgeID-Index(len(gs.roundaboutFlag))+1)...)
	}
	if isRoundabout {
		gs.roundaboutFlag[index] |= 1 << (edgeID % 32)
	}
}

func (gs *GraphStorage) SetStreetDirection(streetDirection map[int64][2]bool) {
	gs.streetDirection = streetDirection
}

func (gs *GraphStorage) SetTagStringIdMap(tagStringIDMap util.IDMap) {
	gs.tagStringIDMap = tagStringIDMap
}

func (gs *GraphStorage) GetStreetDirection(wayId int64) [2]bool {
	return gs.streetDirection[wayId]
}

func (gs *GraphStorage) SetTrafficLight(nodeID Index) {
	index := int(math.Floor(float64(nodeID) / 32))

	if len(gs.nodeTrafficLight) <= int(index) {
		gs.nodeTrafficLight = append(gs.nodeTrafficLight, make([]Index, nodeID-Index(len(gs.nodeTrafficLight))+1)...)
	}

	gs.nodeTrafficLight[index] |= 1 << (nodeID % 32)
}

func (gs *GraphStorage) GetTrafficLight(nodeID Index) bool {
	index := int(math.Floor(float64(nodeID) / 32))

	var tf bool
	if len(gs.nodeTrafficLight) > 0 {
		tf = (gs.nodeTrafficLight[index] & (1 << (nodeID % 32))) != 0
	}
	return tf
}

type EdgeExtraInfo struct {
	startPointsIndex Index
	endPointsIndex   Index
	streetName       int
	roadClass        uint8
	roadClassLink    uint8
	lanes            uint8
	osmWayId         int64
}

func NewEdgeExtraInfo(streetName int, roadClass, lanes, roadClassLink uint8, StartPointsIdx, EndPointsIdx Index,
	osmWayId int64) EdgeExtraInfo {
	return EdgeExtraInfo{
		streetName:       streetName,
		roadClass:        roadClass,
		roadClassLink:    roadClassLink,
		lanes:            lanes,
		startPointsIndex: StartPointsIdx,
		endPointsIndex:   EndPointsIdx,
		osmWayId:         osmWayId,
	}
}

func (e *EdgeExtraInfo) GetStartPointsIndex() Index {
	return e.startPointsIndex
}

func (e *EdgeExtraInfo) GetEndPointsIndex() Index {
	return e.endPointsIndex
}

func (gs *GraphStorage) GetEdgeGeometry(edgeID Index) []Coordinate {
	edge := gs.mapEdgeInfo[edgeID]
	var (
		edgePoints []Coordinate
	)
	if edge.osmWayId == -1 {
		return []Coordinate{}
	}
	startIndex := edge.startPointsIndex
	endIndex := edge.endPointsIndex
	if startIndex < endIndex {
		edgePoints = gs.globalPoints[startIndex:endIndex]

		return edgePoints
	}

	if startIndex <= 0 {
		return make([]Coordinate, 0)
	}

	for i := startIndex - 1; i >= endIndex; i-- {
		edgePoints = append(edgePoints, gs.globalPoints[i])
	}

	return edgePoints
}

func (gs *GraphStorage) isWayTrafficLight(wayId int64) bool {
	_, tf := gs.trafficWay[wayId]
	return tf
}

func (gs *GraphStorage) SetWayTraffic(wayId int64) {
	gs.trafficWay[wayId] = struct{}{}
}

// return edgeExtraInfo, isRoundabout
func (gs *GraphStorage) GetEdgeExtraInfo(edgeID Index, reverse bool) (EdgeExtraInfo, bool) {

	index := int(math.Floor(float64(edgeID) / 32))
	var roundabout bool
	if len(gs.roundaboutFlag) > 0 {
		roundabout = (gs.roundaboutFlag[index] & (1 << (edgeID % 32))) != 0
	}
	return gs.mapEdgeInfo[edgeID], roundabout
}

func (gs *GraphStorage) UpdateEdgePoints(edgeID Index, startIdx, endIdx Index) {
	edge := gs.mapEdgeInfo[edgeID]
	edge.startPointsIndex = startIdx
	edge.endPointsIndex = endIdx
	gs.mapEdgeInfo[edgeID] = edge
}

func (gs *GraphStorage) AppendGlobalPoints(edgePoints []Coordinate) {
	gs.globalPoints = append(gs.globalPoints, edgePoints...)
}

func (gs *GraphStorage) AppendMapEdgeInfo(edgeInfo EdgeExtraInfo) {
	gs.mapEdgeInfo = append(gs.mapEdgeInfo, edgeInfo)
}

func (gs *GraphStorage) GetGlobalPointsCount() int {
	return len(gs.globalPoints)
}
