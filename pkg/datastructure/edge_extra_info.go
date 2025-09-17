package datastructure

import "math"

/*
----
edge extra info related section
----
*/
type GraphStorage struct {
	globalPoints []Coordinate

	/*
		32 bit -> 32 boolean flag for roundabout & trafficlight

		idx in flag array = floor(edgeID/32)
		idx in flag = edgeID % 32
	*/
	roundaboutFlag   []uint32
	nodeTrafficLight []uint32

	mapEdgeInfo []EdgeExtraInfo

	reversedEdgeFlag map[uint32]map[uint32]bool // fromNodeID -> toNodeID -> isReversed
}

func NewGraphStorage() *GraphStorage {
	return &GraphStorage{
		reversedEdgeFlag: make(map[uint32]map[uint32]bool),
	}
}
func (gs *GraphStorage) SetRoundabout(edgeID uint32, isRoundabout bool) {
	index := int(math.Floor(float64(edgeID) / 32))
	if len(gs.roundaboutFlag) <= int(index) {
		gs.roundaboutFlag = append(gs.roundaboutFlag, make([]uint32, edgeID-uint32(len(gs.roundaboutFlag))+1)...)
	}
	if isRoundabout {
		gs.roundaboutFlag[index] |= 1 << (edgeID % 32)
	}
}

func (gs *GraphStorage) SetTrafficLight(nodeID uint32) {
	index := int(math.Floor(float64(nodeID) / 32))

	if len(gs.nodeTrafficLight) <= int(index) {
		gs.nodeTrafficLight = append(gs.nodeTrafficLight, make([]uint32, nodeID-uint32(len(gs.nodeTrafficLight))+1)...)
	}

	gs.nodeTrafficLight[index] |= 1 << (nodeID % 32)
}

func (gs *GraphStorage) GetTrafficLight(nodeID uint32) bool {
	index := int(math.Floor(float64(nodeID) / 32))

	return (gs.nodeTrafficLight[index] & (1 << (nodeID % 32))) != 0
}

type EdgeExtraInfo struct {
	StartPointsIndex uint32
	EndPointsIndex   uint32
	StreetName       int
	RoadClass        uint8
	RoadClassLink    uint8
	Lanes            uint8
}

func NewEdgeExtraInfo(streetName int, roadClass, lanes, roadClassLink uint8, StartPointsIdx, EndPointsIdx uint32) EdgeExtraInfo {
	return EdgeExtraInfo{
		StreetName:       streetName,
		RoadClass:        roadClass,
		RoadClassLink:    roadClassLink,
		Lanes:            lanes,
		StartPointsIndex: StartPointsIdx,
		EndPointsIndex:   EndPointsIdx,
	}
}

func (gs *GraphStorage) GetPointsInbetween(edgeID uint32) []Coordinate {
	edge := gs.mapEdgeInfo[edgeID]
	var (
		edgePoints []Coordinate
	)
	startIndex := edge.StartPointsIndex
	endIndex := edge.EndPointsIndex
	if startIndex <= endIndex {
		edgePoints = gs.globalPoints[startIndex:endIndex]

		return edgePoints
	}

	for i := startIndex - 1; i >= endIndex; i-- {
		edgePoints = append(edgePoints, gs.globalPoints[i])
	}

	return edgePoints
}

// return edgeExtraInfo, isRoundabout
func (gs *GraphStorage) GetEdgeExtraInfo(edgeID uint32, reverse bool) (EdgeExtraInfo, bool) {

	index := int(math.Floor(float64(edgeID) / 32))
	roundabout := (gs.roundaboutFlag[index] & (1 << (edgeID % 32))) != 0
	return gs.mapEdgeInfo[edgeID], roundabout

}

func (gs *GraphStorage) UpdateEdgePoints(edgeID uint32, startIdx, endIdx uint32) {
	edge := gs.mapEdgeInfo[edgeID]
	edge.StartPointsIndex = startIdx
	edge.EndPointsIndex = endIdx
	gs.mapEdgeInfo[edgeID] = edge
}

func (gs *GraphStorage) AppendGlobalPoints(edgePoints []Coordinate) {
	gs.globalPoints = append(gs.globalPoints, edgePoints...)
}

func (gs *GraphStorage) AppendMapEdgeInfo(edgeInfo EdgeExtraInfo) {
	gs.mapEdgeInfo = append(gs.mapEdgeInfo, edgeInfo)
}

func (gs *GraphStorage) SetFlagReversedEdge(fromID, toID uint32) {
	if _, ok := gs.reversedEdgeFlag[fromID]; !ok {
		gs.reversedEdgeFlag[fromID] = make(map[uint32]bool)
	}
	gs.reversedEdgeFlag[fromID][toID] = true
}

func (gs *GraphStorage) GetGlobalPointsCount() int {
	return len(gs.globalPoints)
}

func (gs *GraphStorage) IsReversedEdge(fromID, toID uint32) bool {
	if _, ok := gs.reversedEdgeFlag[fromID]; !ok {
		return false
	}
	if _, ok := gs.reversedEdgeFlag[fromID][toID]; !ok {
		return false
	}

	return gs.reversedEdgeFlag[fromID][toID]
}
