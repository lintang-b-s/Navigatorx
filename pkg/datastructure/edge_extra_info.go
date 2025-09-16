package datastructure

import "math"

/*
----
edge extra info related section
----
*/
type GraphStorage struct {
	GlobalPoints []Coordinate

	/*
		32 bit -> 32 boolean flag for roundabout & trafficlight

		idx in flag array = floor(edgeID/32)
		idx in flag = edgeID % 32
	*/
	RoundaboutFlag   []uint32
	NodeTrafficLight []uint32

	MapEdgeInfo []EdgeExtraInfo

	ReversedEdgeFlag map[uint32]map[uint32]bool // fromNodeID -> toNodeID -> isReversed
}

func NewGraphStorage() *GraphStorage {
	return &GraphStorage{
		ReversedEdgeFlag: make(map[uint32]map[uint32]bool),
	}
}
func (gs *GraphStorage) SetRoundabout(edgeID uint32, isRoundabout bool) {
	index := int(math.Floor(float64(edgeID) / 32))
	if len(gs.RoundaboutFlag) <= int(index) {
		gs.RoundaboutFlag = append(gs.RoundaboutFlag, make([]uint32, edgeID-uint32(len(gs.RoundaboutFlag))+1)...)
	}
	if isRoundabout {
		gs.RoundaboutFlag[index] |= 1 << (edgeID % 32)
	}
}

func (gs *GraphStorage) SetTrafficLight(nodeID uint32) {
	index := int(math.Floor(float64(nodeID) / 32))

	if len(gs.NodeTrafficLight) <= int(index) {
		gs.NodeTrafficLight = append(gs.NodeTrafficLight, make([]uint32, nodeID-uint32(len(gs.NodeTrafficLight))+1)...)
	}

	gs.NodeTrafficLight[index] |= 1 << (nodeID % 32)
}

func (gs *GraphStorage) GetTrafficLight(nodeID uint32) bool {
	index := int(math.Floor(float64(nodeID) / 32))

	return (gs.NodeTrafficLight[index] & (1 << (nodeID % 32))) != 0
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
	edge := gs.MapEdgeInfo[edgeID]
	var (
		edgePoints []Coordinate
	)
	startIndex := edge.StartPointsIndex
	endIndex := edge.EndPointsIndex
	if startIndex <= endIndex {
		edgePoints = gs.GlobalPoints[startIndex:endIndex]

		return edgePoints
	}

	for i := startIndex - 1; i >= endIndex; i-- {
		edgePoints = append(edgePoints, gs.GlobalPoints[i])
	}

	return edgePoints
}

// return edgeExtraInfo, isRoundabout
func (gs *GraphStorage) GetEdgeExtraInfo(edgeID uint32, reverse bool) (EdgeExtraInfo, bool) {

	index := int(math.Floor(float64(edgeID) / 32))
	roundabout := (gs.RoundaboutFlag[index] & (1 << (edgeID % 32))) != 0
	return gs.MapEdgeInfo[edgeID], roundabout

}

func (gs *GraphStorage) UpdateEdgePoints(edgeID uint32, startIdx, endIdx uint32) {
	edge := gs.MapEdgeInfo[edgeID]
	edge.StartPointsIndex = startIdx
	edge.EndPointsIndex = endIdx
	gs.MapEdgeInfo[edgeID] = edge
}

func (gs *GraphStorage) AppendGlobalPoints(edgePoints []Coordinate) {
	gs.GlobalPoints = append(gs.GlobalPoints, edgePoints...)
}

func (gs *GraphStorage) AppendMapEdgeInfo(edgeInfo EdgeExtraInfo) {
	gs.MapEdgeInfo = append(gs.MapEdgeInfo, edgeInfo)
}

func (gs *GraphStorage) SetFlagReversedEdge(fromID, toID uint32) {
	if _, ok := gs.ReversedEdgeFlag[fromID]; !ok {
		gs.ReversedEdgeFlag[fromID] = make(map[uint32]bool)
	}
	gs.ReversedEdgeFlag[fromID][toID] = true
}

func (gs *GraphStorage) IsReversedEdge(fromID, toID uint32) bool {
	if _, ok := gs.ReversedEdgeFlag[fromID]; !ok {
		return false
	}
	if _, ok := gs.ReversedEdgeFlag[fromID][toID]; !ok {
		return false
	}

	return gs.ReversedEdgeFlag[fromID][toID]
}
