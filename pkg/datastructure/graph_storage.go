package datastructure

import (
	"github.com/bits-and-blooms/bitset"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// TODO: use mmap for storing edgeInfos & osmNodePoints (https://man7.org/linux/man-pages/man2/mmap.2.html), baca linux programming inteface chap 49
// bisa ngurangin banyak heap allocations
// ini pakai file osm diy_solo_jogja physical mem (RES): 1.9 gb
// osrm pakai file osm diy_solo_jogja physical mem (RES): cuma 520 mb

type GraphStorage struct {
	edgeInfos      []EdgeExtraInfo
	osmNodePoints  []Coordinate
	tagStringIDMap util.IDMap

	//  https://abseil.io/fast/hints.html#bit-vectors-instead-of-sets
	streetDirectionForward  *bitset.BitSet // kalau direction dari edgeId forward: bitset.Test(edgeId) return true
	streetDirectionBackward *bitset.BitSet
	roundaboutFlag          *bitset.BitSet
	nodeTrafficLight        *bitset.BitSet
}

func NewGraphStorage() *GraphStorage {
	return &GraphStorage{
		streetDirectionForward:  bitset.New(50000),
		streetDirectionBackward: bitset.New(50000),
		edgeInfos:               make([]EdgeExtraInfo, 0),
		tagStringIDMap:          util.NewIdMap(),
		roundaboutFlag:          bitset.New(5000),
		nodeTrafficLight:        bitset.New(5000),
		osmNodePoints:           make([]Coordinate, 0),
	}
}

func BuildGraphStorage(osmNodePoints []Coordinate, roundaboutFlag, nodeTrafficLight *bitset.BitSet,
	edgeInfos []EdgeExtraInfo, tagStringIDMap util.IDMap, streetDirectionForward, streetDirectionBackward *bitset.BitSet) *GraphStorage {
	return &GraphStorage{osmNodePoints: osmNodePoints, roundaboutFlag: roundaboutFlag,
		nodeTrafficLight: nodeTrafficLight, edgeInfos: edgeInfos, tagStringIDMap: tagStringIDMap,
		streetDirectionForward: streetDirectionForward, streetDirectionBackward: streetDirectionBackward}
}

func NewGraphStorageWithSize(numberOfEdges int, numberOfVertices int) *GraphStorage {
	return &GraphStorage{
		streetDirectionForward:  bitset.New(uint(numberOfEdges)),
		streetDirectionBackward: bitset.New(uint(numberOfEdges)),
		edgeInfos:               make([]EdgeExtraInfo, numberOfEdges),
		tagStringIDMap:          util.NewIdMap(),
		roundaboutFlag:          bitset.New(uint(numberOfEdges)),
		nodeTrafficLight:        bitset.New(uint(numberOfVertices)),
		osmNodePoints:           make([]Coordinate, 1),
	}
}

func (gs *GraphStorage) SetRoundabout(edgeID Index, isRoundabout bool) {
	if isRoundabout {
		gs.roundaboutFlag.Set(uint(edgeID))
	} else {
		gs.roundaboutFlag.Clear(uint(edgeID))
	}
}

func (gs *GraphStorage) SetStreetDirection(streetDirectionForward, streetDirectionBackward *bitset.BitSet) {
	gs.streetDirectionForward = streetDirectionForward
	gs.streetDirectionBackward = streetDirectionBackward
}

func (gs *GraphStorage) SetTagStringIdMap(tagStringIDMap util.IDMap) {
	gs.tagStringIDMap = tagStringIDMap
}

func (gs *GraphStorage) GetStreetDirection(edgeId Index) [2]bool {
	var direction [2]bool
	forward := gs.streetDirectionForward.Test(uint(edgeId))
	direction[0] = forward
	backward := gs.streetDirectionBackward.Test(uint(edgeId))
	direction[1] = backward
	return direction

}

func (gs *GraphStorage) SetTrafficLight(nodeID Index, yes bool) {
	if yes {
		gs.nodeTrafficLight.Set(uint(nodeID))
	} else {
		gs.nodeTrafficLight.Clear(uint(nodeID))
	}
}

func (gs *GraphStorage) GetTrafficLight(nodeID Index) bool {

	return gs.nodeTrafficLight.Test(uint(nodeID))
}

type EdgeExtraInfo struct {
	osmWayId         int64
	startPointsIndex Index // edge geometry start index di gs.osmNodePoints
	endPointsIndex   Index
	streetName       int
	roadClass        int
	roadClassLink    int
	lanes            uint8
}

func NewEdgeExtraInfo(streetName int, roadClass, roadClassLink int, lanes uint8, StartPointsIdx, EndPointsIdx Index,
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
	edge := gs.edgeInfos[edgeID]
	if edge.osmWayId == INVALID_OSM_WAY_ID {
		return make([]Coordinate, 0)
	}
	startIndex := edge.startPointsIndex
	endIndex := edge.endPointsIndex
	return gs.GetOsmNodePoints(startIndex, endIndex)
}

func (gs *GraphStorage) GetOsmNodePoints(startIndex, endIndex Index) []Coordinate {

	if startIndex < endIndex {
		return gs.osmNodePoints[startIndex:endIndex]
	}

	if startIndex <= 0 {
		return make([]Coordinate, 0)
	}

	// reversed road segment
	// di road network osm ada beberapa osm way yang two way
	// nah ini edge geometry yang reversed direction
	// daripada simpan edge geometry untuk setiap direction untuk edge yang sama, kita simpan satu edge geometry saja untuk kedua arah
	// bisa hemat lebih banyak space
	edgePoints := make([]Coordinate, 0, startIndex-endIndex)
	for i := int(startIndex - 1); i >= int(endIndex); i-- { // harus int(), karena kalo gak, endIndex == 0, next iteration jd maxuint32
		edgePoints = append(edgePoints, gs.osmNodePoints[i])
	}

	return edgePoints
}

// return edgeExtraInfo, isRoundabout
func (gs *GraphStorage) GetEdgeExtraInfo(edgeID Index, reverse bool) (EdgeExtraInfo, bool) {

	roundabout := gs.roundaboutFlag.Test(uint(edgeID))
	return gs.edgeInfos[edgeID], roundabout
}

func (gs *GraphStorage) AppendOsmNodePoints(edgePoints []Coordinate) {
	gs.osmNodePoints = append(gs.osmNodePoints, edgePoints...)
}

func (gs *GraphStorage) AppendEdgeInfos(edgeInfo EdgeExtraInfo) {
	gs.edgeInfos = append(gs.edgeInfos, edgeInfo)
}

func (gs *GraphStorage) SetEdgeInofs(edgeInfos []EdgeExtraInfo) {
	gs.edgeInfos = edgeInfos
}

func (gs *GraphStorage) GetOsmNodePointsCount() int {
	return len(gs.osmNodePoints)
}

func (gs *GraphStorage) FlattenIdMap() {
	gs.tagStringIDMap.ToStringArray()
}
