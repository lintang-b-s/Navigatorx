package datastructure

import (
	"sort"

	"github.com/bits-and-blooms/bitset"
	"github.com/lintang-b-s/Navigatorx/pkg"
)

// bisa ngurangin banyak heap allocations
// ini pakai file osm diy_solo_jogja physical mem (RES): 1.6 gb -> sekarang 1.1 gb setelah gak pake pointer  buat graph []Vertex, []OutEdge, []InEdge, overlay graph []OverlayVertex dll
// -> sekarang setelah implement da.PackedSlice + add dummy edge untuk vertex dengan outdegree/indegree = 0: cuma 906mb
// osrm pakai file osm diy_solo_jogja physical mem (RES): cuma 520 mb
// todo: coba implement packed_vector  osrm (bikin lebih simple) buat simpan osm node Ids dan osm way Ids: https://github.com/Telenav/open-source-spec/blob/master/osrm/doc/packed_vector.md (DONE)
// https://wiki.openstreetmap.org/wiki/Stats: 2025 ada 10 billions osm node ids dan 500 jt osm way ids (DONE)
// bisa pake packed vector 34 bit untuk  store osm node id, dan 33 bit untuk osm way id ?
// todo2: coba implement name table osrm (bikin lebih simple): https://github.com/Telenav/open-source-spec/blob/master/osrm/doc/osrm-toolchain-files/map.osrm.names.md (ini gak usah)
// daripada slice of string di idmap.go, mungkin implement simplified name table bisa ngurangin space lebih banyak lagi, pakai single slice []byte/[]rune buat semua strings tapi ada offset & size utk setiap item?
// todo3: buat edgeInfos juga mending jadiin slice setiap field daripada slice of struct

type GraphStorage struct {
	osmNodePoints []Coordinate
	nameTable     []string // map dari integer ke string (tag name di osm way)

	//  https://abseil.io/fast/hints.html#bit-vectors-instead-of-sets
	streetDirectionForward  *bitset.BitSet // kalau direction dari edgeId forward: bitset.Test(edgeId) return true
	streetDirectionBackward *bitset.BitSet
	roundaboutFlag          *bitset.BitSet
	nodeTrafficLight        *bitset.BitSet

	// metadata dari edges
	edgeStartPointsIndex []Index
	edgeEndPointsIndex   []Index
	streetName           []uint32
	roadClass            []pkg.OsmHighwayType
	roadClassLink        []pkg.OsmHighwayType
	lanes                []uint8
	edgeOsmWayId         *PackedSlice // map dari outEdgeId ke osm way id dari edge
	osmwayBitSize        uint8
}

func NewGraphStorage(osmwayBitSize uint8) *GraphStorage {

	return &GraphStorage{
		streetDirectionForward:  bitset.New(INITIAL_BIT_VECTOR_SIZE),
		streetDirectionBackward: bitset.New(INITIAL_BIT_VECTOR_SIZE),
		roundaboutFlag:          bitset.New(INITIAL_BIT_VECTOR_SIZE),
		nodeTrafficLight:        bitset.New(INITIAL_BIT_VECTOR_SIZE),
		osmNodePoints:           make([]Coordinate, 0),
		edgeOsmWayId:            NewPackedSlice(osmwayBitSize), // ini 41 bit aja, buat eval map matching dataset newson 41 bit setiap eId
		osmwayBitSize:           osmwayBitSize,
		edgeStartPointsIndex:    make([]Index, 0),
		edgeEndPointsIndex:      make([]Index, 0),
		streetName:              make([]uint32, 0),
		roadClass:               make([]pkg.OsmHighwayType, 0),
		roadClassLink:           make([]pkg.OsmHighwayType, 0),
		lanes:                   make([]uint8, 0),
	}
}

func BuildGraphStorage(osmNodePoints []Coordinate, roundaboutFlag, nodeTrafficLight *bitset.BitSet,
	streetDirectionForward, streetDirectionBackward *bitset.BitSet) *GraphStorage {
	return &GraphStorage{osmNodePoints: osmNodePoints, roundaboutFlag: roundaboutFlag,
		nodeTrafficLight:       nodeTrafficLight,
		streetDirectionForward: streetDirectionForward, streetDirectionBackward: streetDirectionBackward}
}

func NewGraphStorageWithSize(numberOfEdges int, numberOfVertices int) *GraphStorage {
	return &GraphStorage{
		streetDirectionForward:  bitset.New(uint(numberOfEdges)),
		streetDirectionBackward: bitset.New(uint(numberOfEdges)),
		roundaboutFlag:          bitset.New(uint(numberOfEdges)),
		nodeTrafficLight:        bitset.New(uint(numberOfVertices)),
		osmNodePoints:           make([]Coordinate, 1),
		edgeOsmWayId:            NewPackedSlice(DEFAULT_BIT_SIZE_OSM_WAY_ID),
		edgeStartPointsIndex:    make([]Index, 0),
		edgeEndPointsIndex:      make([]Index, 0),
		streetName:              make([]uint32, 0),
		roadClass:               make([]pkg.OsmHighwayType, 0),
		roadClassLink:           make([]pkg.OsmHighwayType, 0),
		lanes:                   make([]uint8, 0)}
}

func (gs *GraphStorage) IsRoundabout(edgeId Index) bool {
	return gs.roundaboutFlag.Test(uint(edgeId))

}
func (gs *GraphStorage) SetRoundabout(edgeId Index, isRoundabout bool) {
	if isRoundabout {
		gs.roundaboutFlag.Set(uint(edgeId))
	} else {
		gs.roundaboutFlag.Clear(uint(edgeId))
	}
}

func (gs *GraphStorage) GetOsmwayBitSize() uint8 {
	return gs.osmwayBitSize
}

func (gs *GraphStorage) SetStreetDirection(streetDirectionForward, streetDirectionBackward *bitset.BitSet) {
	gs.streetDirectionForward = streetDirectionForward
	gs.streetDirectionBackward = streetDirectionBackward
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

func (gs *GraphStorage) GetEdgeGeometryLength(edgeID Index) int {

	startIndex := gs.edgeStartPointsIndex[edgeID]
	endIndex := gs.edgeEndPointsIndex[edgeID]
	if startIndex < endIndex {
		return int(endIndex) - int(startIndex)
	}
	return int(startIndex) - int(endIndex)
}

func (gs *GraphStorage) AppendPathWithEdgeGeometry(path *Coordinates, edgeID Index) {

	startIndex := gs.edgeStartPointsIndex[edgeID]
	endIndex := gs.edgeEndPointsIndex[edgeID]
	if startIndex < endIndex {
		path.Append(gs.osmNodePoints[startIndex:endIndex])
	}

	if startIndex == 0 {
		return
	}

	// reversed road segment
	// di road network osm ada beberapa osm way yang two way
	// nah ini edge geometry yang reversed direction
	// daripada simpan edge geometry untuk setiap direction untuk edge yang sama, kita simpan satu edge geometry saja untuk kedua arah
	// bisa hemat lebih banyak space
	slice := *path
	for i := int(startIndex - 1); i >= int(endIndex); i-- { // harus int(), karena kalo gak, endIndex == 0, next iteration jd maxuint32
		slice = append(slice, gs.osmNodePoints[i])
	}
	*path = slice
}

func (gs *GraphStorage) GetEdgeGeometry(edgeID Index) []Coordinate {

	startIndex := gs.edgeStartPointsIndex[edgeID]
	endIndex := gs.edgeEndPointsIndex[edgeID]
	return gs.GetOsmNodePoints(startIndex, endIndex)
}

func (gs *GraphStorage) GetOsmNodePoints(startIndex, endIndex Index) []Coordinate {

	if startIndex < endIndex {
		return gs.osmNodePoints[startIndex:endIndex]
	}

	if startIndex == 0 {
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

func (gs *GraphStorage) AppendOsmNodePoints(edgePoints []Coordinate) {
	gs.osmNodePoints = append(gs.osmNodePoints, edgePoints...)
}

func (gs *GraphStorage) AppendEdgeMetadata(osmWayId int64,
	startPointsIndex, // edge geometry start index di gs.osmNodePoints
	endPointsIndex Index,
	streetName uint32,
	roadClass, roadClassLink pkg.OsmHighwayType,
	lanes uint8) {
	gs.edgeOsmWayId.Append(uint64(osmWayId))
	gs.edgeStartPointsIndex = append(gs.edgeStartPointsIndex, startPointsIndex)
	gs.edgeEndPointsIndex = append(gs.edgeEndPointsIndex, endPointsIndex)
	gs.streetName = append(gs.streetName, streetName)
	gs.roadClass = append(gs.roadClass, roadClass)
	gs.roadClassLink = append(gs.roadClassLink, roadClassLink)
	gs.lanes = append(gs.lanes, lanes)
}

func (gs *GraphStorage) SetNewEdgeMetadata(osmWayIds *PackedSlice,
	edgeStartPointsIndex, // edge geometry start index di gs.osmNodePoints
	edgeEndPointsIndex []Index,
	streetName []uint32,
	roadClass, roadClassLink []pkg.OsmHighwayType,
	lanes []uint8) {
	gs.edgeOsmWayId = osmWayIds
	gs.edgeStartPointsIndex = edgeStartPointsIndex
	gs.edgeEndPointsIndex = edgeEndPointsIndex
	gs.streetName = streetName
	gs.roadClass = roadClass
	gs.roadClassLink = roadClassLink
	gs.lanes = lanes
}

func (gs *GraphStorage) GetOsmNodePointsCount() int {
	return len(gs.osmNodePoints)
}

func (gs *GraphStorage) BuildNameTable(idToStr map[uint32]string) {
	keys := make([]uint32, 0, len(idToStr))
	for key, _ := range idToStr {
		keys = append(keys, key)
	}

	sort.Slice(keys, func(i, j int) bool {
		return i < j
	})

	gs.nameTable = make([]string, len(keys))
	for i := 0; i < len(keys); i++ {
		key := keys[i]
		gs.nameTable[key] = idToStr[key]
	}
}

func (gs *GraphStorage) GetStr(id uint32) string {
	return gs.nameTable[id]
}
