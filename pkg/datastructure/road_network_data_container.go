package datastructure

import (
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// RoadNetworkDataContainer stores annotation and supplementary information for the osm road network  data.
// todo: add ref, destination, destination:ref storage
type RoadNetworkDataContainer struct {
	// di set sebelum buildGraph()
	osmNodePoints []Coordinate // geometry of each road segments, flatenned.
	osmNodeIds    *PackedSlice

	nameTable []string // map dari integer ke string (tag name di osm way)
	// dua ini di set sebelum buildGraph()

	// conditional restrictions
	conditionalBarrierNodes     []ConditionalBarrierNode
	conditionalReversibleEdges  []ConditionalReversibleEdge
	conditionalSpeedLimits      []ConditionalSpeedLimit
	conditionalTrafficModes     []ConditionalTrafficMode
	conditionalTurnRestrictions []ConditionalTurnRestriction

	// annotation data dari road segments
	// diset saat osmparser
	segmentTurnLanesData    []TurnLanesData // segmentTurnLanesData[i] is the turnLanesData for road segment/edge with id=i
	segmentOsmWayId         *PackedSlice    // map dari segment id ke osm way id dari edge
	segmentStartPointsIndex []Index
	segmentEndPointsIndex   []Index
	streetName              []uint32
	roadClass               []pkg.OsmHighwayType
	roadClassLink           []pkg.OsmHighwayType
	lanes                   []uint8
	osmwayBitSize           uint8

	segmentFlags []SegmentFlagType

	boundingBox *BoundingBox
}

// NewRoadNetworkDataContainer creates and returns road network data container.
func NewRoadNetworkDataContainer(osmwayBitSize uint8) *RoadNetworkDataContainer {

	return &RoadNetworkDataContainer{
		osmNodePoints:           make([]Coordinate, 0),
		osmNodeIds:              NewPackedSlice(BitSizeOSMNodeId, InitialApproxSegmentSize),
		segmentOsmWayId:         NewPackedSlice(osmwayBitSize, InitialApproxSegmentSize), // ini 41 bit aja, buat eval map matching dataset newson 41 bit setiap eId
		osmwayBitSize:           osmwayBitSize,
		segmentStartPointsIndex: make([]Index, 0),
		segmentEndPointsIndex:   make([]Index, 0),
		streetName:              make([]uint32, 0),
		roadClass:               make([]pkg.OsmHighwayType, 0),
		roadClassLink:           make([]pkg.OsmHighwayType, 0),
		lanes:                   make([]uint8, 0),
		segmentFlags:            make([]SegmentFlagType, 0),
	}
}

// NewRoadNetworkDataContainerWithSize creates and returns road network data container with size.
func NewRoadNetworkDataContainerWithSize(numberOfEdges int, _ int) *RoadNetworkDataContainer {
	return &RoadNetworkDataContainer{
		osmNodePoints:           make([]Coordinate, 1),
		segmentOsmWayId:         NewPackedSlice(DefaultBitSizeOSMWayId, uint64(numberOfEdges)),
		segmentStartPointsIndex: make([]Index, 0),
		osmNodeIds:              NewPackedSlice(BitSizeOSMNodeId, InitialApproxSegmentSize),
		segmentEndPointsIndex:   make([]Index, 0),
		streetName:              make([]uint32, 0),
		osmwayBitSize:           DefaultBitSizeOSMWayId,
		roadClass:               make([]pkg.OsmHighwayType, 0),
		roadClassLink:           make([]pkg.OsmHighwayType, 0),
		lanes:                   make([]uint8, 0),
	}
}

// IsSegmentFlagBitOn reports whether segment flag bit on.
func (rn *RoadNetworkDataContainer) IsSegmentFlagBitOn(id Index, mask SegmentFlagType) bool {
	return rn.segmentFlags[id]&mask != 0
}

// IsRoundabout reports whether road segment id is a roundabout.
func (rn *RoadNetworkDataContainer) IsRoundabout(id Index) bool {
	return rn.IsSegmentFlagBitOn(id, FlagIsRoundabout)
}

// IsCurved reports whether road segment id is curved.
func (rn *RoadNetworkDataContainer) IsCurved(id Index) bool {
	return rn.IsSegmentFlagBitOn(id, FlagIsCurved)
}

// GetOsmwayBitSize returns osmway bit size.
func (rn *RoadNetworkDataContainer) GetOsmwayBitSize() uint8 {
	return rn.osmwayBitSize
}

// SetSegmentBit sets road segment bit.
func (rn *RoadNetworkDataContainer) SetSegmentBit(id Index, mask SegmentFlagType, on bool) {
	rn.padSegmentflags(id)
	if on {
		rn.segmentFlags[id] |= mask
		return
	}
	rn.segmentFlags[id] &= ^mask
}

// SetSegmentFlag sets road segment flag.
func (rn *RoadNetworkDataContainer) SetSegmentFlag(id Index, mask SegmentFlagType) {
	rn.padSegmentflags(id)
	rn.segmentFlags[id] = mask
}

func (rn *RoadNetworkDataContainer) padSegmentflags(id Index) {
	if len(rn.segmentFlags) <= int(id) {
		pad := int(id) - len(rn.segmentFlags) + 1
		ss := make([]SegmentFlagType, pad)
		rn.segmentFlags = append(rn.segmentFlags, ss...)
	}
}

// GetSegmentFlag returns road segment flag.
func (rn *RoadNetworkDataContainer) GetSegmentFlag(id Index) SegmentFlagType {
	return rn.segmentFlags[id]
}

// StreetDirection returns drection dari osm way nya id
func (rn *RoadNetworkDataContainer) StreetDirection(id Index) [2]bool {
	var direction [2]bool
	forward := rn.IsSegmentFlagBitOn(id, FlagIsForward)
	direction[0] = forward
	backward := rn.IsSegmentFlagBitOn(id, FlagIsBackward)
	direction[1] = backward
	return direction
}

// GetSegmentGeometryPoint return osm road segment geometry i-th point
func (rn *RoadNetworkDataContainer) GetSegmentGeometryPoint(id Index, i int) Coordinate {
	sIndex := rn.segmentStartPointsIndex[id]
	eIndex := rn.segmentEndPointsIndex[id]
	if sIndex < eIndex {
		return rn.osmNodePoints[int(sIndex)+i]
	}
	return rn.osmNodePoints[int(sIndex)-1-i]
}

// GetSegmentGeometryLength returns segment geometry length.
func (rn *RoadNetworkDataContainer) GetSegmentGeometryLength(id Index) Index {
	sIndex := rn.segmentStartPointsIndex[id]
	eIndex := rn.segmentEndPointsIndex[id]
	if sIndex < eIndex {
		return eIndex - sIndex
	}
	return sIndex - eIndex
}

// GetSegmentGeometry returns segment geometry.
func (rn *RoadNetworkDataContainer) GetSegmentGeometry(id Index) []Coordinate {
	sIndex := rn.segmentStartPointsIndex[id]
	eIndex := rn.segmentEndPointsIndex[id]

	if sIndex < eIndex {
		return rn.osmNodePoints[sIndex:eIndex]
	}

	// reversed road segment
	// di road network osm ada beberapa osm way yang two way
	// nah ini edge geometry yang reversed direction
	// daripada simpan edge geometry untuk setiap direction untuk edge yang sama, kita simpan satu edge geometry saja untuk kedua arah
	// bisa hemat lebih banyak space
	edgePoints := make([]Coordinate, 0, sIndex-eIndex)
	for i := int(sIndex - 1); i >= int(eIndex); i-- { // harus int(), karena kalo gak, eIndex == 0, next iteration jd maxuint32
		edgePoints = append(edgePoints, rn.osmNodePoints[i])
	}
	return edgePoints
}

// AppendSegmentGeometry appends segment geometry.
func (rn *RoadNetworkDataContainer) AppendSegmentGeometry(id Index, coords *Coordinates) {
	sIndex := rn.segmentStartPointsIndex[id]
	eIndex := rn.segmentEndPointsIndex[id]

	if sIndex < eIndex {
		coords.Append(rn.osmNodePoints[sIndex:eIndex])
		return
	}

	for i := int(sIndex - 1); i >= int(eIndex); i-- {
		coords.AppendCoordinate(rn.osmNodePoints[i])
	}
}

// GetTailHeadOsmNodeId returns tail head osm node id.
func (rn *RoadNetworkDataContainer) GetTailHeadOsmNodeId(id Index) (uint64, uint64) {
	sIndex := rn.segmentStartPointsIndex[id]
	eIndex := rn.segmentEndPointsIndex[id]
	if sIndex < eIndex {
		return rn.osmNodeIds.Get(uint64(sIndex)), rn.osmNodeIds.Get(uint64(eIndex) - 1)
	}
	return rn.osmNodeIds.Get(uint64(sIndex - 1)), rn.osmNodeIds.Get(uint64(eIndex))
}

// GetSegmentOsmNodeIds returns segment osm node ids.
func (rn *RoadNetworkDataContainer) GetSegmentOsmNodeIds(id Index) []uint64 {
	sIndex := rn.segmentStartPointsIndex[id]
	eIndex := rn.segmentEndPointsIndex[id]

	if sIndex < eIndex {
		osmnids := make([]uint64, 0, eIndex-sIndex)
		for i := sIndex; i < eIndex; i++ {
			v := rn.osmNodeIds.Get(uint64(i))
			osmnids = append(osmnids, v)
		}
		return osmnids
	}

	nPoints := make([]uint64, 0, sIndex-eIndex)
	for i := int(sIndex - 1); i >= int(eIndex); i-- { // harus int(), karena kalo gak, eIndex == 0, next iteration jd maxuint64
		v := rn.osmNodeIds.Get(uint64(i))
		nPoints = append(nPoints, v)
	}
	return nPoints
}

// GetStrFromId returns str from id.
func (rn *RoadNetworkDataContainer) GetStrFromId(id uint32) string {
	return rn.nameTable[id]
}

// GetSegmentTailCoord returns coordinate of tail node u of road segment/edge (u,v)
func (rn *RoadNetworkDataContainer) GetSegmentTailCoord(id Index) Coordinate {
	return rn.GetSegmentGeometryPoint(id, 0)
}

// GetSegmentHeadCoord returns coordinate of head node v of road segment/edge (u,v)
func (rn *RoadNetworkDataContainer) GetSegmentHeadCoord(id Index) Coordinate {
	l := int(rn.GetSegmentGeometryLength(id))
	return rn.GetSegmentGeometryPoint(id, l-1)
}

// GetSegmentGeometryEndpoints returns segment geometry endpoints.
func (rn *RoadNetworkDataContainer) GetSegmentGeometryEndpoints(id Index) (Index, Index) {
	sIndex := rn.segmentStartPointsIndex[id]
	eIndex := rn.segmentEndPointsIndex[id]
	return sIndex, eIndex
}

// AppendOsmNodePoints appends osm node points.
func (rn *RoadNetworkDataContainer) AppendOsmNodePoints(edgePoints []Coordinate, osmNodeIds []uint64) {
	rn.osmNodePoints = append(rn.osmNodePoints, edgePoints...)
	for _, v := range osmNodeIds {
		rn.osmNodeIds.Append(v)
	}
}

// AppendSegmentData append road segment data
func (rn *RoadNetworkDataContainer) AppendSegmentData(osmWayId int64,
	startPointsIndex, // edge geometry start index di rn.osmNodePoints
	endPointsIndex Index,
	streetName uint32,
	roadClass, roadClassLink pkg.OsmHighwayType,
	lanes uint8, tld TurnLanesData) {
	rn.segmentOsmWayId.Append(uint64(osmWayId))
	rn.segmentStartPointsIndex = append(rn.segmentStartPointsIndex, startPointsIndex)
	rn.segmentEndPointsIndex = append(rn.segmentEndPointsIndex, endPointsIndex)
	rn.streetName = append(rn.streetName, streetName)
	rn.roadClass = append(rn.roadClass, roadClass)
	rn.roadClassLink = append(rn.roadClassLink, roadClassLink)
	rn.lanes = append(rn.lanes, lanes)
	rn.segmentTurnLanesData = append(rn.segmentTurnLanesData, tld)
}

// GetOsmNodePointsCount returns osm node points count.
func (rn *RoadNetworkDataContainer) GetOsmNodePointsCount() int {
	return len(rn.osmNodePoints)
}

// BuildNameTable builds name table.
func (rn *RoadNetworkDataContainer) BuildNameTable(idToStr map[uint32]string) {
	keys := make([]uint32, 0, len(idToStr))
	for key := range idToStr {
		keys = append(keys, key)
	}

	sort.Slice(keys, func(i, j int) bool {
		return i < j
	})

	rn.nameTable = make([]string, len(keys))
	for i := 0; i < len(keys); i++ {
		key := keys[i]
		rn.nameTable[key] = idToStr[key]
	}
}

// GetStr returns str of name.
func (rn *RoadNetworkDataContainer) GetStr(id uint32) string {
	return rn.nameTable[id]
}

// GetOsmWayId returns osm way id.
func (rn *RoadNetworkDataContainer) GetOsmWayId(id Index) uint64 {
	return rn.segmentOsmWayId.Get(uint64(id))
}

// GetStreetName returns street name.
func (rn *RoadNetworkDataContainer) GetStreetName(id Index) string {
	return rn.nameTable[rn.streetName[id]]
}

// GetStreetNameId returns street name id.
func (rn *RoadNetworkDataContainer) GetStreetNameId(id Index) uint32 {
	return rn.streetName[id]
}

// GetRoadClass returns road class.
func (rn *RoadNetworkDataContainer) GetRoadClass(id Index) pkg.OsmHighwayType {
	roadClassId := rn.roadClass[id]
	return roadClassId
}

// IsStreetBidirectional reports whether street bidirectional.
func (rn *RoadNetworkDataContainer) IsStreetBidirectional(segmentId Index) bool {
	dir := rn.StreetDirection(segmentId)
	return dir[0] && dir[1]
}

// GetRoadClassLink returns road class link.
func (rn *RoadNetworkDataContainer) GetRoadClassLink(id Index) pkg.OsmHighwayType {
	roadClassLinkId := rn.roadClassLink[id]
	return roadClassLinkId
}

// GetRoadLanes returns road lanes.
func (rn *RoadNetworkDataContainer) GetRoadLanes(id Index) uint8 {
	return rn.lanes[id]
}

// ValidTurnLane returns true if i-th lane of edge id allowed to turn type turnt.
func (rn *RoadNetworkDataContainer) ValidTurnLane(id Index, i int, turnt pkg.TurnLaneType) bool {
	return rn.segmentTurnLanesData[id].Valid(i, turnt)
}

// GetTurnLaneData returns turn lane data of road segment id.
func (rn *RoadNetworkDataContainer) GetTurnLaneData(id Index) TurnLanesData {
	return rn.segmentTurnLanesData[id]
}

// ApplySegmentsPermutation apply permutation to road segments annotation
// perm=permutation that maps from new edge annotation id to old edge id
// nPerm=permutation that maps from new vertex id to old vertex id
func (rn *RoadNetworkDataContainer) ApplySegmentsPermutation(nPerm []int) {

	m := Index(len(nPerm))
	// apply permutation edge annotation data
	rn.lanes = util.ApplyPermutation(rn.lanes, nPerm)
	rn.roadClass = util.ApplyPermutation(rn.roadClass, nPerm)
	rn.roadClassLink = util.ApplyPermutation(rn.roadClassLink, nPerm)
	rn.streetName = util.ApplyPermutation(rn.streetName, nPerm)
	rn.segmentStartPointsIndex = util.ApplyPermutation(rn.segmentStartPointsIndex, nPerm)
	rn.segmentEndPointsIndex = util.ApplyPermutation(rn.segmentEndPointsIndex, nPerm)
	newOsmWayIds := NewPackedSlice(rn.osmwayBitSize, uint64(m))
	for e := Index(0); e < m; e++ {

		oldE := Index(nPerm[e])
		newOsmWayIds.Append(rn.GetOsmWayId(oldE))
	}
	rn.segmentFlags = util.ApplyPermutation(rn.segmentFlags, nPerm)
	rn.segmentOsmWayId = newOsmWayIds

}

// IsParallelVia reports whether parallel via.
func (rn *RoadNetworkDataContainer) IsParallelVia(id Index) bool {
	return rn.segmentFlags[id]&FlagParallel != 0
}

// SetBoundingBox sets bounding box.
func (rn *RoadNetworkDataContainer) SetBoundingBox(bb *BoundingBox) {
	rn.boundingBox = bb
}

// GetBoundingBox returns bounding box.
func (rn *RoadNetworkDataContainer) GetBoundingBox() *BoundingBox {
	return rn.boundingBox
}

// ----  conditonal restrictions related ----

// SetConditionalBarrierNodes sets conditional barrier nodes.
func (rn *RoadNetworkDataContainer) SetConditionalBarrierNodes(nodes []ConditionalBarrierNode) {
	rn.conditionalBarrierNodes = nodes
}

// SetConditionalReversibleEdges sets conditional reversible edges.
func (rn *RoadNetworkDataContainer) SetConditionalReversibleEdges(ways []ConditionalReversibleEdge) {
	rn.conditionalReversibleEdges = ways
}

// SetConditionalSpeedLimits sets conditional speed limits.
func (rn *RoadNetworkDataContainer) SetConditionalSpeedLimits(limits []ConditionalSpeedLimit) {
	rn.conditionalSpeedLimits = limits
}

// SetConditionalTrafficModes sets conditional traffic modes.
func (rn *RoadNetworkDataContainer) SetConditionalTrafficModes(modes []ConditionalTrafficMode) {
	rn.conditionalTrafficModes = modes
}

// SetConditionalTurnRestrictions sets conditional turn restrictions.
func (rn *RoadNetworkDataContainer) SetConditionalTurnRestrictions(restrictions []ConditionalTurnRestriction) {
	rn.conditionalTurnRestrictions = restrictions
}

// GetConditionalBarrierNodes returns conditional barrier nodes.
func (rn *RoadNetworkDataContainer) GetConditionalBarrierNodes() []ConditionalBarrierNode {
	return rn.conditionalBarrierNodes
}

// GetConditionalReversibleEdges returns conditional reversible edges.
func (rn *RoadNetworkDataContainer) GetConditionalReversibleEdges() []ConditionalReversibleEdge {
	return rn.conditionalReversibleEdges
}

// GetConditionalSpeedLimits returns conditional speed limits.
func (rn *RoadNetworkDataContainer) GetConditionalSpeedLimits() []ConditionalSpeedLimit {
	return rn.conditionalSpeedLimits
}

// GetConditionalTrafficModes returns conditional traffic modes.
func (rn *RoadNetworkDataContainer) GetConditionalTrafficModes() []ConditionalTrafficMode {
	return rn.conditionalTrafficModes
}

// GetConditionalTurnRestrictions returns conditional turn restrictions.
func (rn *RoadNetworkDataContainer) GetConditionalTurnRestrictions() []ConditionalTurnRestriction {
	return rn.conditionalTurnRestrictions
}
