package guidance

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type Graph interface {
	GetVertex(u da.Index) da.Vertex
	GetVertexCoordinate(u da.Index) da.Coordinate
	ForOutEdgeIdsOf(u da.Index, handle func(eId da.Index))
	ForOutEdgesOf(u da.Index, handle func(eId, head da.Index, entryPoint da.Index))
	ForInEdgesOf(v da.Index, handle func(eId, tail da.Index, exitPoint da.Index))
	GetHead(e da.Index) da.Index
	GetTail(e da.Index) da.Index
	GetOutId(e da.Index) da.Index
}

type RoadNetworkDataContainer interface {
	IsRoundabout(segmentId da.Index) bool
	GetStreetName(segmentId da.Index) string
	GetSegmentGeometry(edgeID da.Index) []da.Coordinate
	GetRoadClass(segmentId da.Index) pkg.OsmHighwayType
	GetRoadClassLink(segmentId da.Index) pkg.OsmHighwayType
	GetStreetDirection(segmentId da.Index) [2]bool
	GetStreetNameId(id da.Index) uint32
	GetOsmWayId(segmentId da.Index) uint64
	GetStrFromId(stNameId uint32) string
	IsCurved(segmentId da.Index) bool
	GetRoadLanes(segmentId da.Index) uint8
	IsStreetBidirectional(segmentId da.Index) bool
	GetSegmentTailCoord(id da.Index) da.Coordinate
	GetSegmentHeadCoord(id da.Index) da.Coordinate
	GetSegmentGeometryPoint(id da.Index, point int) da.Coordinate
	GetSegmentGeometryLength(id da.Index) da.Index
	GetTailHeadOsmNodeId(id da.Index) (uint64, uint64)
	IsParallelVia(id da.Index) bool
}

type RoutingEngine interface {
	GetGraph() *da.Graph
	PathExists(u, v da.Index) bool
	GetDurationSeconds(segId da.Index) float64
	GetSegmentSpeed(segId da.Index) float64
	GetSegmentLength(segId da.Index) float64
}
