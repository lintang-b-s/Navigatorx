package guidance

import (
	"iter"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

// Graph represents graph.
type Graph interface {
	GetVertex(u da.Index) da.Vertex
	GetVertexCoordinate(u da.Index) da.Coordinate
	NumEdges(u da.Index, out bool) (da.Index, da.Index)
	GetHead(e da.Index) da.Index
	GetTail(e da.Index) da.Index
	GetOutId(e da.Index) da.Index
}

// RoadNetworkDataContainer represents road network data container.
type RoadNetworkDataContainer interface {
	IsRoundabout(segmentId da.Index) bool
	GetStreetName(segmentId da.Index) string
	GetSegmentGeometry(edgeId da.Index) []da.Coordinate
	GetRoadClass(segmentId da.Index) pkg.OsmHighwayType
	GetRoadClassLink(segmentId da.Index) pkg.OsmHighwayType
	StreetDirection(segmentId da.Index) [2]bool
	GetStreetNameId(id da.Index) uint32
	GetOsmWayId(segmentId da.Index) uint64
	GetStrFromId(stNameId uint32) string
	IsCurved(segmentId da.Index) bool
	GetRoadLanes(segmentId da.Index) uint8
	IsStreetBidirectional(segmentId da.Index) bool
	GetSegmentTailCoord(id da.Index) da.Coordinate
	GetSegmentHeadCoord(id da.Index) da.Coordinate
	GetSegmentGeometryPoint(id da.Index, point int) da.Coordinate
	ForSegmentGeometry(id da.Index) iter.Seq[da.Coordinate]
	YieldRouteCoordinates(segmentIdPath []da.Index, yield func(da.Coordinate) bool)
	GetSegmentGeometryLength(id da.Index) da.Index
	GetTailHeadOsmNodeId(id da.Index) (uint64, uint64)
	IsParallelVia(id da.Index) bool
}

// RoutingEngine represents routing engine.
type RoutingEngine interface {
	GetGraph() *da.Graph
	PathExists(u, v da.Index) bool
}

// Metrics represents metrics.
type Metrics interface {
	GetDurationSeconds(segId da.Index) float64
	GetSegmentSpeed(segId da.Index) float64
	GetSegmentLength(segId da.Index) float64
}
