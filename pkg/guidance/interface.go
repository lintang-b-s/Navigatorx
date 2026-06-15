package guidance

import da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

type Graph interface {
	GetTailOfOutedge(e da.Index) da.Index
	GetVertex(u da.Index) da.Vertex
	GetVertexCoordinate(u da.Index) da.Coordinate
	GetOutEdge(eId da.Index) *da.OutEdge

	IsRoundabout(edgeId da.Index) bool
	GetStreetName(edgeId da.Index) string
	GetEdgeGeometry(edgeID da.Index) []da.Coordinate
	GetEdgeGeometryLength(edgeID da.Index) int
	GetEdgeGeometryPoint(edgeID da.Index, point int) da.Coordinate
	AppendEdgeGeometryWithoutLast(path *da.Coordinates, edgeID da.Index)
	GetOutEdgeBounds(u da.Index) (da.Index, da.Index)
	IsTraversableOutEdge(e da.Index) bool
	GetRoadClass(edgeId da.Index) string
	ForOutEdgeIdsOf(u da.Index, handle func(eId da.Index))
	GetRoadClassLink(edgeId da.Index) string
	GetStreetDirection(edgeId da.Index) [2]bool
	GetRoadLanes(edgeId da.Index) uint8
	GetInEdgeBounds(v da.Index) (da.Index, da.Index)
	IsTraversableInEdge(e da.Index) bool
	GetHeadFromInEdge(entryId da.Index) da.Index
	IsTrafficLight(vertexId da.Index) bool
	IsDummyOutEdge(eId da.Index) bool
	GetHeadOfOutEdge(e da.Index) da.Index
	GetTailOfInedge(e da.Index) da.Index
	IsStreetBidirectional(edgeId da.Index) bool
	GetExitIdOfInEdge(e da.Index) da.Index
	GetOsmWayId(edgeId da.Index) int64
	GetStrFromId(stNameId uint32) string
	GetStreetNameId(edgeId da.Index) uint32
	IsCurved(edgeId da.Index) bool
}

type RoutingEngine interface {
	GetGraph() *da.Graph
	PathExists(u, v da.Index) bool
	GetWeightSeconds(eId da.Index, outEdge bool) float64
	GetSegmentSpeed(eId da.Index, outEdge bool) float64
	GetSegmentLength(eId da.Index, outEdge bool) float64
	GetWeightFromLength(eId da.Index, outEdge bool, eLength float64) float64
	IsDummyOutEdge(eId da.Index) bool
	IsDummyInEdge(eId da.Index) bool
}
