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
	ForOutEdgeIdsOf(u da.Index, handle func(eId da.Index))
	GetRoadClass(edgeId da.Index) string
	GetRoadClassLink(edgeId da.Index) string
	GetStreetDirection(edgeId da.Index) [2]bool
	GetRoadLanes(edgeId da.Index) uint8
	ForInEdgeIdsOf(v da.Index, handle func(eId da.Index))
	GetHeadFromInEdge(entryId da.Index) da.Index
	IsTrafficLight(vertexId da.Index) bool
	IsDummyOutEdge(eId da.Index) bool
	GetHeadOfOutEdge(e da.Index) da.Index
	GetTailOfInedge(e da.Index) da.Index
	GetOutEdgeLength(e da.Index) float64
	GetOutEdgeWeight(e da.Index) float64
	IsStreetBidirectional(edgeId da.Index) bool
	GetExitIdOfInEdge(e da.Index) da.Index
}
