package guidance

import "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

type Graph interface {
	GetTailOfOutedge(e datastructure.Index) datastructure.Index
	GetVertex(u datastructure.Index) *datastructure.Vertex
	IsRoundabout(edgeId datastructure.Index) bool
	GetStreetName(edgeId datastructure.Index) string
	ForOutEdgesOfWithId(u datastructure.Index, handle func(e *datastructure.OutEdge, id datastructure.Index))
	GetRoadClass(edgeId datastructure.Index) string
	GetRoadClassLink(edgeId datastructure.Index) string
	GetStreetDirection(edgeId datastructure.Index) [2]bool
	GetRoadLanes(edgeId datastructure.Index) uint8
	ForInEdgesOfWithId(v datastructure.Index, handle func(e *datastructure.InEdge, id datastructure.Index))
	GetHeadFromInEdge(entryPoint datastructure.Index) datastructure.Index
	IsTrafficLight(vertexId datastructure.Index) bool
}
