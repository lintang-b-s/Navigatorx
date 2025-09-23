package usecases

import (
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
)

type RoutingEngine interface {
	GetGraph() *datastructure.Graph
	GetHaversineDistanceFromUtoV(u, v datastructure.Index) float64 
	GetVertexCoordinatesFromOutEdge(u datastructure.Index) (float64, float64)
	GetVertexCoordinatesFromInEdge(u datastructure.Index) (float64, float64)
	VerticeUandVAreConnected(u, v datastructure.Index) bool
}

type SpatialIndex interface {
	SearchWithinRadius(float64, float64, float64) []spatialindex.ArcEndpoint
}
