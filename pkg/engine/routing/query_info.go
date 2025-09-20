package routing

import "github.com/lintang-b-s/navigatorx-crp/pkg/datastructure"

type vertexEdgePair struct {
	vertex datastructure.Index
	edge   datastructure.Index
}

func (ve *vertexEdgePair) getEdge() datastructure.Index {
	return ve.edge
}

func (ve *vertexEdgePair) getVertex() datastructure.Index {
	return ve.vertex
}

func (ve *vertexEdgePair) setEdge(edge datastructure.Index) {
	ve.edge = edge
}

func (ve *vertexEdgePair) setVertex(vertex datastructure.Index){
	ve.vertex = vertex
}

func newVertexEdgePair(vertex, edge datastructure.Index) vertexEdgePair {
	return vertexEdgePair{
		vertex: vertex,
		edge:   edge,
	}
}

type VertexInfo struct {
	eta    float64
	parent vertexEdgePair
}

func NewVertexInfo(eta float64, parent vertexEdgePair) VertexInfo {
	return VertexInfo{
		eta:    eta,
		parent: parent,
	}
}

func (vi VertexInfo) GetEta() float64 {
	return vi.eta
}

func (vi VertexInfo) GetParent() vertexEdgePair {
	return vi.parent
}
