package routing

import "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

type vertexEdgePair struct {
	vertex      datastructure.Index
	edge        datastructure.Index
	isOutEdge   bool
	outInEdgeId datastructure.Index
	tSec        float64
	queryLevel  uint8
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

func (ve *vertexEdgePair) setTime(tSec float64) {
	ve.tSec = tSec
}

func (ve *vertexEdgePair) getTime() float64 {
	return ve.tSec
}

func (ve *vertexEdgePair) setVertex(vertex datastructure.Index) {
	ve.vertex = vertex
}

func (ve *vertexEdgePair) setisOutEdge(is bool) {
	ve.isOutEdge = is
}

func (ve *vertexEdgePair) isOut() bool {
	return ve.isOutEdge
}

func (ve *vertexEdgePair) setQueryLevel(queryLevel uint8) {
	ve.queryLevel = queryLevel
}

func (ve *vertexEdgePair) getQueryLevel() uint8 {
	return ve.queryLevel
}

func newVertexEdgePair(vertex, edge datastructure.Index, isOutEdge bool) vertexEdgePair {
	return vertexEdgePair{
		vertex:    vertex,
		edge:      edge,
		isOutEdge: isOutEdge,
	}
}

func newVertexEdgePairWithOutEdgeId(vertex, edge, outInEdgeId datastructure.Index, isOutEdge bool) vertexEdgePair {
	return vertexEdgePair{
		vertex:      vertex,
		edge:        edge,
		isOutEdge:   isOutEdge,
		outInEdgeId: outInEdgeId,
	}
}

func (ve *vertexEdgePair) getOutInEdgeId() datastructure.Index {
	return ve.outInEdgeId
}

type VertexInfo struct {
	travelTime float64
	parent     vertexEdgePair
}

func NewVertexInfo(travelTime float64, parent vertexEdgePair) VertexInfo {
	return VertexInfo{
		travelTime: travelTime,
		parent:     parent,
	}
}

func (vi VertexInfo) GetTravelTime() float64 {
	return vi.travelTime
}

func (vi VertexInfo) GetParent() vertexEdgePair {
	return vi.parent
}

type altVertexPair struct {
	vertex    datastructure.Index
	outEdgeId datastructure.Index
}

func (a altVertexPair) getVertex() datastructure.Index {
	return a.vertex
}

func (a altVertexPair) getOutEdge() datastructure.Index {
	return a.outEdgeId
}

func NewAltVertexPair(vertexId, outEdgeId datastructure.Index) altVertexPair {
	return altVertexPair{vertex: vertexId, outEdgeId: outEdgeId}
}

type altVertexInfo struct {
	travelTime float64
	parent     altVertexPair
}

func NewAltVertexInfo(travelTime float64, parent altVertexPair) altVertexInfo {
	return altVertexInfo{travelTime: travelTime, parent: parent}
}

func (a altVertexInfo) getTravelTime() float64 {
	return a.travelTime
}

func (a altVertexInfo) getParent() altVertexPair {
	return a.parent
}
