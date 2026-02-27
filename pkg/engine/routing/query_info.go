package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type vertexEdgePair struct {
	vertex      da.Index
	edge        da.Index
	isOutEdge   bool
	outInEdgeId da.Index
	queryLevel  uint8
}

func (ve *vertexEdgePair) getEdge() da.Index {
	return ve.edge
}

func (ve *vertexEdgePair) getVertex() da.Index {
	return ve.vertex
}

func (ve *vertexEdgePair) setEdge(edge da.Index) {
	ve.edge = edge
}

func (ve *vertexEdgePair) setVertex(vertex da.Index) {
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

func (ve *vertexEdgePair) getFirstOverlayEntryExitId() da.Index {
	firstEntryExitId := ve.outInEdgeId
	return firstEntryExitId
}

func (ve *vertexEdgePair) setFirstOverlayEntryExitId(vEntryExitId da.Index) {
	ve.outInEdgeId = vEntryExitId
}

func (ve *vertexEdgePair) isFirstOverlayVertex() bool {
	return ve.outInEdgeId != da.INVALID_EDGE_ID
}

func newVertexEdgePair(vertex, edge da.Index, isOutEdge bool) vertexEdgePair {
	return vertexEdgePair{
		vertex:      vertex,
		edge:        edge,
		isOutEdge:   isOutEdge,
		outInEdgeId: da.INVALID_EDGE_ID,
	}
}

func newVertexEdgePairWithOutEdgeId(vertex, edge, outInEdgeId da.Index, isOutEdge bool) vertexEdgePair {
	return vertexEdgePair{
		vertex:      vertex,
		edge:        edge,
		isOutEdge:   isOutEdge,
		outInEdgeId: outInEdgeId,
	}
}

func (ve *vertexEdgePair) getOutInEdgeId() da.Index {
	return ve.outInEdgeId
}

type VertexInfo[T comparable] struct {
	travelTime float64
	parent     vertexEdgePair
	scanned    bool // scanned or est dist from s to this v is equal to shortest path cost, and contained in shortest path tree
	heapNode   *da.PriorityQueueNode[T]
}

func NewVertexInfo[T comparable](travelTime float64, parent vertexEdgePair, hnode *da.PriorityQueueNode[T]) *VertexInfo[T] {
	return &VertexInfo[T]{
		travelTime: travelTime,
		parent:     parent,
		heapNode:   hnode,
	}
}

func (vi *VertexInfo[T]) GetTravelTime() float64 {
	return vi.travelTime
}

func (vi *VertexInfo[T]) UpdateTravelTime(tt float64) {
	vi.travelTime = tt
}

func (vi *VertexInfo[T]) UpdateParent(par vertexEdgePair) {
	vi.parent = par
}

func (vi *VertexInfo[T]) Scan() {
	vi.scanned = true
}

func (vi *VertexInfo[T]) IsScanned() bool {
	return vi.scanned
}

func (vi *VertexInfo[T]) GetParent() vertexEdgePair {
	return vi.parent
}

func (vi *VertexInfo[T]) GetHeapNode() *da.PriorityQueueNode[T] {
	return vi.heapNode
}

func initInfWeightVertexInfo[T comparable](vs []*VertexInfo[T]) {
	for i := range vs {
		vs[i] = NewVertexInfo[T](pkg.INF_WEIGHT, newVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false), nil)
	}
}

type altVertexPair struct {
	vertex    da.Index
	outEdgeId da.Index
}

func (a altVertexPair) getVertex() da.Index {
	return a.vertex
}

func (a altVertexPair) getOutEdge() da.Index {
	return a.outEdgeId
}

func NewAltVertexPair(vertexId, outEdgeId da.Index) altVertexPair {
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

type pathUnpackingParam struct {
	sourceOverlayId  da.Index
	targetOverlayId  da.Index
	level            uint8
	unpackedEdgePath *[]da.OutEdge
}

func NewPathUnpackingParam(sourceOverlayId, targetOverlayId da.Index, level uint8, unpackedEdgePath *[]da.OutEdge) pathUnpackingParam {

	return pathUnpackingParam{sourceOverlayId: sourceOverlayId, targetOverlayId: targetOverlayId, level: level, unpackedEdgePath: unpackedEdgePath}
}

func (p pathUnpackingParam) getSourceOverlayId() da.Index {
	return p.sourceOverlayId
}

func (p pathUnpackingParam) getTargetOverlayId() da.Index {
	return p.targetOverlayId
}

func (p pathUnpackingParam) getLevel() uint8 {
	return p.level
}

func (p pathUnpackingParam) getUnpackedEdgePath() *[]da.OutEdge {
	return p.unpackedEdgePath
}
