package datastructure

import "github.com/lintang-b-s/Navigatorx/pkg/util"

type VertexEdgePair struct {
	vertex      Index // 4 byte
	edge        Index // 4 byte
	outInEdgeId Index // 4 byte
	queryLevel  uint8 // 1 byte
}

func (ve VertexEdgePair) GetEdge() Index {
	return ve.edge
}

func (ve VertexEdgePair) GetVertex() Index {
	return ve.vertex
}

func (ve *VertexEdgePair) SetEdge(edge Index) {
	ve.edge = edge
}

func (ve *VertexEdgePair) SetVertex(vertex Index) {
	ve.vertex = vertex
}

func (ve *VertexEdgePair) SetQueryLevel(queryLevel uint8) {
	ve.queryLevel = queryLevel
}

func (ve VertexEdgePair) GetQueryLevel() uint8 {
	return ve.queryLevel
}

func (ve VertexEdgePair) GetFirstOverlayEntryExitId() Index {
	firstEntryExitId := ve.outInEdgeId
	return firstEntryExitId
}

func (ve *VertexEdgePair) SetFirstOverlayEntryExitId(vEntryExitId Index) {
	ve.outInEdgeId = vEntryExitId
}

func (ve VertexEdgePair) IsFirstOverlayVertex() bool {
	return ve.outInEdgeId != INVALID_EDGE_ID
}

func NewVertexEdgePair(vertex, edge Index, isOutEdge bool) VertexEdgePair {
	return VertexEdgePair{
		vertex:      vertex,
		edge:        edge,
		outInEdgeId: INVALID_EDGE_ID,
	}
}

func NewVertexEdgePairWithOutEdgeId(vertex, edge, outInEdgeId Index, isOutEdge bool) VertexEdgePair {
	return VertexEdgePair{
		vertex:      vertex,
		edge:        edge,
		outInEdgeId: outInEdgeId,
	}
}

func (ve VertexEdgePair) GetOutInEdgeId() Index {
	return ve.outInEdgeId
}

type VertexInfo[W util.RoutingNumber] struct {
	parent     VertexEdgePair // 13 byte
	travelTime W
	heapNodeId uint32 // 4 byte
}

func NewVertexInfo[W util.RoutingNumber](travelTime W, parent VertexEdgePair) VertexInfo[W] {
	return VertexInfo[W]{
		travelTime: travelTime,
		parent:     parent,
		heapNodeId: 0,
	}
}

func (vi *VertexInfo[W]) GetTravelTime() W {
	return vi.travelTime
}

func (vi *VertexInfo[W]) UpdateTravelTime(tt W) {
	vi.travelTime = tt
}

func (vi *VertexInfo[W]) UpdateParent(par VertexEdgePair) {
	vi.parent = par
}

func (vi *VertexInfo[W]) SetHeapNodeId(id uint32) {
	vi.heapNodeId = id
}

func (vi *VertexInfo[W]) GetHeapNodeId() uint32 {
	return vi.heapNodeId
}

func (vi *VertexInfo[W]) SetFirstOverlayEntryExitId(id Index) {
	vi.parent.SetFirstOverlayEntryExitId(id)
}

func (vi VertexInfo[W]) GetParent() VertexEdgePair {
	return vi.parent
}
