package datastructure

import "github.com/lintang-b-s/Navigatorx/pkg/util"

const (
	overlayFlag uint8 = 1 << iota
	cutEdgeFlag
)

type VertexEdgePair struct {
	vertex      Index // 4 byte
	edge        Index // 4 byte
	outInEdgeId Index // 4 byte
	queryLevel  uint8 // 1 byte
	flag        uint8
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

func (ve *VertexEdgePair) SetIsOverlayVertex() {
	ve.flag |= overlayFlag
}

func (ve *VertexEdgePair) IsOverlayVertex() bool {
	return ve.flag&overlayFlag != 0
}

func (ve *VertexEdgePair) SetIsCutEdge() {
	ve.flag |= cutEdgeFlag
}

func (ve *VertexEdgePair) IsCutEdge() bool {
	return ve.flag&cutEdgeFlag != 0
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

type VertexData[W util.RoutingNumber] struct {
	parent     VertexEdgePair // 13 byte
	cost       W
	heapNodeId uint32 // 4 byte
}

func NewVertexData[W util.RoutingNumber](cost W, parent VertexEdgePair) VertexData[W] {
	return VertexData[W]{
		cost:       cost,
		parent:     parent,
		heapNodeId: 0,
	}
}

func (vi *VertexData[W]) GetCost() W {
	return vi.cost
}

func (vi *VertexData[W]) UpdateCost(tt W) {
	vi.cost = tt
}

func (vi *VertexData[W]) UpdateParent(par VertexEdgePair) {
	vi.parent = par
}

func (vi *VertexData[W]) SetHeapNodeId(id uint32) {
	vi.heapNodeId = id
}

func (vi *VertexData[W]) GetHeapNodeId() uint32 {
	return vi.heapNodeId
}

func (vi VertexData[W]) GetParent() VertexEdgePair {
	return vi.parent
}
