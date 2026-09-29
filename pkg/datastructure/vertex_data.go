package datastructure

import "github.com/lintang-b-s/Navigatorx/pkg/util"

const (
	overlayFlag uint8 = 1 << iota
	cutEdgeFlag
)

type ParentVertex struct {
	vertex     Index // 4 byte
	queryLevel uint8 // 1 byte
	flag       uint8
}

func (ve ParentVertex) GetVertex() Index {
	return ve.vertex
}

func (ve *ParentVertex) SetIsOverlayVertex() {
	ve.flag |= overlayFlag
}

func (ve *ParentVertex) IsOverlayVertex() bool {
	return ve.flag&overlayFlag != 0
}

func (ve *ParentVertex) SetVertex(vertex Index) {
	ve.vertex = vertex
}

func NewParentVertex(vertex Index) ParentVertex {
	return ParentVertex{
		vertex: vertex,
	}
}

type VertexData[W util.RoutingNumber] struct {
	parent     ParentVertex // 13 byte
	cost       W
	heapNodeId uint32 // 4 byte
}

func NewVData[W util.RoutingNumber](cost W, parent ParentVertex) VertexData[W] {
	return VertexData[W]{
		cost:       cost,
		parent:     parent,
		heapNodeId: 0,
	}
}

func (vi VertexData[W]) GetParent() ParentVertex {
	return vi.parent
}
