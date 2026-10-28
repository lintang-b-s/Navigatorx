package datastructure

import "github.com/lintang-b-s/Navigatorx/pkg/util"

const (
	overlayFlag uint8 = 1 << iota
	cutEdgeFlag
)

// ParentVertex represents parent vertex.
type ParentVertex struct {
	vertex Index // 4 byte
	flag   uint8
}

// GetVertex returns vertex.
func (ve ParentVertex) GetVertex() Index {
	return ve.vertex
}

// IsOverlayVertex reports whether overlay vertex.
func (ve ParentVertex) IsOverlayVertex() bool {
	return ve.flag&overlayFlag != 0
}

// NewParentVertex creates and returns parent vertex.
func NewParentVertex(vertex Index, overlay bool) ParentVertex {
	pv := ParentVertex{
		vertex: vertex,
	}
	if overlay {
		pv.flag |= overlayFlag
	}
	return pv
}

// VertexData represents vertex data.
type VertexData[W util.Numeric] struct {
	parent     ParentVertex
	cost       W
	heapNodeId Index // 4 byte
}

// NewVData creates and returns vertex data.
func NewVData[W util.Numeric](cost W, parent ParentVertex) VertexData[W] {
	return VertexData[W]{
		cost:       cost,
		parent:     parent,
		heapNodeId: 0,
	}
}

// GetParent returns parent.
func (vi VertexData[W]) GetParent() ParentVertex {
	return vi.parent
}
