package datastructure

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

type VertexInfo struct {
	parent     VertexEdgePair // 13 byte
	travelTime float64        // 8 byte
	heapNodeId uint32         // 4 byte
}

func NewVertexInfo(travelTime float64, parent VertexEdgePair) VertexInfo {
	return VertexInfo{
		travelTime: travelTime,
		parent:     parent,
		heapNodeId: 0,
	}
}

func (vi *VertexInfo) GetTravelTime() float64 {
	return vi.travelTime
}

func (vi *VertexInfo) UpdateTravelTime(tt float64) {
	vi.travelTime = tt
}

func (vi *VertexInfo) UpdateParent(par VertexEdgePair) {
	vi.parent = par
}

func (vi *VertexInfo) SetHeapNodeId(id uint32) {
	vi.heapNodeId = id
}

func (vi *VertexInfo) GetHeapNodeId() uint32 {
	return vi.heapNodeId
}

func (vi *VertexInfo) SetFirstOverlayEntryExitId(id Index) {
	vi.parent.SetFirstOverlayEntryExitId(id)
}

func (vi VertexInfo) GetParent() VertexEdgePair {
	return vi.parent
}
