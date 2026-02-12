package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

/*
multilevel-ALT (A*, Landmarks , and Triangle Inequality)/multilevel-dijkstra/query phase of Customizable Route Planning only search edges & vertices that in lowest level cells that containing s or t, and all overlay vertices & shortcuts all cells in each level (other than lowest level cells that containing s or t )
thus we can preallocate the capacity of distance slices and heap as max number of edges in each cell * 2 + number of overlayVertices

buat indexing di info/distance slice & heap nya biar gak tabrakan:
edges that in lowest level cell that containing s: originalEdgeId - sCellEdgesOffsetId -> range [0, max number of edges in each cell)
edges that in lowest level cell that containing t: originalEdgeId - tCellEdgesOffsetId +  max number of edges in each cell -> range [max number of edges in each cell, max number of edges in each cell*2)
overlayVertices: original overlayVertexId +  max number of edges in each cell * 2 -> range [ max number of edges in each cell*2  ,  max number of edges in each cell*2 + number of overlayVertices)


*/

func (e *CRPRoutingEngine) offsetOverlay(v da.Index) da.Index {
	return v + e.graph.GetMaxEdgesInCell()*2
}

func (e *CRPRoutingEngine) isOverlay(u da.Index) bool {
	return u >= e.graph.GetMaxEdgesInCell()*2
}

// biar edge yang satu cell dengan s atau t punya range  [0, max number of edges in each cell) atau [max number of edges in each cell, max number of edges in each cell*2)
func (e *CRPRoutingEngine) offsetForward(u, uEntryId da.Index, uCellNumber, sCellNumber da.Pv) da.Index {
	if uCellNumber == sCellNumber {
		return uEntryId - e.graph.GetInEdgeCellOffset(u)
	}
	return uEntryId - e.graph.GetInEdgeCellOffset(u) + e.graph.GetMaxEdgesInCell()
}

func (e *CRPRoutingEngine) offsetBackward(u, uExitId da.Index, uCellNumber, sCellNumber da.Pv) da.Index {

	if uCellNumber == sCellNumber {
		return uExitId - e.graph.GetOutEdgeCellOffset(u)
	}
	return uExitId - e.graph.GetOutEdgeCellOffset(u) + e.graph.GetMaxEdgesInCell()
}

// buat balikin original edgeId
func (e *CRPRoutingEngine) adjustForward(u, uEntryId da.Index) da.Index {
	if uEntryId < e.graph.GetMaxEdgesInCell() {
		return uEntryId + e.graph.GetInEdgeCellOffset(u)
	} else {
		return uEntryId + e.graph.GetInEdgeCellOffset(u) - e.graph.GetMaxEdgesInCell()
	}
}

func (e *CRPRoutingEngine) adjustBackward(u, uExitId da.Index) da.Index {
	if uExitId < e.graph.GetMaxEdgesInCell() {
		return uExitId + e.graph.GetOutEdgeCellOffset(u)
	} else {
		return uExitId + e.graph.GetOutEdgeCellOffset(u) - e.graph.GetMaxEdgesInCell()
	}
}

// offsetedEntryId in range [0, max number of edges in each cell) atau [max number of edges in each cell, max number of edges in each cell*2)
// buat dapetin entryPoint dari edge
func (e *CRPRoutingEngine) getEntryPoint(u, offsetedEntryId da.Index, uStartEntryOffset da.Index) da.Index {
	if offsetedEntryId < e.graph.GetMaxEdgesInCell() {
		return offsetedEntryId + e.graph.GetInEdgeCellOffset(u) - uStartEntryOffset
	} else {
		return offsetedEntryId + e.graph.GetInEdgeCellOffset(u) - e.graph.GetMaxEdgesInCell() - uStartEntryOffset
	}
}

func (e *CRPRoutingEngine) getExitPoint(u, offsetedExitId da.Index, uStartExitOffset da.Index) da.Index {
	if offsetedExitId < e.graph.GetMaxEdgesInCell() {
		return offsetedExitId + e.graph.GetOutEdgeCellOffset(u) - uStartExitOffset
	} else {
		return offsetedExitId + e.graph.GetOutEdgeCellOffset(u) - e.graph.GetMaxEdgesInCell() - uStartExitOffset
	}
}

func (e *CRPRoutingEngine) adjustOverlay(v da.Index) da.Index {
	return onBit(v-e.graph.GetMaxEdgesInCell()*2, UNPACK_OVERLAY_OFFSET)
}

func isBitOn(u da.Index, i int) bool {
	if u&da.Index(uint32(1)<<i) != 0 {
		return true
	}

	return false
}

func offBit(u da.Index, i int) da.Index {
	return u & ^da.Index(uint32(1)<<i)
}

func onBit(u da.Index, i int) da.Index {
	return u | da.Index(uint32(1)<<i)
}
