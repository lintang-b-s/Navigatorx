package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

/*
multilevel-ALT (A*, Landmarks , and Triangle Inequality)/multilevel-dijkstra/query phase of Customizable Route Planning only search at most edges & vertices that in lowest level cells that containing s or t, and all overlay vertices & shortcuts all cells in each level (other than lowest level cells that containing s or t )
thus we can preallocate the capacity of distance slices and heap as max number of edges in each cell * 2 + number of overlayVertices

buat indexing di info/distance slice biar gak tabrakan:
edges that in lowest level cell that containing s: originalEdgeId - sCellEdgesOffsetId -> range [0, max number of edges in each cell)
edges that in lowest level cell that containing t: originalEdgeId - tCellEdgesOffsetId +  max number of edges in each cell -> range [max number of edges in each cell, max number of edges in each cell*2)
overlayVertices: original overlayVertexId +  max number of edges in each cell * 2 -> range [ max number of edges in each cell*2,  max number of edges in each cell*2 + number of overlayVertices)


*/

func (crp *CRPRoutingEngine[W]) offsetOverlay(v da.Index) da.Index {
	return v + crp.graph.GetMaxEdgesInCell()*2
}

func (crp *CRPRoutingEngine[W]) isOverlay(u da.Index) bool {
	return u >= crp.graph.GetMaxEdgesInCell()*2
}

// biar edge yang satu cell dengan s atau t punya range  [0, max number of edges in each cell) atau [max number of edges in each cell, max number of edges in each cell*2)
func (crp *CRPRoutingEngine[W]) offsetForward(u, uEntryId da.Index, uCellNumber, sCellNumber da.Pv) da.Index {

	if uCellNumber == sCellNumber {
		return uEntryId - crp.graph.GetInEdgeCellOffset(u)
	}
	return uEntryId - crp.graph.GetInEdgeCellOffset(u) + crp.graph.GetMaxEdgesInCell()
}

func (crp *CRPRoutingEngine[W]) offsetBackward(u, uExitId da.Index, uCellNumber, sCellNumber da.Pv) da.Index {

	if uCellNumber == sCellNumber {
		return uExitId - crp.graph.GetOutEdgeCellOffset(u)
	}
	return uExitId - crp.graph.GetOutEdgeCellOffset(u) + crp.graph.GetMaxEdgesInCell()
}

// buat balikin original edgeId
func (crp *CRPRoutingEngine[W]) adjustForward(u, uEntryId da.Index) da.Index {
	if uEntryId < crp.graph.GetMaxEdgesInCell() {
		return uEntryId + crp.graph.GetInEdgeCellOffset(u)
	} else {
		return uEntryId + crp.graph.GetInEdgeCellOffset(u) - crp.graph.GetMaxEdgesInCell()
	}
}

func (crp *CRPRoutingEngine[W]) adjustBackward(u, uExitId da.Index) da.Index {
	if uExitId < crp.graph.GetMaxEdgesInCell() {
		return uExitId + crp.graph.GetOutEdgeCellOffset(u)
	} else {
		return uExitId + crp.graph.GetOutEdgeCellOffset(u) - crp.graph.GetMaxEdgesInCell()
	}
}

// offsetedEntryId in range [0, max number of edges in each cell) atau [max number of edges in each cell, max number of edges in each cell*2)
// buat dapetin entryPoint dari edge
func (crp *CRPRoutingEngine[W]) getEntryPoint(u, offsetedEntryId da.Index, uStartEntryOffset da.Index) da.Index {
	if offsetedEntryId < crp.graph.GetMaxEdgesInCell() {
		return offsetedEntryId + crp.graph.GetInEdgeCellOffset(u) - uStartEntryOffset
	} else {
		return offsetedEntryId + crp.graph.GetInEdgeCellOffset(u) - crp.graph.GetMaxEdgesInCell() - uStartEntryOffset
	}
}

func (crp *CRPRoutingEngine[W]) getExitPoint(u, offsetedExitId da.Index, uStartExitOffset da.Index) da.Index {
	if offsetedExitId < crp.graph.GetMaxEdgesInCell() {
		return offsetedExitId + crp.graph.GetOutEdgeCellOffset(u) - uStartExitOffset
	} else {
		return offsetedExitId + crp.graph.GetOutEdgeCellOffset(u) - crp.graph.GetMaxEdgesInCell() - uStartExitOffset
	}
}

func (crp *CRPRoutingEngine[W]) adjustOverlay(v da.Index) da.Index {
	return onBit(v-crp.graph.GetMaxEdgesInCell()*2, UNPACK_OVERLAY_OFFSET)
}

func isBitOn(u da.Index, i int) bool {
	return u&da.Index(uint32(1)<<i) != 0
}

func offBit(u da.Index, i int) da.Index {
	return u & ^da.Index(uint32(1)<<i)
}

func onBit(u da.Index, i int) da.Index {
	return u | da.Index(uint32(1)<<i)
}

// offset without turn costs

func (crp *CRPRoutingEngine[W]) offsetOverlayNoTurnCost(v da.Index) da.Index {
	return v + da.Index(crp.graph.NumberOfVertices())
}

func (crp *CRPRoutingEngine[W]) adjustOffsetOverlayNoTurnCost(v da.Index) da.Index {
	return v - da.Index(crp.graph.NumberOfVertices())
}
