package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (crp *CRPRoutingEngine[W]) RetrievePackedPathNoTurnCost(forwardMid,
	backwardMid da.VertexEdgePair, fpq *da.QueryHeap[da.CRPQueryKeyNoTurnCost, W],
	bpq *da.QueryHeap[da.CRPQueryKeyNoTurnCost, W], sCellNum da.Pv, s, t da.Index) []da.VertexEdgePair {

	forwardPackedPath := crp.RetrieveForwardPackedPathNoTurnCost(forwardMid, fpq, sCellNum, s)
	backwardPackedPath := crp.RetrieveBackwardPackedPathNoTurnCost(backwardMid, bpq, sCellNum, t)

	result := append(forwardPackedPath, backwardPackedPath...)

	return result
}

// RetrieveForwardPackedPath. untuk retrieve (packed) shortest path hasil CRP query dari s ke mid.
// setelah CRP query terminates, kita mendapatkan mid vertex (atau overlay vertex), dengan s-mid-t adalah shortest path
// untuk retrieve full packed path dari s ke mid kita backtrack ke ancestor (parents) dari mid sampai ke s.
// edges pada path s-mid bisa berupa shortcut edges dan base edge
// dinamakan packed path karena masih terdapat shortcut edges yang menyusun packed path
// bobot dari shortcut edge (u,v) adalah shortest path cost dari overlay vertex u ke overlay vertex v yang sudah kita precompute di fase kustomisasi CRP
// shortcut edge (u,v) disusun oleh base edges yang menyusun shortest path dari overlay vertex u ke overlay vertex v
// kita gak simpan base edges yang menyusun shortcut edge secara eksplisit, kita hanya simpan bobot nya
// sehingga untuk unpacking shortcut edges ada tahapan di CRP bernama Path Unpacking (path_unpacker_alt.go)
func (crp *CRPRoutingEngine[W]) RetrieveForwardPackedPathNoTurnCost(forwardMid da.VertexEdgePair, fpq *da.QueryHeap[da.CRPQueryKeyNoTurnCost, W],
	sCellNum da.Pv, s da.Index) []da.VertexEdgePair {
	svPackedPath := make([]da.VertexEdgePair, 0, 32) // list of vertices/overlay vertices di s-v packed path
	mid := forwardMid

	midVertex := mid.GetVertex()
	if mid.IsOverlayVertex() {
		adjMidVertex := onBit(midVertex, UNPACK_OVERLAY_OFFSET)
		mid.SetVertex(adjMidVertex)
	}
	svPackedPath = append(svPackedPath, mid)
	var vData da.VertexData[W]
	if mid.IsOverlayVertex() {
		vData = fpq.Get(crp.offsetOverlayNoTurnCost(midVertex))
	} else {
		vData = fpq.Get(midVertex)
	}

	for vData.GetParent().GetVertex() != da.INVALID_VERTEX_ID {
		parent := vData.GetParent()
		parentVertex := parent.GetVertex()

		if parent.IsOverlayVertex() {
			offParentVertex := onBit(parentVertex, UNPACK_OVERLAY_OFFSET)
			parent.SetVertex(offParentVertex)
			vData = fpq.Get(crp.offsetOverlayNoTurnCost(parentVertex))
		} else {
			vData = fpq.Get(parentVertex)
		}

		svPackedPath = append(svPackedPath, parent)

	}

	util.ReverseG(svPackedPath)

	return svPackedPath
}

// RetrieveBackwardPackedPath. untuk retrieve (packed) shortest path hasil CRP query dari mid ke t.
func (crp *CRPRoutingEngine[W]) RetrieveBackwardPackedPathNoTurnCost(backwardMid da.VertexEdgePair, bpq *da.QueryHeap[da.CRPQueryKeyNoTurnCost, W],
	sCellNum da.Pv, t da.Index) []da.VertexEdgePair {
	vtPackedPath := make([]da.VertexEdgePair, 0, 32)
	mid := backwardMid

	var vData da.VertexData[W]
	if mid.IsOverlayVertex() {
		vData = bpq.Get(crp.offsetOverlayNoTurnCost(mid.GetVertex()))
	} else {
		vData = bpq.Get(mid.GetVertex())
	}

	for vData.GetParent().GetVertex() != da.INVALID_VERTEX_ID {
		parent := vData.GetParent()
		parentVertex := parent.GetVertex()

		if parent.IsOverlayVertex() {
			offParentVertex := onBit(parentVertex, UNPACK_OVERLAY_OFFSET)
			parent.SetVertex(offParentVertex)
			vData = bpq.Get(crp.offsetOverlayNoTurnCost(parentVertex))
		} else {
			vData = bpq.Get(parentVertex)
		}

		vtPackedPath = append(vtPackedPath, parent)

	}

	return vtPackedPath
}
