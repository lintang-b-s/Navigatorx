package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (crp *CRPRoutingEngine[W]) RetrievePackedPath(mid da.ParentVertex, fpq *da.QueryHeap[da.QueryKey, W],
	bpq *da.QueryHeap[da.QueryKey, W], sCellNum da.Pv, s, t da.Index) []da.ParentVertex {

	forwardPackedPath := crp.RetrieveForwardPackedPath(mid, fpq, sCellNum, s)
	backwardPackedPath := crp.RetrieveBackwardPackedPath(mid, bpq, sCellNum, t)

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
func (crp *CRPRoutingEngine[W]) RetrieveForwardPackedPath(mid da.ParentVertex, fpq *da.QueryHeap[da.QueryKey, W],
	sCellNum da.Pv, s da.Index) []da.ParentVertex {
	svPackedPath := make([]da.ParentVertex, 0, 32) // list of vertices/overlay vertices di s-v packed path

	midvId := mid.GetVertex()
	if mid.IsOverlayVertex() {
		amvId := crp.adjustoffsetOverlay(midvId)
		omvId := onBit(amvId, UNPACK_OVERLAY_OFFSET)
		mid.SetVertex(omvId)
	}
	svPackedPath = append(svPackedPath, mid)
	var vData da.VertexData[W]
	if mid.IsOverlayVertex() {
		vData = fpq.Get(midvId)
	} else {
		vData = fpq.Get(midvId)
	}

	vPar := vData.GetParent()
	for vPar.GetVertex() != da.INVALID_VERTEX_ID {
		parvId := vPar.GetVertex()
		if vPar.IsOverlayVertex() {
			aparvId := crp.adjustoffsetOverlay(parvId)
			offpvId := onBit(aparvId, UNPACK_OVERLAY_OFFSET)
			vPar.SetVertex(offpvId)
			vData = fpq.Get(parvId)
		} else {
			vData = fpq.Get(parvId)
		}

		svPackedPath = append(svPackedPath, vPar)
		vPar = vData.GetParent()
	}

	util.ReverseG(svPackedPath)

	return svPackedPath
}

// RetrieveBackwardPackedPath. untuk retrieve (packed) shortest path hasil CRP query dari mid ke t.
func (crp *CRPRoutingEngine[W]) RetrieveBackwardPackedPath(mid da.ParentVertex, bpq *da.QueryHeap[da.QueryKey, W],
	sCellNum da.Pv, t da.Index) []da.ParentVertex {
	vtPackedPath := make([]da.ParentVertex, 0, 32)

	midvId := mid.GetVertex()
	var vData da.VertexData[W]
	if mid.IsOverlayVertex() {
		vData = bpq.Get(midvId)
	} else {
		vData = bpq.Get(midvId)
	}

	vPar := vData.GetParent()
	for vPar.GetVertex() != da.INVALID_VERTEX_ID {
		parvId := vPar.GetVertex()
		if vPar.IsOverlayVertex() {
			aparvId := crp.adjustoffsetOverlay(parvId)
			offpvId := onBit(aparvId, UNPACK_OVERLAY_OFFSET)
			vPar.SetVertex(offpvId)
			vData = bpq.Get(parvId)
		} else {
			vData = bpq.Get(parvId)
		}

		vtPackedPath = append(vtPackedPath, vPar)
		vPar = vData.GetParent()
	}

	return vtPackedPath
}
