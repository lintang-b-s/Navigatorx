package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// RetrievePackedPath retrieves packed path.
func (crp *CRPRoutingEngine[W]) RetrievePackedPath(mid da.ParentVertex, fpq *da.QueryHeap[da.QueryKey, W],
	bpq *da.QueryHeap[da.QueryKey, W], sCellNum da.Pv, s, t da.Index) []da.Index {

	midv := mid.GetVertex()
	midvv := midv
	if mid.IsOverlayVertex() {
		amvId := crp.adjustoffsetOverlay(midv)
		omvId := onBit(amvId, UnpackOverlayOffset)
		midvv = omvId
	}

	c := crp.precomputeSize(midv, fpq, bpq) + 1
	packedPath := make([]da.Index, 0, c) // list of vertices/overlay vertices di s-mid-t packed path

	crp.RetrieveForwardPackedPath(midv, fpq, sCellNum, s, &packedPath)
	util.ReverseG(packedPath)
	packedPath = append(packedPath, midvv)
	crp.RetrieveBackwardPackedPath(midv, bpq, sCellNum, t, &packedPath)
	return packedPath
}

// RetrieveForwardPackedPath untuk retrieve (packed) shortest path hasil CRP query dari vertex s ke vertex mid.
// setelah CRP query terminates, kita mendapatkan mid vertex (atau overlay vertex), dengan s-mid-t adalah shortest path
// untuk retrieve full packed path dari s ke mid kita backtrack ke ancestor (parents) dari mid sampai ke s.
// edges pada path s-mid bisa berupa shortcut edges dan base edge
// dinamakan packed path karena masih terdapat shortcut edges yang menyusun packed path
// bobot dari shortcut edge (u,v) adalah shortest path cost dari overlay vertex u ke overlay vertex v yang sudah kita precompute di fase kustomisasi CRP
// shortcut edge (u,v) disusun oleh base edges yang menyusun shortest path dari overlay vertex u ke overlay vertex v
// kita gak simpan base edges yang menyusun shortcut edge secara eksplisit, kita hanya simpan bobot nya
// sehingga untuk unpacking shortcut edges ada tahapan di CRP bernama Path Unpacking (path_unpacker_alt.go)
func (crp *CRPRoutingEngine[W]) RetrieveForwardPackedPath(midv da.Index, fpq *da.QueryHeap[da.QueryKey, W],
	_ da.Pv, _ da.Index, packedPath *[]da.Index) {

	mvData := fpq.Get(midv)
	vPar := mvData.GetParent()
	for {
		parv := vPar.GetVertex()
		if parv == da.InvalidVertexId {
			break
		}
		pv := parv
		if vPar.IsOverlayVertex() {
			aparv := crp.adjustoffsetOverlay(parv)
			offpv := onBit(aparv, UnpackOverlayOffset)
			pv = offpv
		}
		*packedPath = append(*packedPath, pv)
		vData := fpq.Get(parv)
		vPar = vData.GetParent()
	}
}

// RetrieveBackwardPackedPath untuk retrieve (packed) shortest path hasil CRP query dari mid ke t.
func (crp *CRPRoutingEngine[W]) RetrieveBackwardPackedPath(midv da.Index, bpq *da.QueryHeap[da.QueryKey, W],
	_ da.Pv, _ da.Index, packedPath *[]da.Index) {

	mvData := bpq.Get(midv)
	vPar := mvData.GetParent()
	for {
		parv := vPar.GetVertex()
		if parv == da.InvalidVertexId {
			break
		}
		pv := parv
		if vPar.IsOverlayVertex() {
			aparv := crp.adjustoffsetOverlay(parv)
			offpv := onBit(aparv, UnpackOverlayOffset)
			pv = offpv
		}
		*packedPath = append(*packedPath, pv)
		vData := bpq.Get(parv)
		vPar = vData.GetParent()
	}
}

func (crp *CRPRoutingEngine[W]) precomputeSize(midv da.Index, fpq, bpq *da.QueryHeap[da.QueryKey, W]) int {
	c := 0
	mvData := fpq.Get(midv)
	vPar := mvData.GetParent()
	for {
		parv := vPar.GetVertex()
		if parv == da.InvalidVertexId {
			break
		}
		vData := fpq.Get(parv)
		vPar = vData.GetParent()
		c++
	}

	mvData = bpq.Get(midv)
	vPar = mvData.GetParent()
	for {
		parv := vPar.GetVertex()
		if parv == da.InvalidVertexId {
			break
		}
		vData := bpq.Get(parv)
		c++
		vPar = vData.GetParent()
	}
	return c
}
