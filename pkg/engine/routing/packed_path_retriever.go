package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// RetrievePackedPath retrieves packed path.
func (crp *CRPRoutingEngine[W]) RetrievePackedPath(mid da.Index, fpq *da.QueryHeap[da.QueryKey, W],
	bpq *da.QueryHeap[da.QueryKey, W]) []da.Index {

	midvv := mid
	if crp.isOverlay(mid) {
		amvId := crp.adjustoffsetOverlay(mid)
		omvId := onBit(amvId, UnpackOverlayOffset)
		midvv = omvId
	}

	c := crp.precomputeSize(mid, fpq, bpq) + 1
	packedPath := make([]da.Index, 0, c) // list of vertices/overlay vertices di s-mid-t packed path

	crp.RetrieveForwardPackedPath(mid, fpq, &packedPath)
	util.ReverseG(packedPath)
	packedPath = append(packedPath, midvv)
	crp.RetrieveBackwardPackedPath(mid, bpq, &packedPath)
	return packedPath
}

// RetrievePackedPathUni retrieves packed path.
func (crp *CRPRoutingEngine[W]) RetrievePackedPathUni(t da.Index, pq *da.QueryHeap[da.QueryKey, W]) []da.Index {
	c := crp.precomputeSizeUni(t, pq) + 1
	packedPath := make([]da.Index, 0, c) // list of vertices/overlay vertices di s-mid-t packed path
	packedPath = append(packedPath, t)
	crp.RetrieveForwardPackedPath(t, pq, &packedPath)
	util.ReverseG(packedPath)
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
	packedPath *[]da.Index) {

	mvData := fpq.Get(midv)
	vPar := mvData.GetParent()
	for vPar != da.InvalidVertexId {
		pv := vPar
		if crp.isOverlay(pv) {
			aparv := crp.adjustoffsetOverlay(vPar)
			offpv := onBit(aparv, UnpackOverlayOffset)
			pv = offpv
		}
		*packedPath = append(*packedPath, pv)
		vData := fpq.Get(vPar)
		vPar = vData.GetParent()
	}
}

// RetrieveBackwardPackedPath untuk retrieve (packed) shortest path hasil CRP query dari mid ke t.
func (crp *CRPRoutingEngine[W]) RetrieveBackwardPackedPath(midv da.Index, bpq *da.QueryHeap[da.QueryKey, W],
	packedPath *[]da.Index) {

	mvData := bpq.Get(midv)
	vPar := mvData.GetParent()
	for vPar != da.InvalidVertexId {
		pv := vPar
		if crp.isOverlay(pv) {
			aparv := crp.adjustoffsetOverlay(vPar)
			offpv := onBit(aparv, UnpackOverlayOffset)
			pv = offpv
		}
		*packedPath = append(*packedPath, pv)
		vData := bpq.Get(vPar)
		vPar = vData.GetParent()
	}
}

func (crp *CRPRoutingEngine[W]) precomputeSizeUni(t da.Index, pq *da.QueryHeap[da.QueryKey, W]) int {
	c := 0
	mvData := pq.Get(t)
	vPar := mvData.GetParent()
	for vPar != da.InvalidVertexId {
		vData := pq.Get(vPar)
		vPar = vData.GetParent()
		c++
	}

	return c
}

func (crp *CRPRoutingEngine[W]) precomputeSize(midv da.Index, fpq, bpq *da.QueryHeap[da.QueryKey, W]) int {
	c := 0
	mvData := fpq.Get(midv)
	vPar := mvData.GetParent()
	for {
		parv := vPar
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
		parv := vPar
		if parv == da.InvalidVertexId {
			break
		}
		vData := bpq.Get(parv)
		c++
		vPar = vData.GetParent()
	}
	return c
}
