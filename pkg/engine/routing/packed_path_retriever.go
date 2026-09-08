package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// todo: ini bisa di refactor jadi lebih clean lagi... tapi harus diubah dulu predecessor dari setiap explored vertex di multilevel_astar_landmarks.go/multilevel_dijkstra.go
// kerjain setelah submit revisi
// done

func (crp *CRPRoutingEngine[W]) RetrievePackedPath(forwardMid,
	backwardMid da.VertexEdgePair, fpq *da.QueryHeap[da.CRPQueryKey, W],
	bpq *da.QueryHeap[da.CRPQueryKey, W], sForwardId, tBackwardId da.Index, sCellNumber da.Pv, s, t da.Index) []da.VertexEdgePair {

	forwardPackedPath := crp.RetrieveForwardPackedPath(forwardMid, fpq, sForwardId, sCellNumber, s)
	backwardPackedPath := crp.RetrieveBackwardPackedPath(backwardMid, bpq, tBackwardId, sCellNumber, t)

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
func (crp *CRPRoutingEngine[W]) RetrieveForwardPackedPath(forwardMid da.VertexEdgePair, fpq *da.QueryHeap[da.CRPQueryKey, W],
	sForwardId da.Index, sCellNumber da.Pv, s da.Index) []da.VertexEdgePair {
	svPackedPath := make([]da.VertexEdgePair, 0, 32)
	// let n = number of edges in shortest path from s to mid, (from forward search)
	// O(n)
	mid := forwardMid

	if !crp.isOverlay(mid.GetEdge()) {
		adjustedMidEdge := crp.adjustForward(mid.GetVertex(), mid.GetEdge())
		mid.SetEdge(adjustedMidEdge)

		midOutHead, midOutEdgeId := crp.graph.GetHeadOfInedgeWithOutEdge(mid.GetEdge())
		mid.SetEdge(midOutEdgeId)

		tail := crp.graph.GetTailFromOutEdge(midOutEdgeId)
		if tail != midOutHead {
			svPackedPath = append(svPackedPath, mid)
		}
	}

	fMidEdge := forwardMid.GetEdge()
	vData := fpq.Get(fMidEdge)

	for vData.GetParent().GetEdge() != sForwardId {
		parent := vData.GetParent()
		parentEdge := parent.GetEdge()
		parentCopy := parent

		if crp.isOverlay(parentCopy.GetEdge()) {

			// overlay vertex
			ov := crp.adjustOverlay(parentCopy.GetEdge())
			parentCopy.SetEdge(ov)
		} else {
			adjForwEdge := crp.adjustForward(parentCopy.GetVertex(), parentCopy.GetEdge())

			// jadiin outEdge semua
			inEdge := crp.graph.GetInEdge(adjForwEdge)
			_, outEId := crp.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
			parentCopy.SetEdge(outEId)
		}

		svPackedPath = append(svPackedPath, parentCopy)
		vData = fpq.Get(parentEdge)
	}

	util.ReverseG[da.VertexEdgePair](svPackedPath)

	return svPackedPath
}

// RetrieveBackwardPackedPath. untuk retrieve (packed) shortest path hasil CRP query dari mid ke t.
func (crp *CRPRoutingEngine[W]) RetrieveBackwardPackedPath(backwardMid da.VertexEdgePair, bpq *da.QueryHeap[da.CRPQueryKey, W],
	tBackwardId da.Index, sCellNumber da.Pv, t da.Index) []da.VertexEdgePair {
	vtPackedPath := make([]da.VertexEdgePair, 0, 32)

	// let n = number of edges in shortest path from mid to t, (from backward search)
	// worst: case O(n)

	mid := backwardMid
	if crp.isOverlay(mid.GetEdge()) {
		// overlay vertex
		adjustedMidEdge := crp.adjustOverlay(mid.GetEdge())
		mid.SetEdge(adjustedMidEdge)
		vtPackedPath = append(vtPackedPath, mid)

	} else {

		adjustedMidEdge := crp.adjustBackward(mid.GetVertex(), mid.GetEdge())
		mid.SetEdge(adjustedMidEdge)

		midOutEdge := crp.graph.GetOutEdge(mid.GetEdge())
		tail := crp.graph.GetTailFromOutEdge(midOutEdge.GetEdgeId())
		if tail != midOutEdge.GetHead() {
			vtPackedPath = append(vtPackedPath, mid)
		}
	}

	bMidEdge := backwardMid.GetEdge()
	vData := bpq.Get(bMidEdge)

	for vData.GetParent().GetEdge() != tBackwardId {
		parent := vData.GetParent()
		parentEdge := parent.GetEdge()
		parentCopy := parent

		if crp.isOverlay(parentCopy.GetEdge()) {

			// overlay vertex
			ov := crp.adjustOverlay(parentCopy.GetEdge())
			parentCopy.SetEdge(ov)
		} else {

			adjEdge := crp.adjustBackward(parentCopy.GetVertex(), parentCopy.GetEdge())
			parentCopy.SetEdge(adjEdge)
		}

		vtPackedPath = append(vtPackedPath, parentCopy)
		vData = bpq.Get(parentEdge)

	}

	return vtPackedPath
}
