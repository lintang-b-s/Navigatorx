package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

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
	curInfo := fpq.Get(fMidEdge)

	lastParVertex := forwardMid.GetVertex()

	for curInfo.GetParent().GetEdge() != sForwardId {
		parent := curInfo.GetParent()
		parentEdge := parent.GetEdge()
		parentCopy := parent

		if crp.isOverlay(parentCopy.GetEdge()) {

			// shortcut
			adjForwEdge := crp.adjustOverlay(parentCopy.GetEdge())
			parentCopy.SetEdge(adjForwEdge)
		} else {

			adjForwEdge := crp.adjustForward(parentCopy.GetVertex(), parentCopy.GetEdge())

			// jadiin outEdge semua
			inEdge := crp.graph.GetInEdge(adjForwEdge)
			_, outEdgeId := crp.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
			parentCopy.SetEdge(outEdgeId)
		}

		svPackedPath = append(svPackedPath, parentCopy)
		curInfo = fpq.Get(parentEdge)

		if curInfo.GetParent().GetEdge() != sForwardId && curInfo.GetParent().IsFirstOverlayVertex() {
			// first entry ke overlay graph
			// lihat line 517 sampai akhir dari fungsi forwardGraphSearch,
			// dari  uParent -uEntryEdge-> u -vEntryEdge-> vOverlay
			// kita gak simpan parent dari vOverlay sebagai vEntryEdge tapi langsung uEntryEdge
			// sedangkan fungsi calculatePlateau di admissible_paths_alternatives.go buat cari alternative routes pakai plateau method,
			// kita butuh simpan parent dari vOverlay sebagai vEntryEdge, karena mungkin aja plateau (s-> .... -> u -> ..plateau... -> mid <- ...plateau... <- v <- .... <-  t) nya di di backward search
			// fInfo[vOverlay].GetParent().edge == uEntryEdge

			v := parentCopy.GetVertex()
			vEntryId := curInfo.GetParent().GetFirstOverlayEntryExitId()
			// jadiin outEdge semua
			inEdge := crp.graph.GetInEdge(vEntryId)
			_, outEdgeId := crp.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())

			firstOvArc := da.NewVertexEdgePair(v, outEdgeId, true)
			svPackedPath = append(svPackedPath, firstOvArc)

		}
	}

	if curInfo.GetParent().IsFirstOverlayVertex() {
		// first arc ke overlay graph
		// lihat line 517 sampai akhir dari fungsi forwardGraphSearch,
		// dari  uParent -uEntryEdge-> u -vEntryEdge-> vOverlay
		// kita gak simpan parent dari vOverlay sebagai vEntryEdge tapi langsung uEntryEdge
		// ini buat case:  anyVertex/dummyVertex-sForwardEdge-> s -vEntryId-> vOverlay

		vEntryId := curInfo.GetParent().GetFirstOverlayEntryExitId()
		inEdge := crp.graph.GetInEdge(vEntryId)
		_, outEdgeId := crp.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
		v := lastParVertex

		firstOvArc := da.NewVertexEdgePair(v, outEdgeId, true)
		svPackedPath = append(svPackedPath, firstOvArc)
	}

	util.ReverseG[da.VertexEdgePair](svPackedPath)

	if len(svPackedPath) == 1 {
		tail := crp.graph.GetTailFromOutEdge(svPackedPath[0].GetEdge())
		if tail != s {
			return svPackedPath[:0]
		}
	}

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
	curInfo := bpq.Get(bMidEdge)

	lastParVertex := backwardMid.GetVertex()

	for curInfo.GetParent().GetEdge() != tBackwardId {
		parent := curInfo.GetParent()
		parentEdge := parent.GetEdge()
		parentCopy := parent
		lastParVertex = parentCopy.GetVertex()

		if crp.isOverlay(parentCopy.GetEdge()) {

			// overlay vertex
			adjBackEdge := crp.adjustOverlay(parentCopy.GetEdge())
			parentCopy.SetEdge(adjBackEdge)
		} else {

			adjBackEdge := crp.adjustBackward(parentCopy.GetVertex(), parentCopy.GetEdge())
			parentCopy.SetEdge(adjBackEdge)
		}

		vtPackedPath = append(vtPackedPath, parentCopy)
		curInfo = bpq.Get(parentEdge)

		if curInfo.GetParent().GetEdge() != tBackwardId && curInfo.GetParent().IsFirstOverlayVertex() {
			v := parentCopy.GetVertex()
			vExitId := curInfo.GetParent().GetFirstOverlayEntryExitId()

			// first arc ke overlay graph
			firstOvArc := da.NewVertexEdgePair(v, vExitId, true)
			vtPackedPath = append(vtPackedPath, firstOvArc)

		}
	}

	if curInfo.GetParent().IsFirstOverlayVertex() {
		// first arc ke overlay graph (dari backward search)
		// lihat line 677 sampai akhir dari fungsi backwardGraphSearch,
		// dari  uParent  vOverlay <-vExitId- u <-uExitId-
		// kita gak simpan parent dari vOverlay sebagai vExitId tapi langsung uExitId
		// ini buat case:  vOverlay <-vExitId- t <-tBackwardEdge- anyVertex/dummyVertex

		vExitId := curInfo.GetParent().GetFirstOverlayEntryExitId()
		v := lastParVertex

		firstOvArc := da.NewVertexEdgePair(v, vExitId, true)
		vtPackedPath = append(vtPackedPath, firstOvArc)

	}

	if len(vtPackedPath) == 1 {
		head := crp.graph.GetHeadOfOutEdge(vtPackedPath[0].GetEdge())
		if head != t {
			return vtPackedPath[:0]
		}
	}

	return vtPackedPath
}
