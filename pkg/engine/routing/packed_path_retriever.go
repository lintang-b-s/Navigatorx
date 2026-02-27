package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (re *CRPRoutingEngine) RetrievePackedPath(forwardMid,
	backwardMid vertexEdgePair, forwardInfo []*VertexInfo[da.CRPQueryKey],
	backwardInfo []*VertexInfo[da.CRPQueryKey], sForwardId, tBackwardId da.Index, sCellNumber da.Pv) []vertexEdgePair {

	forwardPackedPath := re.RetrieveForwardPackedPath(forwardMid, forwardInfo, sForwardId, sCellNumber)

	backwardPackedPath := re.RetrieveBackwardPackedPath(backwardMid, backwardInfo, tBackwardId, sCellNumber)

	return append(forwardPackedPath, backwardPackedPath...)
}

func (re *CRPRoutingEngine) RetrieveForwardPackedPath(forwardMid vertexEdgePair, forwardInfo []*VertexInfo[da.CRPQueryKey],
	sForwardId da.Index, sCellNumber da.Pv) []vertexEdgePair {
	idPath := make([]vertexEdgePair, 0) // contains all outedges that make up the shortest path

	// let n = number of edges in shortest path from s to mid, (from forward search)
	// worst: case O(n)
	mid := forwardMid

	if !re.isOverlay(mid.getEdge()) {
		adjustedMidEdge := re.adjustForward(mid.getVertex(), mid.getEdge())
		mid.setEdge(adjustedMidEdge)

		_, midOutEdge := re.graph.GetHeadOfInedgeWithOutEdge(mid.getEdge())
		mid.setEdge(midOutEdge.GetEdgeId())
		tail := re.graph.GetTailFromOutEdge(midOutEdge.GetEdgeId())
		if tail != midOutEdge.GetHead() {
			idPath = append(idPath, mid)
		}
	}

	fMidEdge := forwardMid.getEdge()
	curInfo := forwardInfo[fMidEdge]

	lastParVertex := forwardMid.getVertex()

	for curInfo.parent.getEdge() != sForwardId {
		parent := curInfo.GetParent()
		parentEdge := parent.getEdge()
		parentCopy := parent

		if re.isOverlay(parentCopy.getEdge()) {

			// shortcut
			adjForwEdge := re.adjustOverlay(parentCopy.getEdge())
			parentCopy.setEdge(adjForwEdge)
		} else {

			adjForwEdge := re.adjustForward(parentCopy.getVertex(), parentCopy.getEdge())

			// jadiin outEdge semua
			inEdge := re.graph.GetInEdge(adjForwEdge)
			_, outEdge := re.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
			parentCopy.setEdge(outEdge.GetEdgeId())
			parentCopy.setisOutEdge(true)
		}

		idPath = append(idPath, parentCopy)
		curInfo = forwardInfo[parentEdge]

		if curInfo.parent.getEdge() != sForwardId && curInfo.parent.isFirstOverlayVertex() {
			// first entry ke overlay graph
			// lihat line 517 sampai akhir dari fungsi forwardGraphSearch,
			// dari  uParent -uEntryEdge-> u -vEntryEdge-> vOverlay
			// kita gak simpan parent dari vOverlay sebagai vEntryEdge tapi langsung uEntryEdge
			// sedangkan fungsi calculatePlateau di admissible_paths_alternatives.go buat cari alternative routes pakai plateau method,
			// kita butuh simpan parent dari vOverlay sebagai vEntryEdge, karena mungkin aja plateau (s-> .... -> u -> ..plateau... -> mid <- ...plateau... <- v <- .... <-  t) nya di di backward search
			// fInfo[vOverlay].parent.edge == uEntryEdge
			
			v := parentCopy.getVertex()
			vEntryId := curInfo.parent.getFirstOverlayEntryExitId()
			// jadiin outEdge semua
			inEdge := re.graph.GetInEdge(vEntryId)
			_, outEdge := re.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())

			firstOvArc := newVertexEdgePair(v, outEdge.GetEdgeId(), true)
			idPath = append(idPath, firstOvArc)

		}
	}

	if curInfo.parent.isFirstOverlayVertex() {
		// first arc ke overlay graph
		// lihat line 517 sampai akhir dari fungsi forwardGraphSearch,
		// dari  uParent -uEntryEdge-> u -vEntryEdge-> vOverlay
		// kita gak simpan parent dari vOverlay sebagai vEntryEdge tapi langsung uEntryEdge
		// ini buat case:  anyVertex/dummyVertex-sForwardEdge-> s -vEntryId-> vOverlay

		vEntryId := curInfo.parent.getFirstOverlayEntryExitId()
		inEdge := re.graph.GetInEdge(vEntryId)
		_, outEdge := re.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
		v := lastParVertex

		firstOvArc := newVertexEdgePair(v, outEdge.GetEdgeId(), true)
		idPath = append(idPath, firstOvArc)

	}

	idPath = util.ReverseG[vertexEdgePair](idPath)

	return idPath
}

func (re *CRPRoutingEngine) RetrieveBackwardPackedPath(backwardMid vertexEdgePair, backwardInfo []*VertexInfo[da.CRPQueryKey],
	tBackwardId da.Index, sCellNumber da.Pv) []vertexEdgePair {
	idPath := make([]vertexEdgePair, 0) // contains all outedges that make up the shortest path
	// let n = number of edges in shortest path from mid to t, (from backward search)
	// worst: case O(n)

	mid := backwardMid
	if re.isOverlay(mid.getEdge()) {
		// overlay vertex
		adjustedMidEdge := re.adjustOverlay(mid.getEdge())
		mid.setEdge(adjustedMidEdge)
		idPath = append(idPath, mid)

	} else {

		adjustedMidEdge := re.adjustBackward(mid.getVertex(), mid.getEdge())
		mid.setEdge(adjustedMidEdge)

		midOutEdge := re.graph.GetOutEdge(mid.getEdge())
		tail := re.graph.GetTailFromOutEdge(midOutEdge.GetEdgeId())
		if tail != midOutEdge.GetHead() {
			idPath = append(idPath, mid)
		}
	}

	bMidEdge := backwardMid.getEdge()
	curInfo := backwardInfo[bMidEdge]

	lastParVertex := backwardMid.getVertex()

	for curInfo.parent.getEdge() != tBackwardId {
		parent := curInfo.GetParent()
		parentEdge := parent.getEdge()
		parentCopy := parent
		lastParVertex = parentCopy.getVertex()

		if re.isOverlay(parentCopy.getEdge()) {

			// overlay vertex
			adjBackEdge := re.adjustOverlay(parentCopy.getEdge())
			parentCopy.setEdge(adjBackEdge)
		} else {

			adjBackEdge := re.adjustBackward(parentCopy.getVertex(), parentCopy.getEdge())
			parentCopy.setEdge(adjBackEdge)
		}

		idPath = append(idPath, parentCopy)
		curInfo = backwardInfo[parentEdge]

		if curInfo.parent.getEdge() != tBackwardId && curInfo.parent.isFirstOverlayVertex() {
			v := parentCopy.getVertex()
			vExitId := curInfo.parent.getFirstOverlayEntryExitId()

			// first arc ke overlay graph
			firstOvArc := newVertexEdgePair(v, vExitId, true)
			idPath = append(idPath, firstOvArc)

		}
	}

	if curInfo.parent.isFirstOverlayVertex() {
		// first arc ke overlay graph (dari backward search)
		// lihat line 677 sampai akhir dari fungsi backwardGraphSearch,
		// dari  uParent  vOverlay <-vExitId- u <-uExitId-
		// kita gak simpan parent dari vOverlay sebagai vExitId tapi langsung uExitId
		// ini buat case:  vOverlay <-vExitId- t <-tBackwardEdge- anyVertex/dummyVertex

		vExitId := curInfo.parent.getFirstOverlayEntryExitId()
		v := lastParVertex

		firstOvArc := newVertexEdgePair(v, vExitId, true)
		idPath = append(idPath, firstOvArc)

	}

	return idPath
}
