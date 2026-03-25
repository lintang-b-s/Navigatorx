package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (re *CRPRoutingEngine) RetrievePackedPath(forwardMid,
	backwardMid da.VertexEdgePair, fpq *da.QueryHeap[da.CRPQueryKey],
	bpq *da.QueryHeap[da.CRPQueryKey], sForwardId, tBackwardId da.Index, sCellNumber da.Pv) []da.VertexEdgePair {

	svPackedPath := make([]da.VertexEdgePair, 0, 256)
	vtPackedPath := make([]da.VertexEdgePair, 0, 256)

	forwardPackedPath := re.RetrieveForwardPackedPath(svPackedPath, forwardMid, fpq, sForwardId, sCellNumber)

	backwardPackedPath := re.RetrieveBackwardPackedPath(vtPackedPath, backwardMid, bpq, tBackwardId, sCellNumber)

	return append(forwardPackedPath, backwardPackedPath...)
}

// RetrieveForwardPackedPath. untuk retrieve (packed) shortest path hasil CRP query dari s ke mid.
// setelah CRP query terminates, kita mendapatkan mid vertex (atau overlay vertex), dengan s-mid-t adalah shortest path
// untuk retrieve full packed path dari s ke mid kita backtrack ke ancestor (parents) dari mid sampai ke s.
// edges pada path s-mid bisa berupa shortcut edges dan base edge
// dinakan packed path karena masih terdapat shortcut edges yang menyusun packed path
// bobot dari shortcut edge (u,v) adalah shortest path cost dari overlay vertex u ke overlay vertex v yang sudah kita precompute di fase kustomisasi CRP
// shortcut edge (u,v) disusun oleh base edges yang menyusun shortest path dari overlay vertex u ke overlay vertex v
// kita gak simpan base edges yang menyusun shortcut edge secara eksplisit, kita hanya simpan bobot nya
// sehingga untuk unpacking shortcut edges ada tahapan di CRP bernama Path Unpacking (path_unpacker_alt.go)
func (re *CRPRoutingEngine) RetrieveForwardPackedPath(svPackedPath []da.VertexEdgePair, forwardMid da.VertexEdgePair, fpq *da.QueryHeap[da.CRPQueryKey],
	sForwardId da.Index, sCellNumber da.Pv) []da.VertexEdgePair {

	// let n = number of edges in shortest path from s to mid, (from forward search)
	// O(n)
	mid := forwardMid

	if !re.isOverlay(mid.GetEdge()) {
		adjustedMidEdge := re.adjustForward(mid.GetVertex(), mid.GetEdge())
		mid.SetEdge(adjustedMidEdge)

		_, midOutEdge := re.graph.GetHeadOfInedgeWithOutEdge(mid.GetEdge())
		mid.SetEdge(midOutEdge.GetEdgeId())
		tail := re.graph.GetTailFromOutEdge(midOutEdge.GetEdgeId())
		if tail != midOutEdge.GetHead() {
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

		if re.isOverlay(parentCopy.GetEdge()) {

			// shortcut
			adjForwEdge := re.adjustOverlay(parentCopy.GetEdge())
			parentCopy.SetEdge(adjForwEdge)
		} else {

			adjForwEdge := re.adjustForward(parentCopy.GetVertex(), parentCopy.GetEdge())

			// jadiin outEdge semua
			inEdge := re.graph.GetInEdge(adjForwEdge)
			_, outEdge := re.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
			parentCopy.SetEdge(outEdge.GetEdgeId())
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
			inEdge := re.graph.GetInEdge(vEntryId)
			_, outEdge := re.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())

			firstOvArc := da.NewVertexEdgePair(v, outEdge.GetEdgeId(), true)
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
		inEdge := re.graph.GetInEdge(vEntryId)
		_, outEdge := re.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
		v := lastParVertex

		firstOvArc := da.NewVertexEdgePair(v, outEdge.GetEdgeId(), true)
		svPackedPath = append(svPackedPath, firstOvArc)
	}

	util.ReverseG[da.VertexEdgePair](svPackedPath)

	return svPackedPath
}

// RetrieveBackwardPackedPath. untuk retrieve (packed) shortest path hasil CRP query dari mid ke t.
func (re *CRPRoutingEngine) RetrieveBackwardPackedPath(vtPackedPath []da.VertexEdgePair, backwardMid da.VertexEdgePair, bpq *da.QueryHeap[da.CRPQueryKey],
	tBackwardId da.Index, sCellNumber da.Pv) []da.VertexEdgePair {
	// let n = number of edges in shortest path from mid to t, (from backward search)
	// worst: case O(n)

	mid := backwardMid
	if re.isOverlay(mid.GetEdge()) {
		// overlay vertex
		adjustedMidEdge := re.adjustOverlay(mid.GetEdge())
		mid.SetEdge(adjustedMidEdge)
		vtPackedPath = append(vtPackedPath, mid)

	} else {

		adjustedMidEdge := re.adjustBackward(mid.GetVertex(), mid.GetEdge())
		mid.SetEdge(adjustedMidEdge)

		midOutEdge := re.graph.GetOutEdge(mid.GetEdge())
		tail := re.graph.GetTailFromOutEdge(midOutEdge.GetEdgeId())
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

		if re.isOverlay(parentCopy.GetEdge()) {

			// overlay vertex
			adjBackEdge := re.adjustOverlay(parentCopy.GetEdge())
			parentCopy.SetEdge(adjBackEdge)
		} else {

			adjBackEdge := re.adjustBackward(parentCopy.GetVertex(), parentCopy.GetEdge())
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

	return vtPackedPath
}

func (re *CRPRoutingEngine) RetrieveForwardUnpackedPath(forwardMid da.VertexEdgePair, fpq *da.QueryHeap[da.CRPQueryKey],
	sForwardId da.Index, sCellNumber da.Pv) (float64, float64, []da.Coordinate, []da.OutEdge) {
	finalEdgePath := make([]da.OutEdge, 0)
	finalPath := make([]da.Coordinate, 0)

	mid := forwardMid

	totalDistance := 0.0
	totalTravelTime := 0.0
	_, midOutEdge := re.graph.GetHeadOfInedgeWithOutEdge(mid.GetEdge())
	mid.SetEdge(midOutEdge.GetEdgeId())
	tail := re.graph.GetTailFromOutEdge(midOutEdge.GetEdgeId())

	if tail != midOutEdge.GetHead() {
		geom := re.graph.GetEdgeGeometry(midOutEdge.GetEdgeId())
		revGeom := make([]da.Coordinate, len(geom))
		copy(revGeom, geom)
		util.ReverseG(revGeom)
		finalPath = append(finalPath, revGeom...)
		finalEdgePath = append(finalEdgePath, *midOutEdge)
		totalDistance += midOutEdge.GetLength()
		totalTravelTime += re.costFunction.GetWeight(midOutEdge)
	}

	curInfo := fpq.Get(forwardMid.GetEdge())

	for curInfo.GetParent().GetEdge() != sForwardId {
		parent := curInfo.GetParent()
		parentEdge := parent.GetEdge()
		parentCopy := parent

		// jadiin outEdge semua
		inEdge := re.graph.GetInEdge(parentCopy.GetEdge())
		_, outEdge := re.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
		parentCopy.SetEdge(outEdge.GetEdgeId())

		finalEdgePath = append(finalEdgePath, *outEdge)

		geom := re.graph.GetEdgeGeometry(outEdge.GetEdgeId())
		revGeom := make([]da.Coordinate, len(geom))
		copy(revGeom, geom)
		util.ReverseG(revGeom)
		finalPath = append(finalPath, revGeom...)
		totalDistance += outEdge.GetLength()
		totalTravelTime += re.costFunction.GetWeight(outEdge)
		curInfo = fpq.Get(parentEdge)
	}

	util.ReverseG(finalPath)

	return totalTravelTime, totalDistance, finalPath, finalEdgePath
}

func (re *CRPRoutingEngine) RetrieveBackwardUnpackedPath(backwardMid da.VertexEdgePair, bpq *da.QueryHeap[da.CRPQueryKey],
	tBackwardId da.Index, sCellNumber da.Pv) (float64, float64, []da.Coordinate, []da.OutEdge) {

	totalTravelTime := 0.0
	finalEdgePath := make([]da.OutEdge, 0)
	finalPath := make([]da.Coordinate, 0)

	mid := backwardMid

	totalDistance := 0.0

	midOutEdge := re.graph.GetOutEdge(mid.GetEdge())
	tail := re.graph.GetTailFromOutEdge(midOutEdge.GetEdgeId())
	if tail != midOutEdge.GetHead() {
		geom := re.graph.GetEdgeGeometry(midOutEdge.GetEdgeId())
		finalPath = append(finalPath, geom...)
		finalEdgePath = append(finalEdgePath, *midOutEdge)
		totalDistance += midOutEdge.GetLength()
		totalTravelTime += re.metrics.GetWeight(midOutEdge)
	}

	curInfo := bpq.Get(backwardMid.GetEdge())

	for curInfo.GetParent().GetEdge() != tBackwardId {
		parent := curInfo.GetParent()
		parentEdge := parent.GetEdge()
		parentCopy := parent

		outEdge := re.graph.GetOutEdge(parentCopy.GetEdge())
		finalEdgePath = append(finalEdgePath, *outEdge)

		geom := re.graph.GetEdgeGeometry(outEdge.GetEdgeId())
		finalPath = append(finalPath, geom...)
		totalDistance += outEdge.GetLength()
		totalTravelTime += re.metrics.GetWeight(outEdge)
		curInfo = bpq.Get(parentEdge)
	}

	return totalTravelTime, totalDistance, finalPath, finalEdgePath
}
