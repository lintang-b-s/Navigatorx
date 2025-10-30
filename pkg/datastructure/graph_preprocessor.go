package datastructure

import "math"

func (g *Graph) SortByCellNumber() {
	cellVertices := make([][]struct {
		vertex        *Vertex
		originalIndex Index
	}, g.GetNumberOfCellsNumbers()) // slice of slice of vertices in each cell

	minLat, minLon := math.MaxFloat64, math.MaxFloat64
	maxLat, maxLon := math.Inf(-1), math.Inf(-1)

	numOutEdgesInCell := make([]Index, g.GetNumberOfCellsNumbers()) // number of outEdges in each cell
	numInEdgesInCell := make([]Index, g.GetNumberOfCellsNumbers())

	oEdges := make([][]*OutEdge, g.NumberOfVertices()) // copy of original outEdges of each vertex
	iEdges := make([][]*InEdge, g.NumberOfVertices())

	g.maxEdgesInCell = Index(0) // maximum number of edges in any cell
	for i := Index(0); i < Index(g.NumberOfVertices()); i++ {
		cell := g.vertices[i].pvPtr // cellNumber
		v := g.vertices[i]

		cellVertices[cell] = append(cellVertices[cell], struct {
			vertex        *Vertex
			originalIndex Index
		}{vertex: v, originalIndex: i})

		oEdges[i] = make([]*OutEdge, g.GetOutDegree(i))
		iEdges[i] = make([]*InEdge, g.GetInDegree(i))

		k := Index(0)
		e := g.vertices[i].firstOut
		for e < g.vertices[i+1].firstOut {
			oEdge := g.outEdges[e]
			oEdges[i][k] = NewOutEdge(
				oEdge.edgeId,
				oEdge.head,
				oEdge.weight,
				oEdge.dist,
				oEdge.entryPoint,
			)
			e++
			k++
		}

		k = Index(0)
		e = g.vertices[i].firstIn
		for e < g.vertices[i+1].firstIn {
			inEdge := g.inEdges[e]
			iEdges[i][k] = NewInEdge(
				inEdge.edgeId,
				inEdge.tail,
				inEdge.weight,
				inEdge.dist,
				inEdge.exitPoint,
			)
			e++
			k++
		}

		numOutEdgesInCell[cell] += g.GetOutDegree(i)
		numInEdgesInCell[cell] += g.GetInDegree(i)

		if g.maxEdgesInCell < numOutEdgesInCell[cell] {
			g.maxEdgesInCell = numOutEdgesInCell[cell]
		}

		if g.maxEdgesInCell < numInEdgesInCell[cell] {
			g.maxEdgesInCell = numInEdgesInCell[cell]
		}

		minLat = math.Min(minLat, g.vertices[i].lat)
		minLon = math.Min(minLon, g.vertices[i].lon)
		maxLat = math.Max(maxLat, g.vertices[i].lat)
		maxLon = math.Max(maxLon, g.vertices[i].lon)
	}
	g.SetBoundingBox(NewBoundingBox(minLat, minLon, maxLat, maxLon))

	newIds := make([]Index, g.NumberOfVertices()) // new vertex id after sorting by cell number
	newVid := Index(0)                            // new vertex id after sorting by cell number
	for i := 0; i < len(cellVertices); i++ {
		for v := 0; v < len(cellVertices[i]); v++ {
			newIds[cellVertices[i][v].originalIndex] = newVid
			newVid++
		}
	}

	outOffset := Index(0)                                   // new offset for outEdges for each vertex for each cell
	g.outEdgeCellOffset = make([]Index, len(g.cellNumbers)) // offset of first outEdge for each cell
	inOffset := Index(0)                                    // new offset for inEdges for each vertex for each cell
	g.inEdgeCellOffset = make([]Index, len(g.cellNumbers))  // offset of first inEdge for each cell

	gsEdgeExtraInfos := make([]EdgeExtraInfo, len(g.graphStorage.mapEdgeInfo))
	copy(gsEdgeExtraInfos, g.graphStorage.mapEdgeInfo)

	roundabout := make([]Index, len(g.graphStorage.roundaboutFlag))
	copy(roundabout, g.graphStorage.roundaboutFlag)

	nodeTrafficLight := make([]Index, len(g.graphStorage.nodeTrafficLight))
	copy(nodeTrafficLight, g.graphStorage.nodeTrafficLight)
	vId := Index(0)

	lastVertex := g.vertices[len(g.vertices)-1]
	newVertices := make([]*Vertex, len(g.vertices))
	for i := Index(0); i < Index(len(g.cellNumbers)); i++ {
		g.outEdgeCellOffset[i] = outOffset
		g.inEdgeCellOffset[i] = inOffset

		for v := Index(0); v < Index(len(cellVertices[i])); v++ {
			// update vertex to use new vId
			// in the end of the outer loop, graph vertices are sorted by cell number

			newVertices[vId] = cellVertices[i][v].vertex

			vOldId := cellVertices[i][v].originalIndex
			newVertices[vId].SetFirstOut(outOffset)
			newVertices[vId].SetFirstIn(inOffset)
			newVertices[vId].SetId(vId)

			index := int(math.Floor(float64(vOldId) / 32))

			isTraficLight := (nodeTrafficLight[index] & (1 << (vOldId % 32))) != 0
			if isTraficLight {
				g.graphStorage.SetTrafficLight(vId)
			}

			// update outedges & inedges
			for k := Index(0); k < Index(len(oEdges[vOldId])); k++ {

				oldOutEdge := oEdges[vOldId][k]
				g.outEdges[outOffset] = NewOutEdge(
					oldOutEdge.edgeId, oldOutEdge.head, oldOutEdge.weight,
					oldOutEdge.dist, oldOutEdge.entryPoint,
				)
				g.graphStorage.mapEdgeInfo[outOffset] = gsEdgeExtraInfos[oldOutEdge.edgeId] // update edge extra info storage

				indexRoundabout := int(math.Floor(float64(oldOutEdge.edgeId) / 32))
				roundabout := (roundabout[indexRoundabout] & (1 << (oldOutEdge.edgeId % 32))) != 0
				g.graphStorage.SetRoundabout(outOffset, roundabout)

				g.outEdges[outOffset].edgeId = outOffset
				g.outEdges[outOffset].head = newIds[oldOutEdge.head]

				outOffset++
			}

			for k := Index(0); k < Index(len(iEdges[vOldId])); k++ {
				oldInEdge := iEdges[vOldId][k]
				g.inEdges[inOffset] = NewInEdge(
					oldInEdge.edgeId, oldInEdge.tail, oldInEdge.weight,
					oldInEdge.dist, oldInEdge.exitPoint,
				)
				g.inEdges[inOffset].edgeId = inOffset
				g.inEdges[inOffset].tail = newIds[oldInEdge.tail]
				inOffset++
			}

			vId++
		}
	}

	newVertices[len(newVertices)-1] = lastVertex
	g.SetVertices(newVertices)

}

func (g *Graph) findCellVerticesThatContainV(vId Index, cellVertices [][]struct {
	vertex        *Vertex
	originalIndex Index
}) *Vertex {
	for i, _ := range cellVertices {
		for _, v := range cellVertices[i] {
			if v.vertex.id == vId {
				return v.vertex
			}
		}
	}
	return nil
}
