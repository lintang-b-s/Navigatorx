// Package preprocessor handles the preprocessing phase of the Customizable Route Planning (CRP) by delling et al. (2015).
package preprocessor

import (
	"math"

	"github.com/bits-and-blooms/bitset"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/tiler"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/mmcloughlin/geohash"
	"go.uber.org/zap"
)

type Preprocessor[W util.RoutingNumber] struct {
	graph                                                                  *da.Graph
	mlp                                                                    *da.MultilevelPartition
	overlayGraph                                                           *da.OverlayGraph
	logger                                                                 *zap.Logger
	newVIdMap                                                              []da.Index
	newToOldVIdMap                                                         map[da.Index]da.Index
	edgeInfoIds                                                            [][]da.Index
	timeFunction                                                           *costfunction.TimeFunction[W]
	graphFilename, overlayGraphFilename, preprocessingTimeFunctionFilename string
	writeTiles                                                             bool
}

func NewPreprocessor[W util.RoutingNumber](graph *da.Graph, timeFunction *costfunction.TimeFunction[W], mlp *da.MultilevelPartition,
	logger *zap.Logger, gFilename string, ogFilename string, edgeInfoIds [][]da.Index,
) *Preprocessor[W] {
	return &Preprocessor[W]{
		graph:                             graph,
		mlp:                               mlp,
		logger:                            logger,
		newVIdMap:                         make([]da.Index, graph.NumberOfVertices()),
		newToOldVIdMap:                    make(map[da.Index]da.Index, graph.NumberOfVertices()),
		graphFilename:                     gFilename,
		overlayGraphFilename:              ogFilename,
		preprocessingTimeFunctionFilename: costfunction.PreprocessingTimeFunctionPath(gFilename),
		edgeInfoIds:                       edgeInfoIds,
		timeFunction:                      timeFunction,
		writeTiles:                        true,
	}
}

func (p *Preprocessor[W]) SetWriteTiles(writeTiles bool) {
	p.writeTiles = writeTiles
}

// Preprocesssing. Preprocessing (building Overlay Graph) phase. see section 5.1 Metric Independent Preprocessing (Overlay Topology) :  https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
func (p *Preprocessor[W]) PreProcessing(writefile bool) error {
	p.logger.Sugar().Infof("Starting preprocessing step of Customizable Route Planning...")

	p.logger.Sugar().Infof("Building Overlay Graph of each levels...")
	p.BuildCellNumber()
	p.SortByCellNumber()

	p.overlayGraph = da.NewOverlayGraph(p.graph, p.mlp)
	p.logger.Sugar().Infof("Overlay graph built and written to ./data/overlay_graph.ngraph")
	for l := p.overlayGraph.GetLevelInfo().GetLevelCount(); l >= 1; l-- {
		p.logger.Sugar().Infof("overlay graph level %v: number of overlay vertices %v", l, p.overlayGraph.NumberOfVerticesInLevel(l))
	}

	p.logger.Sugar().Infof("Running Kosaraju's algorithm to find strongly connected components (SCCs)...")
	p.graph.RunKosaraju()

	p.logger.Sugar().Infof("Writing graph to ./data/original.ngraph")

	if writefile {
		err := p.overlayGraph.WriteToFile(p.overlayGraphFilename)
		if err != nil {
			return err
		}

		// write graph tiles
		if p.writeTiles {
			tilingEngine := tiler.NewTilingEngine(p.graph, p.logger, p.timeFunction)
			if err := tilingEngine.PreprocessTiles(); err != nil {
				return err
			}
		}

		if err := p.graph.WriteGraph(p.graphFilename); err != nil {
			return err
		}
		return p.timeFunction.WritePreprocessingToFile(p.preprocessingTimeFunctionFilename)
	}
	return nil
}

func (p *Preprocessor[W]) BuildCellNumber() {
	cellNumbers := make([]da.Pv, 0, p.mlp.GetNumberOfCellsInLevel(0))
	pvMap := make(map[da.Pv]da.Index, p.mlp.GetNumberOfCellsInLevel(0))
	p.graph.ForVertices(func(_ da.Vertex, id da.Index) {

		cellNumber := p.mlp.GetCellNumber(id)
		if _, exists := pvMap[cellNumber]; !exists {
			cellNumbers = append(cellNumbers, cellNumber)
			cellPvPtr := len(cellNumbers) - 1
			pvMap[cellNumber] = da.Index(cellPvPtr)
			p.graph.SetVertexPvPtr(id, da.Index(cellPvPtr)) // set pointer to the index in cellNumbers slice
		} else {
			p.graph.SetVertexPvPtr(id, pvMap[cellNumber])
		}
	})

	// cellNumbers contains all unique bitpacked cell numbers from level 0->L.
	p.graph.SetCellNumbers(cellNumbers)
}

/*
group vertices s.t. vertices within the same cell are adjacent to each other
*/
func (p *Preprocessor[W]) SortByCellNumber() {
	cellVertices := make([][]struct {
		vertex        da.Vertex
		originalIndex da.Index
	}, p.graph.GetNumberOfCellsNumbers()) // slice of slice of vertices in each cell

	minLat, minLon := math.MaxFloat64, math.MaxFloat64
	maxLat, maxLon := math.Inf(-1), math.Inf(-1)

	numOutEdgesInCell := make([]da.Index, p.graph.GetNumberOfCellsNumbers()) // number of outEdges in each cell
	numInEdgesInCell := make([]da.Index, p.graph.GetNumberOfCellsNumbers())

	oEdges := make([][]da.OutEdge, p.graph.NumberOfVertices()) // copy of original outEdges of each vertex
	iEdges := make([][]da.InEdge, p.graph.NumberOfVertices())

	p.graph.SetMaxEdgesInCell(da.Index(0)) // maximum number of edges in any cell
	for i := da.Index(0); i < da.Index(p.graph.NumberOfVertices()); i++ {
		cell := p.graph.GetVertexPvPtr(i) // cellNumber
		v := p.graph.GetVertex(i)

		cellVertices[cell] = append(cellVertices[cell], struct {
			vertex        da.Vertex
			originalIndex da.Index
		}{vertex: v, originalIndex: i})

		oEdges[i] = make([]da.OutEdge, p.graph.GetOutDegree(i))
		iEdges[i] = make([]da.InEdge, p.graph.GetInDegree(i))

		k := da.Index(0)
		eOut := p.graph.GetVertexFirstOut(i)
		for eOut < p.graph.GetVertexFirstOut(i+1) {
			oEdge := p.graph.GetOutEdge(eOut)
			newOEdge := da.NewOutEdge(
				oEdge.GetEdgeId(),
				oEdge.GetHead(),
				oEdge.GetEntryPoint(),
				oEdge.GetHighwayType(),
			)
			newOEdge.SetFlag(oEdge.GetFlag())
			oEdges[i][k] = newOEdge
			eOut++
			k++
		}

		k = da.Index(0)
		eIn := p.graph.GetVertexFirstIn(i)
		for eIn < p.graph.GetVertexFirstIn(i+1) {
			inEdge := p.graph.GetInEdge(eIn)
			newInEdge := da.NewInEdge(
				inEdge.GetEdgeId(),
				inEdge.GetTail(),
				inEdge.GetExitPoint(),
				inEdge.GetHighwayType(),
			)
			newInEdge.SetFlag(inEdge.GetFlag())
			iEdges[i][k] = newInEdge
			eIn++
			k++
		}

		numOutEdgesInCell[cell] += p.graph.GetOutDegree(i)
		numInEdgesInCell[cell] += p.graph.GetInDegree(i)

		if p.graph.GetMaxEdgesInCell() < numOutEdgesInCell[cell] {
			p.graph.SetMaxEdgesInCell(numOutEdgesInCell[cell])
		}

		if p.graph.GetMaxEdgesInCell() < numInEdgesInCell[cell] {
			p.graph.SetMaxEdgesInCell(numInEdgesInCell[cell])
		}

		minLat = min(minLat, p.graph.GetVertexCoordinate(i).GetLat())
		minLon = min(minLon, p.graph.GetVertexCoordinate(i).GetLon())
		maxLat = max(maxLat, p.graph.GetVertexCoordinate(i).GetLat())
		maxLon = max(maxLon, p.graph.GetVertexCoordinate(i).GetLon())
	}

	p.graph.SetBoundingBox(da.NewBoundingBox(minLat, minLon, maxLat, maxLon))

	maxNumVerticesInCell := 0
	p.newVIdMap = make([]da.Index, p.graph.NumberOfVertices()) // new vertex id after sorting by cell number
	newVid := da.Index(0)                                      // new vertex id after sorting by cell number
	for i := 0; i < len(cellVertices); i++ {
		for v := 0; v < len(cellVertices[i]); v++ {
			p.newVIdMap[cellVertices[i][v].originalIndex] = newVid
			p.newToOldVIdMap[newVid] = cellVertices[i][v].originalIndex
			newVid++
		}
		numVerticesInCell := len(cellVertices[i])
		maxNumVerticesInCell = max(maxNumVerticesInCell, numVerticesInCell)
	}
	p.graph.SetMaxNumVerticesInCell(da.Index(maxNumVerticesInCell))

	newOutEdgeId := da.Index(0)                                      // new id for outEdges for each vertex for each cell
	p.graph.MakeOutEdgeCellOffset(p.graph.GetNumberOfCellsNumbers()) // offset of first outEdge for each cell
	newInEdgeId := da.Index(0)                                       // new id for inEdges for each vertex for each cell
	p.graph.MakeInEdgeCellOffset(p.graph.GetNumberOfCellsNumbers())  // offset of first inEdge for each cell

	// create new edge metadatas
	newOsmWayIds := da.NewPackedSlice(p.graph.GetOsmWayBitSize(), uint64(p.graph.NumberOfEdges()))
	newEdgeStartPointsIndex := make([]da.Index, p.graph.NumberOfEdges())
	newEdgeEndPointsIndex := make([]da.Index, p.graph.NumberOfEdges())
	newStreetNameIds := make([]uint32, p.graph.NumberOfEdges())
	newRoadClass := make([]pkg.OsmHighwayType, p.graph.NumberOfEdges())
	newRoadClassLink := make([]pkg.OsmHighwayType, p.graph.NumberOfEdges())
	newLanes := make([]uint8, p.graph.NumberOfEdges())

	// create new vertices osm ids
	newVerticesOsmIds := da.NewPackedSlice(da.BIT_SIZE_OSM_NODE_ID, uint64(p.graph.NumberOfVertices())+1)

	// create new roundabout flag
	oldRoundaboutFlag := p.graph.GetRoundaboutFlag()
	newRoundaboutFlags := bitset.New(oldRoundaboutFlag.Len())

	// create new traffic light flag
	oldGraphTrafficLight := p.graph.GetNodeTrafficLight()
	newNodeTrafficLight := bitset.New(oldGraphTrafficLight.Len())

	// create new street direction flags
	newStreetDirectionForward := bitset.New(uint(p.graph.NumberOfEdges()))
	newStreetDirectionBackward := bitset.New(uint(p.graph.NumberOfEdges()))

	// create new is curved flags
	oldIsCurvedFlag := p.graph.GetIsCurvedFlag()
	newIsCurvedFlag := bitset.New(oldIsCurvedFlag.Len())

	vId := da.Index(0)
	isRoadNetworkGraph := p.graph.IsRoadNetworkGraph()

	lastVertex := p.graph.GetVertex(da.Index(p.graph.GetNumberOfVerticesWithDummyVertex() - 1))
	newVertices := make([]da.Vertex, p.graph.GetNumberOfVerticesWithDummyVertex())

	edgeGeohashes := make([]uint32, p.graph.NumberOfEdges())

	newVIdToOldVId := make([]da.Index, p.graph.NumberOfVertices())
	oldOutEdgeIDs := make([]da.Index, 0, p.graph.NumberOfOutEdges())
	for i := da.Index(0); i < da.Index(p.graph.GetNumberOfCellsNumbers()); i++ {
		p.graph.SetOutEdgeCellOffset(i, newOutEdgeId)
		p.graph.SetInEdgeCellOffset(i, newInEdgeId)

		for v := da.Index(0); v < da.Index(len(cellVertices[i])); v++ {
			// update vertex to use new vId
			// in the end of the outer loop, graph vertices are sorted by cell number

			newVertices[vId] = cellVertices[i][v].vertex

			newVIdToOldVId[vId] = cellVertices[i][v].originalIndex

			vOldId := cellVertices[i][v].originalIndex
			newVertices[vId].SetFirstOut(newOutEdgeId)
			newVertices[vId].SetFirstIn(newInEdgeId)
			newVertices[vId].SetId(vId)

			// update new vertex osm id
			newVerticesOsmIds.Append(p.graph.GetVertexOsmId(vOldId))

			// update traffic light flag
			isTraficLight := oldGraphTrafficLight.Test(uint(vOldId))
			if isTraficLight {
				newNodeTrafficLight.Set(uint(vId))
			}

			// update outedges & inedges
			for k := da.Index(0); k < da.Index(len(oEdges[vOldId])); k++ {

				oldOutEdge := oEdges[vOldId][k]

				newOutEdgeHead := p.newVIdMap[oldOutEdge.GetHead()]

				newOutEdge := da.NewOutEdge(
					newOutEdgeId, newOutEdgeHead, oldOutEdge.GetEntryPoint(), oldOutEdge.GetHighwayType(),
				)
				newOutEdge.SetFlag(oldOutEdge.GetFlag())

				p.graph.SetOutEdge(newOutEdgeId, newOutEdge)
				oldOutEdgeIDs = append(oldOutEdgeIDs, oldOutEdge.GetEdgeId())

				// update edge metadata
				vExitPoint := p.graph.GetExitOrder(vOldId, oldOutEdge.GetEdgeId())
				oldEdgeInfoId := p.edgeInfoIds[vOldId][vExitPoint]

				if isRoadNetworkGraph && oldEdgeInfoId != da.INVALID_EDGE_INFO_ID { // skip dummy edge (vOldI, vOldId)
					oldOsmWayId := p.graph.GetOsmWayId(oldEdgeInfoId)
					newOsmWayIds.Append(uint64(oldOsmWayId))

					oldEdgePointsStartIndex, oldEdgePointsEndIndex := p.graph.GetEdgePointsIndices(oldEdgeInfoId)
					newEdgeEndPointsIndex[newOutEdgeId] = oldEdgePointsEndIndex
					newEdgeStartPointsIndex[newOutEdgeId] = oldEdgePointsStartIndex

					oldStreetNameId := p.graph.GetStreetNameId(oldEdgeInfoId)
					newStreetNameIds[newOutEdgeId] = oldStreetNameId

					oldRoadClass := p.graph.GetRoadClass(oldEdgeInfoId)
					newRoadClass[newOutEdgeId] = pkg.GetHighwayType(oldRoadClass)

					oldRoadClassLink := p.graph.GetRoadClassLink(oldEdgeInfoId)
					newRoadClassLink[newOutEdgeId] = pkg.GetHighwayType(oldRoadClassLink)

					oldLanes := p.graph.GetRoadLanes(oldEdgeInfoId)
					newLanes[newOutEdgeId] = oldLanes

					// update roundabout flag
					isRoundabout := oldRoundaboutFlag.Test(uint(oldEdgeInfoId))
					if isRoundabout {
						newRoundaboutFlags.Set(uint(newOutEdgeId))
					}

					// update street direction flag
					streetdir := p.graph.GetStreetDirection(oldOutEdge.GetEdgeId())
					if streetdir[0] {
						newStreetDirectionForward.Set(uint(newOutEdgeId))
					}

					if streetdir[1] {
						newStreetDirectionBackward.Set(uint(newOutEdgeId))
					}

					// update is curved flag
					isCurved := oldIsCurvedFlag.Test(uint(oldEdgeInfoId))
					if isCurved {
						newIsCurvedFlag.Set(uint(newOutEdgeId))
					}

					// add edge geohash
					// kita set edge geohash based on tailcoord geohash
					tailCoord := newVertices[vId].GetCoordinate()

					eGeoHash := geohash.EncodeIntWithPrecision(tailCoord.GetLat(), tailCoord.GetLon(), tiler.GeohashBits)
					edgeGeohashes[newOutEdgeId] = uint32(eGeoHash)
				} else if isRoadNetworkGraph {
					newOsmWayIds.Append(uint64(da.INVALID_OSM_WAY_ID))
				}

				newOutEdgeId++
			}

			for k := da.Index(0); k < da.Index(len(iEdges[vOldId])); k++ {
				oldInEdge := iEdges[vOldId][k]

				newInEdgeTail := p.newVIdMap[oldInEdge.GetTail()]
				newInEdge := da.NewInEdge(
					newInEdgeId, newInEdgeTail, oldInEdge.GetExitPoint(),
					oldInEdge.GetHighwayType(),
				)

				newInEdge.SetFlag(oldInEdge.GetFlag())

				p.graph.SetInEdge(newInEdgeId, newInEdge)

				newInEdgeId++
			}

			vId++
		}
	}

	newVertices[len(newVertices)-1] = lastVertex
	p.graph.SetVertices(newVertices)
	p.graph.SetRoundaboutFlags(newRoundaboutFlags)
	p.graph.SetTrafficLightFlags(newNodeTrafficLight)
	p.graph.SetStreetDirection(newStreetDirectionForward, newStreetDirectionBackward)
	p.graph.SetNewEdgeMetadatas(newOsmWayIds, newEdgeStartPointsIndex, newEdgeEndPointsIndex,
		newStreetNameIds, newRoadClass, newRoadClassLink, newLanes)
	p.graph.SetVertexOsmIds(newVerticesOsmIds)
	p.graph.SetIsCurvedFlags(newIsCurvedFlag)
	p.graph.SetEdgeGeohashes(edgeGeohashes)
	p.timeFunction.ReorderEdges(oldOutEdgeIDs)

}

func (p *Preprocessor[W]) GetOldToNewVIdMap() []da.Index {
	return p.newVIdMap
}

func (p *Preprocessor[W]) GetNewToOldVIdMap() map[da.Index]da.Index {
	return p.newToOldVIdMap
}

func (p *Preprocessor[W]) GetOverlayGraph() *da.OverlayGraph {
	return p.overlayGraph
}

func (p *Preprocessor[W]) GetGraph() *da.Graph {
	return p.graph
}

func (p *Preprocessor[W]) GetTimeFunction() *costfunction.TimeFunction[W] {
	return p.timeFunction
}
