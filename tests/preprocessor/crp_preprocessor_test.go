package preprocessor

import (
	"bufio"
	"flag"
	"fmt"
	"io"
	"math"
	"net/http"
	"strings"
	"testing"

	log "github.com/lintang-b-s/Navigatorx/pkg/logger"

	"os"
	"strconv"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	partitionSizes = flag.String("us", "8,10,11,12,14", "Multilevel Partition Sizes")
)

const (
	mlpFile                 = "stress_test_yogyakarta"
	url                     = "https://docs.google.com/uc?export=download&id=1gxrkLPTfuyDl_3KzlcV4MpGXxCKkgDlx"
	osmfFile                = "./data/yogyakarta.osm.pbf"
	graphFile        string = "./data/original_preprocessor_test.graph"
	overlayGraphFile string = "./data/overlay_graph_preprocessor_test.graph"
	metricsFile      string = "./data/metrics_preprocessor_test.txt"
	landmarkFile     string = "./data/landmark_preprocessor_test.lm"
)

type query struct {
	s, t da.Index
}

// cd tests/preprocessor &&  go test -v . --cover -coverpkg=../../pkg/... -coverprofile=prep_coverage.out
// go tool cover -func=prep_coverage.out
// go tool cover -html=prep_coverage.out
func TestPreprocessorSimple(t *testing.T) {

	buildGraph := func(filepath string, cellVertices [][][]da.Index) (*preprocesser.Preprocessor, error) {
		var (
			err  error
			line string
			n, m int
			f    *os.File
		)

		f, err = os.OpenFile(filepath+".in", os.O_RDONLY, 0644)
		if err != nil {
			t.Fatalf("could not open test file: %v", err)
		}
		defer f.Close()

		br := bufio.NewReader(f)

		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff := fields(line)
		n, err = strconv.Atoi(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		m, err = strconv.Atoi(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		var nodeCoords []osmparser.NodeCoord

		for i := 0; i < n; i++ {
			nodeCoords = append(nodeCoords, osmparser.NewNodeCoord(float64(i), float64(i)))
		}

		adjList := make([][]pairEdge, n)
		for i := 0; i < m; i++ {
			line, err = readLine(br)
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			ff := fields(line)
			u, err := strconv.Atoi(ff[0])
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			v, err := strconv.Atoi(ff[1])
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			w, err := strconv.Atoi(ff[2])
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			adjList[u] = append(adjList[u], pairEdge{v, float64(w)})
		}
		es := flattenEdges(adjList)

		op := osmparser.NewOSMParserV2()
		acceptedNodeMap := make(map[int64]osmparser.NodeCoord, n)
		nodeToOsmId := make(map[da.Index]int64, n)
		for i := 0; i < n; i++ {
			acceptedNodeMap[int64(i)] = nodeCoords[i]
			nodeToOsmId[da.Index(i)] = int64(i)
		}

		op.SetAcceptedNodeMap(acceptedNodeMap)
		op.SetNodeToOsmId(nodeToOsmId)

		gs := da.NewGraphStorageWithSize(len(es), n)
		g, edgeInfoIds := op.BuildGraph(es, gs, uint32(n), true, false)

		t.Logf("number of vertices: %v, number of edges: %v", uint32(n), len(es))

		g.SetGraphStorage(gs)

		logger, err := logger.New()
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		mp := partitioner.NewMultilevelPartitioner(
			[]int{int(math.Pow(2, 2)), int(math.Pow(2, 3))},
			2, 1,
			g, logger, true, false, true,
		)
		mp.SetCellVertices(cellVertices)

		mlp := mp.BuildMLP()

		prep := preprocesser.NewPreprocessor(g, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
		err = prep.PreProcessing(false)

		return prep, err
	}

	// https://visualgo.net/en/sssp
	// example graph -> weighted -> big

	bitpack := func(i, j da.Index) uint64 {
		return uint64(i) | (uint64(j) << 30)
	}

	testCases := []struct {
		name               string
		filepath           string
		want               [][]map[uint64]float64 // level -> cellId -> bitpack(source,target) -> st-shortcutWeight
		cellVertices       [][][]da.Index         // level -> cellId -> vertices
		entryVertices      [][][]da.Index         //  level  -> cellId -> entry vertices
		exitVertices       [][][]da.Index         //  level  -> cellId -> exit vertices
		sccs               [][]da.Index           // sccId -> vId
		sccCondensationAdj [][]da.Index           // condensation connection of scc of u -> scc of v

		numOfLevelOneCells    int
		minNumShortcuts       int
		minNumOverlayVertices int
		n                     int // number of vetices
		wantErr               bool
	}{
		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> big
			name:     "visualgo example graph weighted big",
			filepath: "./data/samplegraph/big",
			wantErr:  false,
			cellVertices: [][][]da.Index{
				{ // level 1
					{ // cell 1, level 1. // entry vertices: 1. exit vertices: 1,0
						0, 1,
					},
					{ // cell 2, level 1 // entry vertices: 3. exit vertices: 3,4
						3, 4,
					},
					{ // cell 3, level 1 // entry vertices: 5,7,8. exit vertices: gak ada
						5, 7, 8,
					},
					{ // cell 4, level 1 // entry vertices: 2,6 . exit vertices: 2,6
						2, 6,
					},
				},
				{ // level 2
					{ // cell 1, level 2.  // entry vertices: 3. exit vertices: 3,4,0,1
						0, 1, 3, 4,
					},
					{ // cell 2, level 2. // entry vertices: 2,5,6,7,8. exit vertices: 2
						5, 6, 7, 8, 2,
					},
				},
			},
			entryVertices: [][][]da.Index{
				{
					{ // cell 1, level 1. // entry vertices: 1. exit vertices: 1,0
						1,
					},
					{ // cell 2, level 1 // entry vertices: 3. exit vertices: 3,4
						3,
					},
					{ // cell 3, level 1 // entry vertices: 5,7,8. exit vertices: gak ada
						5, 7, 8,
					},
					{ // cell 4, level 1 // entry vertices: 2,6 . exit vertices: 2,6
						2, 6,
					},
				},
				{ // level 2
					{ // cell 1, level 2.  // entry vertices: 3. exit vertices: 3,4,0,1 // cut edges dari tail 3: (3,5), (3,7), (3,8), (3,6)
						3,
					},
					{ // cell 2, level 2. // entry vertices: 2,5,6,7,8. exit vertices: 2
						5, 6, 7, 8, 2,
					},
				},
			},
			exitVertices: [][][]da.Index{
				{
					{ // cell 1, level 1. // entry vertices: 1. exit vertices: 1,0
						1, 0,
					},
					{ // cell 2, level 1 // entry vertices: 3. exit vertices: 3,4
						3, 4,
					},
					{ // cell 3, level 1 // entry vertices: 5,7,8. exit vertices: gak ada

					},
					{ // cell 4, level 1 // entry vertices: 2,6 . exit vertices: 2,6
						2, 6,
					},
				},
				{ // level 2
					{ // cell 1, level 2.  // entry vertices: 3. exit vertices: 3,4,0,1
						3, 4, 0, 1,
					},
					{ // cell 2, level 2. // entry vertices: 2,5,6,7,8. exit vertices: 2
						2,
					},
				},
			},
			want: [][]map[uint64]float64{
				{
					{
						bitpack(1, 1): 0,
						bitpack(1, 0): pkg.INF_WEIGHT,
					},
					{
						bitpack(3, 3): 0.0,
						bitpack(3, 4): 20.0,
					},
					{},
					{
						bitpack(6, 2): pkg.INF_WEIGHT,
						bitpack(6, 6): 0,
						bitpack(2, 2): 0,
						bitpack(2, 6): 21,
					},
				},
				{
					{
						bitpack(3, 3): 0.0,
						bitpack(3, 4): 20.0,
						bitpack(3, 0): pkg.INF_WEIGHT,
						bitpack(3, 1): 29,
					},
					{
						bitpack(2, 2): 0.0,
						bitpack(5, 2): pkg.INF_WEIGHT, // gak ada path dari 5 ke 2 only pakai edges in cell 2 level 1. edges \in cell C, iff head and tail dari edge \in C.
						bitpack(6, 2): pkg.INF_WEIGHT,
						bitpack(7, 2): pkg.INF_WEIGHT,
						bitpack(8, 2): pkg.INF_WEIGHT,
					},
				},
			},
			minNumShortcuts:       17,
			numOfLevelOneCells:    4,
			n:                     9,
			minNumOverlayVertices: 13,
			sccs: [][]da.Index{
				{
					0,
				},
				{
					1, 2, 3, 4,
				},
				{
					6,
				},
				{
					5,
				},
				{
					7,
				},
				{
					8,
				},
			},
			sccCondensationAdj: [][]da.Index{
				{
					1,
				},
				{
					2, 3, 4, 5,
				},
				{
					5,
				},
				{
					4,
				},
				{
					5,
				},
				{},
			},
		},
		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> cp4 4.10 d/w
			name:     "visualgo example graph weighted cp4 4.10 d/w",
			filepath: "./data/samplegraph/410",
			wantErr:  false,
			cellVertices: [][][]da.Index{
				{ // level 1
					{ // cell 1, level 1. entry vertices: 1, exit vertices: 1
						1,
					},
					{ // cell 2, level 1. entry vertices: 2, exit vertices: 2
						2,
					},
					{ // cell 3, level 1. entry vertices: 0, 3. exit vertices:  0,3
						0, 3,
					},
					{ // cell 4, level 1. entry vertices: 4. exit vertices: gak ada
						4,
					},
				},
				{ // level 2
					{ // cell 1, level 2. entry vertices: 1. exit vertices: 2
						1, 2,
					},
					{ // cell 2, level 2. entry vertices: 4, 0, 3. exit vertices: 0
						4, 0, 3,
					},
				},
			},
			entryVertices: [][][]da.Index{
				{ // level 1
					{ // cell 1, level 1. entry vertices: 1, exit vertices: 1
						1,
					},
					{ // cell 2, level 1. entry vertices: 2, exit vertices: 2
						2,
					},
					{ // cell 3, level 1. entry vertices: 0, 3. exit vertices:  0,3
						0, 3,
					},
					{ // cell 4, level 1. entry vertices: 4. exit vertices: gak ada
						4,
					},
				},
				{ // level 2
					{ // cell 1, level 2. entry vertices: 1. exit vertices: 2
						1,
					},
					{ // cell 2, level 2. entry vertices:  0, 3. exit vertices: 0
						0, 3,
					},
				},
			},
			exitVertices: [][][]da.Index{
				{ // level 1
					{ // cell 1, level 1. entry vertices: 1, exit vertices: 1
						1,
					},
					{ // cell 2, level 1. entry vertices: 2, exit vertices: 2
						2,
					},
					{ // cell 3, level 1. entry vertices: 0, 3. exit vertices:  0,3
						0, 3,
					},
					{ // cell 4, level 1. entry vertices: 4. exit vertices: gak ada

					},
				},
				{ // level 2
					{ // cell 1, level 2. entry vertices: 1. exit vertices: 2
						2,
					},
					{ // cell 2, level 2. entry vertices:  0, 3. exit vertices: 0
						0,
					},
				},
			},
			want: [][]map[uint64]float64{
				{
					{
						bitpack(1, 1): 0,
					},
					{
						bitpack(2, 2): 0,
					},
					{
						bitpack(0, 0): 0,
						bitpack(3, 3): 0,
						bitpack(0, 3): 6,
						bitpack(3, 0): pkg.INF_WEIGHT,
					},
					{},
				},
				{
					{
						bitpack(1, 2): 2,
					},
					{
						bitpack(4, 0): pkg.INF_WEIGHT,
						bitpack(0, 0): 0,
						bitpack(3, 0): pkg.INF_WEIGHT,
					},
				},
			},
			minNumShortcuts:       9,
			numOfLevelOneCells:    4,
			n:                     5,
			minNumOverlayVertices: 8,
			sccs: [][]da.Index{
				{0, 1, 2},
				{3},
				{4},
			},
			sccCondensationAdj: [][]da.Index{
				{
					1, 2,
				},
				{
					2,
				},
				{},
			},
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			prep, err := buildGraph(tc.filepath, tc.cellVertices)
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			newToOldVidMap := prep.GetNewToOldVIdMap()

			// di preprocessor kita melakukan:
			// 1. build g.cellNumbers
			// 2. set cellNumber index dari setiap vertices
			// 3. group vertices by cellNumber
			// 4. build overlay graph
			// 5. bikin strongly connected components

			g := prep.GetGraph()
			og := prep.GetOverlayGraph()
			n := g.NumberOfVertices()

			// 1. cek g.celllNumbers
			cellNumbers := g.GetCellNumbers()
			if len(cellNumbers) != tc.numOfLevelOneCells {
				t.Errorf("expected number of level one cells: %d, got: %d", tc.numOfLevelOneCells, len(cellNumbers))
			}

			// 2. cek cellNumber index dari setiap vertices
			// pertama kita perlu tau setiap vertex ada di cell berapa di setiap level
			numOfLevels := len(tc.cellVertices)
			vertexCells := make([][]da.Pv, numOfLevels) // level -> vertexId -> cellId
			for l := 0; l < numOfLevels; l++ {
				vertexCells[l] = make([]da.Pv, n)
			}
			for l := 0; l < numOfLevels; l++ {
				for cellId := 0; cellId < len(tc.cellVertices[l]); cellId++ {
					for _, v := range tc.cellVertices[l][cellId] {
						vertexCells[l][v] = da.Pv(cellId)
					}
				}
			}

			g.ForVertices(func(v da.Vertex, _ da.Index) {
				vCell := g.GetCellNumber(v.GetID())
				vCellIdInLevelOne := og.GetCellNumberOnLevel(vCell, 1)
				oldVId := newToOldVidMap[v.GetID()]
				expectedCellIdInLevelOne := vertexCells[0][oldVId]
				if vCellIdInLevelOne != expectedCellIdInLevelOne {
					t.Errorf("expected vertex cellId in level 1: %v, got: %v", expectedCellIdInLevelOne, vCellIdInLevelOne)
				}

				for level := 2; level <= og.GetLevelInfo().GetLevelCount(); level++ {
					// validating shortcut weights di level l

					vCellIdInLevelL := og.GetCellNumberOnLevel(vCell, uint8(level))

					expectedCellIdInLevelL := vertexCells[level-1][oldVId]
					if vCellIdInLevelL != expectedCellIdInLevelL {
						t.Errorf("expected vertex cellId in level %d: %v, got: %v", level, expectedCellIdInLevelL, vCellIdInLevelL)
					}

				}
			})

			// 3. group vertices by cellNumber
			// misal cell1: [v1,v2,v3], cell2: [v4,v5], cell3: [v6,v7]
			// contoh g.vertices karena udah g.sortByCellNumber:
			// g.vertices = [v4,v5,v1,v2,v3,v6,v7]
			// sebenarnya sort vertices by its index dari g.cellNumbers
			// kita bisa assert setiap ketemu vertex v pertama dengan cellNumber yg beda dengan cellNumber dari vertex sebelumnya, cek semua vertices mulai dari index v sampai index+len(vCell)-1 cek in vCell

			for v := 0; v < n; {
				vertex := g.GetVertex(da.Index(v))
				vCell := g.GetCellNumber(vertex.GetID())
				// cukup cek level 1
				vCellIdInLevelOne := og.GetCellNumberOnLevel(vCell, 1)
				cellSize := len(tc.cellVertices[0][vCellIdInLevelOne])
				nextV := util.MinInt(v+cellSize, n)
				for i := v; i < nextV; i++ {
					nextVertex := g.GetVertex(da.Index(i))
					nextVertexCell := g.GetCellNumber(nextVertex.GetID())
					nextVertexCell = og.GetCellNumberOnLevel(nextVertexCell, 1)
					if nextVertexCell != vCellIdInLevelOne {
						t.Errorf("expected vertex cellId in level 1: %v, got: %v", vCellIdInLevelOne, nextVertexCell)
					}
				}
				if v+cellSize < n {
					nextCellVertexId := da.Index(v + cellSize)
					nextCellVertex := g.GetCellNumber(nextCellVertexId)
					nextCell := og.GetCellNumberOnLevel(nextCellVertex, 1)
					if nextCell == vCellIdInLevelOne {
						t.Errorf("expected vertex cellId: %v, got: %v", nextCell, vCellIdInLevelOne)
					}
				}

				v = nextV
			}

			// cek 4. build overlay graph

			// cek overlay vertices
			og.ForVertices(func(id da.Index, v da.OverlayVertex) {
				// cek apakah v.neighborOverlayVertex beda cell
				vCell := v.GetCellNumber()

				neighborId := v.GetNeighborOverlayVertex()
				neighborVertex := og.GetVertex(neighborId)
				neighborCell := neighborVertex.GetCellNumber()
				if vCell == neighborCell {
					t.Errorf("expected vertex neighbor cellId different, got same")
				}

				// cek id original vId dari v in range [0,n)
				vId := v.GetOriginalVertex()
				if vId >= da.Index(n) {
					t.Errorf("expected vertex id < n, got: %d", vId)
				}

				// cek total v.entryExitPoint > 0 untuk semua level
				totalEntryExit := int(v.GetEntryPointSize())

				if totalEntryExit == 0 {
					t.Errorf("expected total v.entryExitPoint > 0 untuk semua level, got: %d", totalEntryExit)
				}
				// apa lagi
			})

			// cek og.vertexCountInLevel
			gotTotalCountOverlayVertices := int(og.NumberOfVerticesInLevel(1))

			// og.vertexCountInLevel[l-1] berisi jumlah vertices yang menjadi overlay/boundary vertices di level l cumulative sum dari upper levels (l+1, l+2, ...) .
			// misal ada cut edges: (v1,v3), (v1,v5), (v1,v6)
			// nah ketiga cut edges tsb cell dari tail dan head nya beda di level l..
			// tail sama head ini disebut overlay/boundary vertices di level l.
			// tapi overlay/boundary vertices di level l juga menjadi overlay/boundary vertices di level l-1,l-2,...,1.
			// jadi  og.vertexCountInLevel[1] adalah jumlah dari overlay vertices di level l,l-1,l-2,....,1
			// atau dengan kata lain adalah total unique overlay vertices di OverlayGraph
			// tc.minNumOverlayVertices adalah minimum dari total of overlay vertices
			// minimum karena cuma hitung unique entry/exit vertices, dari contoh cut edges diatas, jumlah v1 cuma dihitung 1 pada tc.minNumOverlayVertices
			// sedangkan pada  og.vertexCountInLevel dihitung 3 kali
			// jadi kita cukup assert gotTotalCountOverlayVertices >=  tc.minNumOverlayVertices atau gak
			if gotTotalCountOverlayVertices < tc.minNumOverlayVertices {
				t.Errorf("expected num overlay vertices greater than or equal to %v, got: %v", tc.minNumOverlayVertices, gotTotalCountOverlayVertices)
			}

			maxOverlayIdOffset := 0
			for l := 1; l <= og.GetLevelInfo().GetLevelCount(); l++ {
				cellMapInLevelL := og.GetAllCellsInLevel(l)

				// cek cellNumber in level l

				expectedNumberOfCellsInLevelL := len(tc.cellVertices[l-1])
				for cellId, cell := range cellMapInLevelL {
					cellIdInLevelL := og.OffUpperBit(cellId, uint8(l))
					if int(cellIdInLevelL) >= expectedNumberOfCellsInLevelL {
						t.Errorf("expected num cells in level %d less than %v, got: %v", l, expectedNumberOfCellsInLevelL, cellIdInLevelL)
					}

					// tc.exitVertices[l-1][cellIdInLevelL] menyimpan unique exit overlay vertices (vertices yang punya setidaknya satu edge yang headnya point to another vertex yang beda cell) di cellIdInLevelL level l
					// tapi pas build OverlayGraph kita bisa simpan duplicates dari exit overlay vertices, karena tergantung berapa jumlah cutEdges yang tail nya exit overlay vertex
					// misal cut edges: (v1, v3), (v1, v5), (v1, v6) ... kita simpan tiga exit overlay vertices yang originalVId nya sama dengan 1 di overlayGraph.overlayVertices dan jumlah exit overlay vertices dari sel v1 ada lebih atau sama dengan 3 (karena v1 duplikat)
					// jadi gotNumExitPoints harusnya lebih besar atau sama dengan len(tc.exitVertices[l-1][cellIdInLevelL])

					expectedMinNumExitPoints := len(tc.exitVertices[l-1][cellIdInLevelL])
					gotNumExitPoints := int(cell.GetNumExitPoints())
					if gotNumExitPoints < expectedMinNumExitPoints {
						t.Errorf("expected num exit overlay vertices in cellId %d level %d greater than or equal to: %v , got: %v", cellIdInLevelL, l,
							expectedMinNumExitPoints, gotNumExitPoints)
					}

					// untuk tc.entryVertices[l-1][cellIdInLevelL] sama dengan penejelasan diatas, tapi entry overlay vertices adlh vertices yang menjadi head dari suatu cut edge
					// contoh: (v1, v3), (v1, v5), (v1, v6) -> entry vertices nya: v3,v5,v6

					expectedMinNumEntryPoints := len(tc.entryVertices[l-1][cellIdInLevelL])
					gotNumEntryPoints := int(cell.GetNumEntryPoints())
					if gotNumEntryPoints < expectedMinNumEntryPoints {
						t.Errorf("expected num entry overlay vertices in cellId %d level %d greater than or equal to: %v , got: %v", cellIdInLevelL, l,
							expectedMinNumEntryPoints, gotNumEntryPoints)
					}

					// urutan iterasi cellMapInLevelL gak sama kayak urutan saat kita build OverlayGraph
					// sedangkan cellOffset dan overlayIdOffset bergantung dari urutan iterasi cellMapInLevelL...
					// shortcut edges adlh edges yang menghubungkan setiap entry overlay vertices ke setiap exit overlay vertices dari setiap cells di setiap level yang weight nya dihitung dengan dijkstra dengan hanya menggunakan vertices & edges di sel tsb
					// jadi kita gak bisa assert cellOffset..
					// overlayIdOffset adlh offset of first entry/exit point (overlay vertex) in og.overlayIdMapping for the each cells
					// overlayIdOffset dihitung dari  int(og.cellMapping[l][cellId].numEntryPoints + og.cellMapping[l][cellId].numExitPoints) setiap kali kita iterate cell di setiap level
					// kita tahu sum over all cell,level  int(og.cellMapping[l][cellId].numEntryPoints + og.cellMapping[l][cellId].numExitPoints) >= tc.minNumOverlayVertices
					// kalau overlayIdOffset kita bisa assert dengan cara:
					// cek max overlayIdOffset >= tc.minNumOverlayVertices

					if int(cell.GetOverlayIdOffset()) > maxOverlayIdOffset {
						maxOverlayIdOffset = int(cell.GetOverlayIdOffset())
					}
				}
			}

			if maxOverlayIdOffset < tc.minNumOverlayVertices {
				t.Errorf("expected maxOverlayIdOffset greater than or equal to %v, got: %v", tc.minNumOverlayVertices, maxOverlayIdOffset)
			}

			if len(og.GetOverlayIdMapping()) < tc.minNumOverlayVertices {
				t.Errorf("expected maxOverlayIdOffset greater than or equal to %v, got: %v", tc.minNumOverlayVertices, len(og.GetOverlayIdMapping()))
			}

			// overlayIdMapping  maps from key = cell.overlayIdOffset + entryExitPoint + (if exit point then + cell.numEntryPoints) to value = overlay entry/exit vertex of a cell (represented as overlay vertex id)
			// kita bisa test overlayIdMapping dengan cara:
			// iterate setiap entry/exit overlay vertices di setiap cells apakah vertices merupakan entry/exit (diasssert dengan cara cek vId in tc.exitVertices/tc.entryVertices)

			for l := 1; l <= og.GetLevelInfo().GetLevelCount(); l++ {
				cellMapInLevelL := og.GetAllCellsInLevel(l)

				// cek cellNumber in level l

				// expectedNumberOfCellsInLevelL := len(tc.cellVertices[l-1])
				for cellId, cell := range cellMapInLevelL {
					cellIdInLevelL := og.OffUpperBit(cellId, uint8(l))

					expectedEntries := tc.entryVertices[l-1][cellIdInLevelL]
					for i := da.Index(0); i < da.Index(cell.GetNumEntryPoints()); i++ {
						entryOvId := og.GetEntryId(cell, i)
						overlayVertex := og.GetVertex(entryOvId)
						oriVId := newToOldVidMap[overlayVertex.GetOriginalVertex()]

						isCorrectEntry := false
						for _, entVId := range expectedEntries {
							if entVId == oriVId {
								isCorrectEntry = true
							}
						}

						if !isCorrectEntry {
							t.Errorf("expected vertex %v in overlay entry vertices in cellId %v level %v", oriVId, cellIdInLevelL, l)
						}
					}

					expectedExit := tc.exitVertices[l-1][cellIdInLevelL]
					for i := da.Index(0); i < da.Index(cell.GetNumExitPoints()); i++ {
						entryOvId := og.GetExitId(cell, i)
						overlayVertex := og.GetVertex(entryOvId)
						oriVId := newToOldVidMap[overlayVertex.GetOriginalVertex()]

						isCorrectExit := false
						for _, entVId := range expectedExit {
							if entVId == oriVId {
								isCorrectExit = true
							}
						}

						if !isCorrectExit {
							t.Errorf("expected vertex %v in overlay exit vertices in cellId %v level %v", oriVId, cellIdInLevelL, l)
						}
					}
				}

			}

			gotShortcutWeightSize := og.GetWeightVectorSize()
			if gotShortcutWeightSize < uint32(tc.minNumShortcuts) {
				t.Errorf("expected shorcut size greater than or equal to: %v, got: %v", uint32(tc.minNumShortcuts), gotShortcutWeightSize)
			}

			levelInfo := og.GetLevelInfo()

			expectedPvOffset := make([]uint8, levelInfo.GetLevelCount()+1)
			for l := 0; l < levelInfo.GetLevelCount(); l++ {
				numCell := len(tc.cellVertices[l])
				expectedPvOffset[l+1] = expectedPvOffset[l] + uint8(math.Ceil(math.Log2(float64(numCell))))
			}

			gotPvOffset := levelInfo.GetOffsets()
			if len(expectedPvOffset) != len(gotPvOffset) {
				t.Errorf("expected pv offset length: %v, got: %v", len(expectedPvOffset), len(gotPvOffset))
			}

			for l := 0; l < len(expectedPvOffset); l++ {
				if expectedPvOffset[l] != gotPvOffset[l] {
					t.Errorf("expected pv offset level %d : %v, got: %v", l, expectedPvOffset[l], gotPvOffset[l])

				}
			}

			// cek 5. bikin strongly connected components
			//

			gotSccs := g.GetSCCS()
			for v := 0; v < n; v++ {
				sccOfV := gotSccs[v]
				oldVId := newToOldVidMap[da.Index(v)]

				vInScc := false
				for _, vv := range tc.sccs[sccOfV] {
					if vv == oldVId {
						vInScc = true
					}
				}
				if !vInScc {
					t.Errorf("expected vertex %v in %v", oldVId, sccOfV)
				}
			}

			gotSccCond := g.GetSCCCondensationAdjList()

			for i := 0; i < len(gotSccCond); i++ {
				if len(gotSccCond[i]) != len(tc.sccCondensationAdj[i]) {
					t.Errorf("expected len(sccCondAdjlist[i]): %v, got: %v", len(tc.sccCondensationAdj[i]), len(gotSccCond[i]))
				}

				for j, to := range tc.sccCondensationAdj[i] {
					if to != gotSccCond[i][j] {
						t.Errorf("expected sccCondAdjlist[i][j]: %v, got: %v", to, gotSccCond[i][j])

					}
				}
			}
		})
	}
}

func setup(t *testing.T, osmFileTest, urlTest string) *preprocesser.Preprocessor {
	if err := os.MkdirAll("./data", 0755); err != nil {
		t.Fatal(err)
	}
	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}
	workingDir, err := util.FindProjectWorkingDir()
	err = util.ReadConfig(workingDir)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := os.Stat(osmFileTest); os.IsNotExist(err) {
		output, err := os.Create(osmFileTest)
		if err != nil {
			t.Fatal(err)
		}
		defer output.Close()

		logger.Sugar().Infof("downloading osm file......")
		response, err := http.Get(urlTest)
		if err != nil {
			t.Fatal(err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		if err != nil {
			t.Fatal(err)
		}
		logger.Sugar().Infof("download complete")
	}

	op := osmparser.NewOSMParserV2()

	graph, edgeInfoIds, err := op.Parse(fmt.Sprintf("%s", osmFileTest), logger, false)
	if err != nil {
		t.Fatal(err)
	}

	pss := strings.Split(*partitionSizes, ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := strconv.Atoi(pss[i])
		if err != nil {
			t.Fatal(err)
		}
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		5,
		graph, logger, true, true, true,
	)

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		t.Fatal(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(fmt.Sprintf("./data/%s", "crp_inertial_flow_"+mlpFile+".mlp"))
	if err != nil {
		t.Fatal(err)
	}
	prep := preprocesser.NewPreprocessor(graph, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
	err = prep.PreProcessing(true)
	if err != nil {
		t.Fatal(err)
	}

	return prep
}

func TestPreprocessUsingOSMFile(t *testing.T) {

	testCases := []struct {
		name string

		urlTest, osmfFileTest string
		roundAboutWay         map[int64]struct{}
		streetNameWay         map[int64]string
		highwayTypeWay        map[int64]string
		roadLanes             map[int64]uint8
	}{
		{
			name:         "file osm yogyakarta",
			urlTest:      url,
			osmfFileTest: osmfFile,
			roundAboutWay: map[int64]struct{}{
				1460805468: struct{}{},
				1460805470: struct{}{},
				1427239361: struct{}{},
			},
			streetNameWay: map[int64]string{
				24277036:  "Jalan Urip Sumoharjo",
				293600459: "Jl. Jenderal Sudirman",
			},
			highwayTypeWay: map[int64]string{
				24277036:  "primary",
				293600459: "primary",
			},
			roadLanes: map[int64]uint8{
				24277036:  3,
				293600459: 4,
			},
		},
	}

	for _, tc := range testCases {

		prep := setup(t, tc.osmfFileTest, tc.urlTest)
		// di preprocessor kita melakukan:
		// 1. build g.cellNumbers
		// 2. set cellNumber index dari setiap vertices
		// 3. group vertices by cellNumber
		// 4. build overlay graph
		// 5. bikin strongly connected components

		// karena step 1-5 udah dicover di TestPreprocessorSimple
		// di prep.SortByCellNumber() kita juga update edge info (edge geometry, edge streetname, edge roadclass/highway type, etc)
		// kita bisa assert edge info aja

		graph := prep.GetGraph()
		n := graph.NumberOfVertices()
		for v := da.Index(0); v < da.Index(n); v++ {
			graph.ForOutEdgeIdsOf(v, func(eId da.Index) {

				tail, _ := graph.GetTailOfOutedgeWithInEdge(eId)
				head := graph.GetHeadOfOutEdge(eId)
				if tail != v {
					t.Errorf("expected tail of outedge (%v, %v): %v, got: %v", v, head, v, tail)
				}

				// cek roundabout
				if _, roundabout := tc.roundAboutWay[graph.GetOsmWayId(eId)]; roundabout && !graph.IsRoundabout(eId) {
					t.Errorf("expected edge with osm way id %v is a roundabout, got no", graph.GetOsmWayId(eId))
				}

				// cek edge geometry
				if len(graph.GetEdgeGeometry(eId)) < 2 {
					t.Errorf("expected number of edge geometry coordinates is greater than or equal to 2, got: %v", len(graph.GetEdgeGeometry(eId)))
				}

				// cek street name dari edge

				eOsmwayId := graph.GetOsmWayId(eId)
				gotStreetName := graph.GetStreetName(eId)
				if expectedStreetname, ok := tc.streetNameWay[eOsmwayId]; ok && expectedStreetname != gotStreetName {
					t.Errorf("expected edge with osm way id %v street name: %v, got: %v", eOsmwayId, expectedStreetname, gotStreetName)
				}

				gotRoadClass := graph.GetRoadClass(eId)
				if expectedHighwayType, ok := tc.highwayTypeWay[eOsmwayId]; ok && expectedHighwayType != gotRoadClass {
					t.Errorf("expected edge with osm way id %v highway type: %v, got: %v", eOsmwayId, expectedHighwayType, gotRoadClass)
				}

				gotRoadLanes := graph.GetRoadLanes(eId)
				if roadLane, ok := tc.roadLanes[eOsmwayId]; ok && roadLane != gotRoadLanes {
					t.Errorf("expected edge with osm way id %v road lanes: %v, got: %v", eOsmwayId, roadLane, gotRoadLanes)
				}
			})
		}
	}

}
