package main

import (
	"bufio"
	"context"
	"flag"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	crpalt "github.com/lintang-b-s/Navigatorx/eval/crp_alt"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	op "github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// todo: setelah selesai run ini, coba cek corrrectness pakai script mbpC.exe di: https://github.com/lintang-b-s/skripsi_code/tree/main/ch9-1.1/solvers/mlb-dimacs
// todo2: add script download file road network dimacs 9th (https://www.diag.uniroma1.it/~challenge9/download.shtml) + buat jalanin ini + mbpC.exe untuk file query input yang dimasukin di cmd flag oleh user
// todo3: add fitur customizer pakai csv file. baca referensi:  https://github.com/Telenav/open-source-spec/blob/master/osrm/doc/osrm_customization.md
// todo4: setelah todo3 selesai, add driving direction motorway (jalan toll) handler.
// todo5: setelah todo4 selesai, tambahin destination di driving direction (https://wiki.openstreetmap.org/wiki/Key:destination) dan pake tag osm way ini: https://wiki.openstreetmap.org/wiki/Key:turn , https://wiki.openstreetmap.org/wiki/Key:turn:lanes
// setelah 5 todo ini selesai keknya udah aja tinggal update repo skripsi_code
// setelah itu mungkin add test cases lagi buat cek correctness path unpacking, pakai test cases dari soal-soal programming contest?? soalnya di dataset dimacs cuma ada sp cost buat ground truth nya
// untuk saat ini cuma shopping_malls.go buat cek correctness path unpacking
// buat routing with turn restrictions gak ada dataset nya kucari diinternet, jadi cukup pakai dijkstra_with_turn_costs.go aja buat cek correctnessnya

var (
	partitionSizes         = flag.String("us", "8,10,11,12,14", "Multilevel Partition Sizes")
	inputCoordFilePath     = flag.String("input_nodes", "./eval/crp_alt/dimacs_9_shortest_path_alt/USA-road-d.CAL.co", "path ke file .co dimacs 9th shortest path challnge")
	inputEdgesFilePath     = flag.String("input_edges", "./eval/crp_alt/dimacs_9_shortest_path_alt/USA-road-t.CAL.gr", "path ke file .gr dimacs 9th shortest path challnge")
	inputSSQueriesFilePath = flag.String("input_queries", "./eval/crp_alt/dimacs_9_shortest_path_alt/USA-road-t.CAL.ss", "path ke file .ss dimacs 9th shortest path challnge")
	outputFileName         = flag.String("output_name", "DIMACS_9_USA_CAL.ss.chk", "sssp correctness output filename. see: https://www.diag.uniroma1.it/~challenge9/format.shtml#ss.chk")
	problemName            = flag.String("problem_name", "DIMACS_9_P2P_CAL", "problem name")
)

const (
	queryWorkers           = 200
	queryChannelSize       = 100000
	dimacsModulo     int64 = 1 << 62
	progress               = 5000
)

func resolveProjectPath(workingDir, path string) string {
	if filepath.IsAbs(path) {
		return path
	}
	return filepath.Join(workingDir, path)
}

func parsePartitionSizes(raw string) ([]int, error) {
	parts := strings.Split(raw, ",")
	sizes := make([]int, 0, len(parts))
	for _, part := range parts {
		part = strings.TrimSpace(part)
		if part == "" {
			continue
		}
		size, err := strconv.Atoi(part)
		if err != nil {
			return nil, fmt.Errorf("invalid partition size %q: %w", part, err)
		}
		sizes = append(sizes, size)
	}
	if len(sizes) == 0 {
		return nil, fmt.Errorf("empty partition sizes")
	}
	return sizes, nil
}

/*
https://www.diag.uniroma1.it/~challenge9/format.shtml#ss.chk
*/

func main() {
	var (
		err     error
		line    string
		f, fOut *os.File
	)

	if err := os.MkdirAll("./data", 0755); err != nil {
		panic(err)
	}
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	flag.Parse()

	workingDir, err := util.FindProjectWorkingDir()
	err = util.ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}

	inputCoordPath := resolveProjectPath(workingDir, *inputCoordFilePath)
	inputEdgesPath := resolveProjectPath(workingDir, *inputEdgesFilePath)
	inputQueriesPath := resolveProjectPath(workingDir, *inputSSQueriesFilePath)
	outputPath := resolveProjectPath(workingDir, *outputFileName)

	f, err = os.OpenFile(inputCoordPath, os.O_RDONLY, 0644)
	if err != nil {
		panic(fmt.Errorf("could not open test file: %v", inputCoordPath))
	}
	defer f.Close()

	br := bufio.NewReader(f)

	// read comments gak penting
	for i := 0; i < 4; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
	}

	line, err = util.ReadLine(br)
	if err != nil {
		panic(fmt.Errorf("err: %w", err))
	}

	ff := util.Fields(line)
	n, err := util.ParseInt(ff[4]) // number of vertices
	if err != nil {
		panic(fmt.Errorf("err: %w", err))
	}

	// read comments gak penting lagi
	for i := 0; i < 2; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
	}

	nodeCoords := make([]op.NodeCoord, n+1)
	for v := 0; v < n; v++ { // vertex id 0 dummy vertex.. id vertex dari file dimacs mulai dari 1
		line, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		ff := util.Fields(line)
		id, err := util.ParseInt(ff[1])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		x, err := util.ParseInt(ff[2])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		y, err := util.ParseInt(ff[3])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		nodeCoords[id] = op.NewNodeCoord(float64(x), float64(y))
	}

	fInputEdges, err := os.OpenFile(inputEdgesPath, os.O_RDONLY, 0644)
	if err != nil {
		panic(fmt.Errorf("could not open test file: %v", inputEdgesPath))
	}
	defer fInputEdges.Close()

	br = bufio.NewReader(fInputEdges)

	// read comments gak penting
	for i := 0; i < 4; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
	}

	line, err = util.ReadLine(br)
	if err != nil {
		panic(fmt.Errorf("err: %w", err))
	}

	ff = util.Fields(line)
	m, err := util.ParseInt(ff[3]) // number of edges
	if err != nil {
		panic(fmt.Errorf("err: %w", err))
	}

	// read comments gak penting lagi
	for i := 0; i < 2; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
	}

	adjList := make([][]crpalt.PairEdge, n+1)
	minWeight := math.MaxInt64
	maxWeight := math.MinInt64
	for i := 0; i < m; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		ff := util.Fields(line)
		u, err := util.ParseInt(ff[1])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		v, err := util.ParseInt(ff[2])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		weight, err := util.ParseInt(ff[3])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		if weight < minWeight {
			minWeight = weight
		}
		if weight > maxWeight {
			maxWeight = weight
		}

		adjList[u] = append(adjList[u], crpalt.NewPairEdge(v, float64(weight)))
	}

	// read sssp queries
	fInputQueries, err := os.OpenFile(inputQueriesPath, os.O_RDONLY, 0644)
	if err != nil {
		panic(fmt.Errorf("could not open test file: %v", inputQueriesPath))
	}
	defer fInputQueries.Close()

	br = bufio.NewReader(fInputQueries)

	// read comments gak penting
	for i := 0; i < 5; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
	}

	line, err = util.ReadLine(br)
	if err != nil {
		panic(fmt.Errorf("err: %w", err))
	}

	ff = util.Fields(line)
	q, err := util.ParseInt(ff[4]) // number of edges
	if err != nil {
		panic(fmt.Errorf("err: %w", err))
	}

	// read comments gak penting lagi
	for i := 0; i < 2; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
	}

	sources := make([]da.Index, 0, q)
	for i := 0; i < q; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		ff := util.Fields(line)
		s, err := util.ParseInt(ff[1])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		sources = append(sources, da.Index(s))
	}

	type queryRes struct {
		spcost float64
	}

	newQueryRes := func(spCost float64) queryRes {
		return queryRes{spcost: spCost}
	}

	parsedPartitionSizes, err := parsePartitionSizes(*partitionSizes)
	if err != nil {
		panic(err)
	}

	re, g, oldToNewVIdMap, _ := crpalt.BuildCRP(nodeCoords, adjList, n+1, parsedPartitionSizes, *problemName)

	calcSp := func(query crpalt.QueryParam) queryRes {
		id := query.GetId()
		s := oldToNewVIdMap[query.GetSource()]
		t := oldToNewVIdMap[query.GetTarget()]

		inEdgeToS := g.GetDummyInEdgeId(s)
		_, as := g.GetHeadOfInedgeWithOutEdge(inEdgeToS)
		outEdgeFromTarget := g.GetDummyOutEdgeId(t)
		_, at := g.GetTailOfOutedgeWithInEdge(outEdgeFromTarget)
		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0)

		sVertex := g.GetVertex(s)
		tVertex := g.GetVertex(t)
		emptyCoords := make([]da.Coordinate, 0)
		sPhantomNode := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, as, sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
		tPhantomNode := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), at, 0, 0, emptyCoords, emptyCoords)

		sp, _, _, _, _ := crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)
		if (id+1)%progress == 0 {
			logger.Sugar().Infof("done query id: %v/%v", id+1, q*n)
		}

		return newQueryRes(sp)
	}

	logger.Sugar().Infof("start crp query...")

	sourceChecksums := make([]int64, n+1)
	ctx := context.Background()

	for i, s := range sources {
		workers := concurrent.NewWorkerPool[crpalt.QueryParam, queryRes](queryWorkers, queryChannelSize)
		workers.StartWithContext(ctx, calcSp)
		ssspCost := int64(0)
		go func() {
			for res := range workers.CollectResults() {
				qSpCost := int64(res.spcost)
				ssspCost = (ssspCost + qSpCost) % dimacsModulo
			}
		}()

		for t := 1; t < n+1; t++ {
			workers.AddJob(crpalt.NewQueryParam(da.Index((i*n)+(t-1)), s, da.Index(t)))
		}
		
		workers.Close()
		workers.Wait()

		sourceChecksums[s] = ssspCost
		logger.Sugar().Infof("done source %d/%d: %d", i+1, len(sources), s)
	}

	// write to ss correctness output file. see: https://www.diag.uniroma1.it/~challenge9/format.shtml#ss.chk
	fOut, err = os.Create(outputPath)
	if err != nil {
		panic(fmt.Errorf("err: %w", err))
	}
	defer fOut.Close()

	w := bufio.NewWriter(fOut)

	graphFilename := filepath.Base(inputEdgesPath)
	queryFilename := filepath.Base(inputQueriesPath)

	fmt.Fprintf(w, "c 9th DIMACS Implementation Challenge: Shortest Paths\n")
	fmt.Fprintf(w, "c http://www.dis.uniroma1.it/~challenge9\n")
	fmt.Fprintf(w, "c Point-to-point problem checking file\n")
	fmt.Fprintf(w, "c\n")
	fmt.Fprintf(w, "c problem and solver name line (first non-comment line):\n")
	fmt.Fprintf(w, "p chk sp ss multilevel-alt\n")
	fmt.Fprintf(w, "D 0\n")
	fmt.Fprintf(w, "c ----------------------------------------------------------------\n")
	fmt.Fprintf(w, "c\n")
	fmt.Fprintf(w, "c info below refers to graph %s and query file %s:\n", graphFilename, queryFilename)
	fmt.Fprintf(w, "f %s %s\n", graphFilename, queryFilename)
	fmt.Fprintf(w, "c\n")
	fmt.Fprintf(w, "c the graph has %d nodes, %d arcs, and arc weights in [%d,%d]:\n", n, m, minWeight, maxWeight)
	fmt.Fprintf(w, "g %d %d %d %d\n", n, m, minWeight, maxWeight)
	fmt.Fprintf(w, "c\n")
	fmt.Fprintf(w, "c distances for point-to-point queries:\n")

	for s, ssspCost := range sourceChecksums {
		fmt.Fprintf(w, "d %d %d\n", s, ssspCost)
	}

	if err = w.Flush(); err != nil {
		panic(fmt.Errorf("dimacs 9th shortestpath multilevel-alt correctness test: failed to flush bufio writer: %v", err))
	}

	logger.Sugar().Infof("done")
}
