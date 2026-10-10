package main

import (
	"bufio"
	"flag"
	"fmt"
	"math"
	"math/rand"
	"net/http"
	_ "net/http/pprof"
	"os"
	"path/filepath"
	"strings"
	"time"

	crpalt "github.com/lintang-b-s/Navigatorx/eval/crp_alt"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	op "github.com/lintang-b-s/Navigatorx/pkg/extractor"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// https://www.diag.uniroma1.it/challenge9/download.shtml
var (
	partitionSizes     = flag.String("us", "8,12,14,18,20", "Multilevel Partition Cells Sizes")
	inputCoordFilePath = flag.String("input_nodes", "./data/USA-road-d.E.co", "path ke file .co dimacs 9th shortest path challenge")
	inputEdgesFilePath = flag.String("input_edges", "./data/USA-road-t.E.gr", "path ke file .gr dimacs 9th shortest path challenge")
	problemName        = flag.String("problem_name", "DIMACS_9_P2P_E", "problem name")
)

// Eastern USA  https://www.diag.uniroma1.it/challenge9/download.shtml:  3.1713 ms/op
// todo: optimize sampai ~1ms/op
const (
	progress = 100
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
		size, err := util.ParseTextInt(part)
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
		err  error
		line string
		f    *os.File
	)

	go func() {
		http.ListenAndServe("localhost:6060", nil)
	}()

	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		panic(err)
	}
	outputDir := filepath.Join(workingDir, "data")
	if err := util.EnsureDirExists(outputDir); err != nil {
		panic(err)
	}
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	flag.Parse()

	err = config.ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}
	config.InitRegionName(*problemName, pkg.EVAL)

	inputCoordPath := resolveProjectPath(workingDir, *inputCoordFilePath)
	inputEdgesPath := resolveProjectPath(workingDir, *inputEdgesFilePath)

	f, err = os.OpenFile(inputCoordPath, os.O_RDONLY, 0600)
	if err != nil {
		panic(fmt.Errorf("could not open test file: %v", inputCoordPath))
	}
	defer f.Close()

	br := bufio.NewReader(f)

	// read comments gak penting
	for i := 0; i < 4; i++ {
		_, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
	}

	line, err = util.ReadLine(br)
	if err != nil {
		panic(fmt.Errorf("err: %w", err))
	}

	ff := util.Fields(line)
	n, err := util.ParseTextInt(ff[4]) // number of vertices
	if err != nil {
		panic(fmt.Errorf("err: %w", err))
	}

	// read comments gak penting lagi
	for i := 0; i < 2; i++ {
		_, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
	}

	const rounder = 1e8

	nodeCoords := make([]op.NodeCoord, n)
	for v := 0; v < n; v++ { // vertex id 0 dummy vertex.. id vertex dari file dimacs mulai dari 1
		line, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		ff := util.Fields(line)
		id, err := util.ParseTextInt(ff[1])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		x, err := util.ParseTextInt(ff[2])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		y, err := util.ParseTextInt(ff[3])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		// di navigatorx versi v0.1.2, weight dari setiap edges pakai tipe generic util.RoutingNumber
		// dan untuk koordinat dari setiap node, kita pakai int32 (lat * 10^7, lon * 10^7) untuk input openstreetmap, buat save space kaya osrm.
		// dan karena di dimacs 9th implementation challenge ini koordinat nya bisa lebih dair 10^8, kita bagi 10^8 biar gak overflow int32

		nodeCoords[id-1] = op.NewNodeCoord(float64(y)/rounder, float64(x)/rounder)
	}

	fInputEdges, err := os.OpenFile(inputEdgesPath, os.O_RDONLY, 0600)
	if err != nil {
		panic(fmt.Errorf("could not open test file: %v", inputEdgesPath))
	}
	defer fInputEdges.Close()

	br = bufio.NewReader(fInputEdges)

	// read comments gak penting
	for i := 0; i < 4; i++ {
		_, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
	}

	line, err = util.ReadLine(br)
	if err != nil {
		panic(fmt.Errorf("err: %w", err))
	}

	ff = util.Fields(line)
	m, err := util.ParseTextInt(ff[3]) // number of edges
	if err != nil {
		panic(fmt.Errorf("err: %w", err))
	}

	// read comments gak penting lagi
	for i := 0; i < 2; i++ {
		_, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
	}

	adjList := make([][]crpalt.PairEdge, n)
	minWeight := math.MaxInt64
	maxWeight := math.MinInt64
	for i := 0; i < m; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		ff := util.Fields(line)
		u, err := util.ParseTextInt(ff[1])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		v, err := util.ParseTextInt(ff[2])
		if err != nil {
			panic(fmt.Errorf("err: %w", err))
		}
		u--
		v--
		weight, err := util.ParseTextInt(ff[3])
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

	type queryRes struct {
		spcost int64
	}

	newQueryRes := func(spCost int64) queryRes {
		return queryRes{spcost: spCost}
	}

	parsedPartitionSizes, err := parsePartitionSizes(*partitionSizes)
	if err != nil {
		panic(err)
	}

	eng, _, _, _ := crpalt.BuildCRP(nodeCoords, adjList, n, parsedPartitionSizes, *problemName)
	re := eng.GetRoutingEngine()
	g := re.GetGraph()

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	V := g.NumberOfVertices()

	nq := 10000
	qset := make(map[uint64]struct{})

	queries := make([]crpalt.QueryParam, 0, nq)

	bitpack := func(i, j da.Index) uint64 {
		return uint64(i) | (uint64(j) << 30)
	}

	i := 0
	for i < n {
		s := da.Index(rd.Intn(V))
		t := da.Index(rd.Intn(V))
		if s == t {
			continue
		}
		if !g.PathExists(s, t) {
			continue
		}
		if _, ok := qset[bitpack(s, t)]; ok {
			continue
		}

		qset[bitpack(s, t)] = struct{}{}
		queries = append(queries, crpalt.NewQueryParam(da.Index(i), s, t))
		i++
	}

	calcSp := func(query crpalt.QueryParam) queryRes {
		id := query.GetId()
		s := query.GetSource()
		t := query.GetTarget()

		crpQuery := routing.NewCRPALTQuery(re)
		sp, _, _ := crpQuery.ShortestPathSearch(s, t)
		if (id+1)%progress == 0 {
			logger.Sugar().Infof("done query id: %v/%v", id+1, nq)
		}

		return newQueryRes(sp)
	}

	start := time.Now()

	logger.Sugar().Infof("start crp query...")

	for i := 0; i < nq; i++ {
		q := queries[i]
		calcSp(q)
	}

	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(nq)
	throughput := float64(nq) / now.Seconds()
	fmt.Printf("avg query runtime: %v ms/op\n", msPerOp)
	fmt.Printf("throughput: %v ops/sec\n", throughput)
	logger.Sugar().Infof("done")
}
