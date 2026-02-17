package main

import (
	"bufio"
	"flag"
	"fmt"
	"io"
	"net/http"
	"os"
	"strconv"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	leafBoundingBoxRadius = flag.Float64("leaf_bounding_box_radius", 0.05, "leaf node (r-tree) bounding box radius in km")
	transitionMHTFile     = flag.String("transmht_file", "./data/omm_transition_history_id.mm", "transition matrix for online map-matching Multiple Hypothesis Technique filepath")
)

const (
	graphFile        string = "./data/original.graph"
	overlayGraphFile string = "./data/overlay_graph.graph"
	metricsFile      string = "./data/metrics.txt"
	landmarkFile     string = "./data/landmark.lm"
)

func main() {
	flag.Parse()
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	lm, err := landmark.ReadLandmark(landmarkFile)
	if err != nil {
		panic(err)
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger)
	if err != nil {
		panic(err)
	}

	type spParam struct {
		row int
		s   da.Index
		t   da.Index
	}
	newSPParam := func(row int, s, t da.Index) spParam {
		return spParam{row, s, t}
	}

	fq, err := os.Open("./data/random_queries_1mil_sp_crp_alt.txt")
	if err != nil {
		panic(err)
	}
	defer fq.Close()

	br := bufio.NewReader(fq)
	line, err := util.ReadLine(br)
	if err != nil {
		panic(err)
	}
	hf := util.Fields(line)
	hours := make([]int, len(hf))
	for i, h := range hf {
		hours[i], _ = strconv.Atoi(h)
	}

	n := 0

	queries := make([]spParam, 0)
	for line, err = util.ReadLine(br); err != io.EOF; line, err = util.ReadLine(br) {
		ff := util.Fields(line)
		s, err := da.ParseIndex(ff[0])
		if err != nil {
			panic(err)
		}
		t, err := da.ParseIndex(ff[1])
		if err != nil {
			panic(err)
		}
		queries = append(queries, newSPParam(n, s, t))
		n++
	}

	go func() {
		http.ListenAndServe("localhost:6060", nil)
	}()

	durations := 0.0

	g := re.GetRoutingEngine().GetGraph()

	efficiency := 0.0
	qRuntime := 0.0
	puRuntime := 0.0
	totScannedVertices := 0
	calcsSP := func(p spParam) any {

		s := p.s
		t := p.t
		row := p.row

		as := g.GetExitOffset(s) + g.GetOutDegree(s) - 1
		at := g.GetEntryOffset(t) + g.GetInDegree(t) - 1

		now := time.Now()
		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0, lm)
		_, _, _, spEdges, _ := crpQuery.ShortestPathSearch(as, at)
		dur := time.Since(now).Milliseconds()
		durations += float64(dur)

		eff, numScannedVertices, queryRuntime, pathUnpackingRuntime := crpQuery.GetStats(len(spEdges) + 1)
		qRuntime += float64(queryRuntime)
		puRuntime += float64(pathUnpackingRuntime)
		efficiency += eff
		totScannedVertices += numScannedVertices

		if (row+1)%1000 == 0 {
			logger.Sugar().Infof("done query %v", row+1)
		}

		return nil
	}

	for _, q := range queries[:10000] {
		calcsSP(q)
	}

	fmt.Printf("avg query times: %f\n", durations/10000.0)
	fmt.Printf("avg efficiency: %f\n", efficiency/10000.0)
	fmt.Printf("avg number of vertices scanned: %d\n", totScannedVertices/10000.0)
	fmt.Printf("avg query runtime: %f\n", qRuntime/10000.0)
	fmt.Printf("avg path unpacking runtime: %f\n", puRuntime/10000.0)

}
