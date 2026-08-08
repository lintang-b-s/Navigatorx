package main

import (
	"bufio"
	"flag"
	"fmt"
	"io"
	"math/rand"
	"os"
	"sort"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	graphFile        string = "./data/profiles/car/jateng_jabar_original.ngraph"
	overlayGraphFile string = "./data/profiles/car/jateng_jabar_overlay_graph.ngraph"
	metricsFile      string = "./data/profiles/car/jateng_jabar_metrics.nmt"
	landmarkFile     string = "./data/profiles/car/jateng_jabar_landmark.nlm"
	timeFunctionFile string = "./data/profiles/car/jateng_jabar_timefunction.ntf"
)

const (
	NUM_QUERIES             = 10000
	CENTISECONDS_TO_MINUTES = 1.0 / 6000
)

func main() {
	flag.Parse()
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		panic(err)
	}
	err = config.ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}

	re, err := engine.NewEngine[int32](graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
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
		hours[i], _ = util.ParseTextInt(h)
	}

	n := 0

	queries := make([]spParam, 0)
	for line, err = util.ReadLine(br); err != io.EOF; line, err = util.ReadLine(br) {
		ff := util.Fields(line)
		s, err2 := da.ParseTextIndex(ff[0])
		if err2 != nil {
			panic(err2)
		}
		t, err2 := da.ParseTextIndex(ff[1])
		if err2 != nil {
			panic(err2)
		}
		queries = append(queries, newSPParam(n, s, t))
		n++
	}

	g := re.GetRoutingEngine().GetGraph()

	logger.Sugar().Infof("starting benchmark")

	travelTimes := []int32{0}
	durations := 0.0
	efficiency := 0.0
	qRuntime := 0.0
	puRuntime := 0.0
	totExploredVertices := 0

	// support turn restrictions & turn costs
	calcsSP := func(i int, p spParam, alt bool) any {

		s := p.s
		t := p.t

		as := g.GetExitOffset(s) + g.GetOutDegree(s) - 1
		at := g.GetEntryOffset(t) + g.GetInDegree(t) - 1

		sVertex := g.GetVertex(s)
		tVertex := g.GetVertex(t)
		emptyCoords := make([]da.Coordinate, 0)
		sPhantomNode := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, as, sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
		tPhantomNode := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), at, 0, 0, emptyCoords, emptyCoords)

		now := time.Now()
		var spEdges []da.Index
		var spcost int32
		if alt {

			crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0)
			spcost, _, _, spEdges, _ = crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)
			dur := time.Since(now).Milliseconds()
			durations += float64(dur)

			eff, numExploredVertices, queryRuntime, pathUnpackingRuntime := crpQuery.GetStats(len(spEdges) + 1)
			qRuntime += float64(queryRuntime)
			puRuntime += float64(pathUnpackingRuntime)
			efficiency += eff
			totExploredVertices += numExploredVertices
			travelTimes = append(travelTimes, spcost)
		} else {

			crpQuery := routing.NewCRPBidirectionalSearch(re.GetRoutingEngine(), 1.0)
			spcost, spEdges, _ = crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)
			dur := time.Since(now).Milliseconds()
			durations += float64(dur)

			eff, numExploredVertices, queryRuntime, pathUnpackingRuntime := crpQuery.GetStats(len(spEdges) + 1)
			qRuntime += float64(queryRuntime)
			puRuntime += float64(pathUnpackingRuntime)
			efficiency += eff
			totExploredVertices += numExploredVertices
			travelTimes = append(travelTimes, spcost)
		}

		if (i+1)%1000 == 0 {
			logger.Sugar().Infof("done query %v", i+1)
		}

		return nil
	}

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))

	rdStartId := rd.Intn(len(queries) - 10000)
	for i, q := range queries[rdStartId : rdStartId+10000] {
		calcsSP(i, q, true)
	}

	sort.Slice(travelTimes, func(i, j int) bool {
		return travelTimes[i] < travelTimes[j]
	})
	avgTravelTime := 0.0
	minTravelTime := float64(travelTimes[0])
	maxTravelTime := float64(0)
	for _, tt := range travelTimes {

		if tt != util.Infinity[int32]() {
			maxTravelTime = float64(tt)
			avgTravelTime += float64(tt)
		}
	}

	fmt.Printf("Algoritma kueri kombinasi CRP dan ALT (with turn costs) : \n")
	fmt.Printf("avg query times: %f\n", durations/NUM_QUERIES)
	fmt.Printf("avg efficiency: %f\n", efficiency/NUM_QUERIES)
	fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/NUM_QUERIES)
	fmt.Printf("avg query runtime: %f\n", qRuntime/NUM_QUERIES)
	fmt.Printf("avg path unpacking runtime: %f\n", puRuntime/NUM_QUERIES)
	fmt.Printf("avg travel time: %f\n", (avgTravelTime/NUM_QUERIES)*CENTISECONDS_TO_MINUTES)
	fmt.Printf("min travel time: %f\n", minTravelTime*CENTISECONDS_TO_MINUTES)
	fmt.Printf("max travel time: %f\n", maxTravelTime*CENTISECONDS_TO_MINUTES)

	durations = 0.0
	efficiency = 0.0
	qRuntime = 0.0
	puRuntime = 0.0
	totExploredVertices = 0
	travelTimes = travelTimes[:0]

	rdStartId = rd.Intn(len(queries) - 10000)
	for i, q := range queries[rdStartId : rdStartId+10000] {
		calcsSP(i, q, false)
	}

	sort.Slice(travelTimes, func(i, j int) bool {
		return travelTimes[i] < travelTimes[j]
	})
	avgTravelTime = 0.0
	minTravelTime = float64(travelTimes[0])
	maxTravelTime = 0
	for _, tt := range travelTimes {

		if tt != util.Infinity[int32]() {
			maxTravelTime = float64(tt)
			avgTravelTime += float64(tt)
		}
	}

	fmt.Printf("Algoritma kueri CRP (with turn costs): \n")
	fmt.Printf("avg query times: %f\n", durations/NUM_QUERIES)
	fmt.Printf("avg efficiency: %f\n", efficiency/NUM_QUERIES)
	fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/NUM_QUERIES)
	fmt.Printf("avg query runtime: %f\n", qRuntime/NUM_QUERIES)
	fmt.Printf("avg path unpacking runtime: %f\n", puRuntime/NUM_QUERIES)
	fmt.Printf("avg travel time: %f\n", (avgTravelTime/NUM_QUERIES)*CENTISECONDS_TO_MINUTES)
	fmt.Printf("min travel time: %f\n", minTravelTime*CENTISECONDS_TO_MINUTES)
	fmt.Printf("max travel time: %f\n", maxTravelTime*CENTISECONDS_TO_MINUTES)

	// gak support turn restrictions & turn costs
	calcsSPWithoutTurnCosts := func(i int, p spParam, alt bool) any {

		s := p.s
		t := p.t

		now := time.Now()
		if alt {
			crpQuery := routing.NewCRPALTBidirectionalSearchWithoutTurnCost(re.GetRoutingEngine())
			spcost, vertexPath, _ := crpQuery.ShortestPathSearch(s, t)
			dur := time.Since(now).Milliseconds()
			durations += float64(dur)

			eff, numExploredVertices, queryRuntime, pathUnpackingRuntime := crpQuery.GetStats(len(vertexPath))
			qRuntime += float64(queryRuntime)
			puRuntime += float64(pathUnpackingRuntime)
			efficiency += eff
			totExploredVertices += numExploredVertices
			travelTimes = append(travelTimes, spcost)
		} else {

			crpQuery := routing.NewCRPBidirectionalSearchWithoutTurnCost(re.GetRoutingEngine())
			spcost, vertexPath, _ := crpQuery.ShortestPathSearch(s, t)
			dur := time.Since(now).Milliseconds()
			durations += float64(dur)

			eff, numExploredVertices, queryRuntime, pathUnpackingRuntime := crpQuery.GetStats(len(vertexPath))
			qRuntime += float64(queryRuntime)
			puRuntime += float64(pathUnpackingRuntime)
			efficiency += eff
			totExploredVertices += numExploredVertices
			travelTimes = append(travelTimes, spcost)
		}

		if (i+1)%1000 == 0 {
			logger.Sugar().Infof("done query %v", i+1)
		}

		return nil
	}

	durations = 0.0
	efficiency = 0.0
	qRuntime = 0.0
	puRuntime = 0.0
	totExploredVertices = 0
	travelTimes = travelTimes[:0]

	rdStartId = rd.Intn(len(queries) - 10000)
	for i, q := range queries[rdStartId : rdStartId+10000] {
		calcsSPWithoutTurnCosts(i, q, true)
	}

	sort.Slice(travelTimes, func(i, j int) bool {
		return travelTimes[i] < travelTimes[j]
	})
	avgTravelTime = 0.0
	minTravelTime = float64(travelTimes[0])
	maxTravelTime = 0
	for _, tt := range travelTimes {

		if tt != util.Infinity[int32]() {
			maxTravelTime = float64(tt)
			avgTravelTime += float64(tt)
		}
	}

	fmt.Printf("Algoritma kueri kombinasi CRP dan ALT (without turn costs): \n")
	fmt.Printf("avg query times: %f\n", durations/NUM_QUERIES)
	fmt.Printf("avg efficiency: %f\n", efficiency/NUM_QUERIES)
	fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/NUM_QUERIES)
	fmt.Printf("avg query runtime: %f\n", qRuntime/NUM_QUERIES)
	fmt.Printf("avg path unpacking runtime: %f\n", puRuntime/NUM_QUERIES)
	fmt.Printf("avg travel time: %f\n", (avgTravelTime/NUM_QUERIES)*CENTISECONDS_TO_MINUTES)
	fmt.Printf("min travel time: %f\n", minTravelTime*CENTISECONDS_TO_MINUTES)
	fmt.Printf("max travel time: %f\n", maxTravelTime*CENTISECONDS_TO_MINUTES)

	durations = 0.0
	efficiency = 0.0
	qRuntime = 0.0
	puRuntime = 0.0
	totExploredVertices = 0
	travelTimes = travelTimes[:0]

	rdStartId = rd.Intn(len(queries) - 10000)
	for i, q := range queries[rdStartId : rdStartId+10000] {
		calcsSPWithoutTurnCosts(i, q, false)
	}

	sort.Slice(travelTimes, func(i, j int) bool {
		return travelTimes[i] < travelTimes[j]
	})
	avgTravelTime = 0.0
	minTravelTime = float64(travelTimes[0])
	maxTravelTime = 0
	for _, tt := range travelTimes {

		if tt != util.Infinity[int32]() {
			maxTravelTime = float64(tt)
			avgTravelTime += float64(tt)
		}
	}

	fmt.Printf("Algoritma kueri CRP (without turn costs): \n")
	fmt.Printf("avg query times: %f\n", durations/NUM_QUERIES)
	fmt.Printf("avg efficiency: %f\n", efficiency/NUM_QUERIES)
	fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/NUM_QUERIES)
	fmt.Printf("avg query runtime: %f\n", qRuntime/NUM_QUERIES)
	fmt.Printf("avg path unpacking runtime: %f\n", puRuntime/NUM_QUERIES)
	fmt.Printf("avg travel time: %f\n", (avgTravelTime/NUM_QUERIES)*CENTISECONDS_TO_MINUTES)
	fmt.Printf("min travel time: %f\n", minTravelTime*CENTISECONDS_TO_MINUTES)
	fmt.Printf("max travel time: %f\n", maxTravelTime*CENTISECONDS_TO_MINUTES)

	durations = 0.0
	efficiency = 0.0
	qRuntime = 0.0
	puRuntime = 0.0
	totExploredVertices = 0
	travelTimes = travelTimes[:0]

	// ALT p2p
	// gak support turn restrictions & turn costs
	calcsSPALT := func(i int, p spParam) any {

		s := p.s
		t := p.t

		now := time.Now()
		crpQuery := routing.NewALTP2P(re.GetRoutingEngine())
		spcost, vertexPath := crpQuery.ShortestPath(s, t)
		dur := time.Since(now).Milliseconds()
		durations += float64(dur)

		eff, numExploredVertices, queryRuntime := crpQuery.GetStats(len(vertexPath))
		qRuntime += float64(queryRuntime)
		efficiency += eff
		totExploredVertices += numExploredVertices
		travelTimes = append(travelTimes, spcost)

		if (i+1)%100 == 0 {
			logger.Sugar().Infof("done query %v", i+1)
		}
		return nil
	}

	rdStartId = rd.Intn(len(queries) - 10000)
	for i, q := range queries[rdStartId : rdStartId+10000] {
		calcsSPALT(i, q)
	}

	sort.Slice(travelTimes, func(i, j int) bool {
		return travelTimes[i] < travelTimes[j]
	})
	avgTravelTime = 0.0
	minTravelTime = float64(travelTimes[0])
	maxTravelTime = 0
	for _, tt := range travelTimes {

		if tt != util.Infinity[int32]() {
			maxTravelTime = float64(tt)
			avgTravelTime += float64(tt)
		}
	}

	fmt.Printf("Algoritma kueri ALT untuk P2PSP (without turn costs): \n")
	fmt.Printf("avg query times: %f\n", durations/NUM_QUERIES)
	fmt.Printf("avg efficiency: %f\n", efficiency/NUM_QUERIES)
	fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/NUM_QUERIES)
	fmt.Printf("avg query runtime: %f\n", qRuntime/NUM_QUERIES)
	fmt.Printf("avg travel time: %f\n", (avgTravelTime/NUM_QUERIES)*CENTISECONDS_TO_MINUTES)
	fmt.Printf("min travel time: %f\n", minTravelTime*CENTISECONDS_TO_MINUTES)
	fmt.Printf("max travel time: %f\n", maxTravelTime*CENTISECONDS_TO_MINUTES)

	durations = 0.0
	efficiency = 0.0
	qRuntime = 0.0
	puRuntime = 0.0
	totExploredVertices = 0
	travelTimes = travelTimes[:0]

	// dijkstra p2p
	// gak support turn restrictions & turn costs
	calcsSPDijsktra := func(i int, p spParam) any {

		s := p.s
		t := p.t

		now := time.Now()

		crpQuery := routing.NewDijkstraP2P(re.GetRoutingEngine())
		spcost, vertexPath := crpQuery.ShortestPath(s, t)
		dur := time.Since(now).Milliseconds()
		durations += float64(dur)

		eff, numExploredVertices, queryRuntime := crpQuery.GetStats(len(vertexPath))
		qRuntime += float64(queryRuntime)
		efficiency += eff
		totExploredVertices += numExploredVertices
		travelTimes = append(travelTimes, spcost)

		if (i+1)%100 == 0 {
			logger.Sugar().Infof("done query %v", i+1)
		}
		return nil
	}

	rdStartId = rd.Intn(len(queries) - 10000)
	for i, q := range queries[rdStartId : rdStartId+10000] {
		calcsSPDijsktra(i, q)
	}

	sort.Slice(travelTimes, func(i, j int) bool {
		return travelTimes[i] < travelTimes[j]
	})
	avgTravelTime = 0.0
	minTravelTime = float64(travelTimes[0])
	maxTravelTime = 0
	for _, tt := range travelTimes {

		if tt != util.Infinity[int32]() {
			maxTravelTime = float64(tt)
			avgTravelTime += float64(tt)
		}
	}

	fmt.Printf("Algoritma kueri Dijkstra untuk P2PSP (without turn costs): \n")
	fmt.Printf("avg query times: %f\n", durations/NUM_QUERIES)
	fmt.Printf("avg efficiency: %f\n", efficiency/NUM_QUERIES)
	fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/NUM_QUERIES)
	fmt.Printf("avg query runtime: %f\n", qRuntime/NUM_QUERIES)
	fmt.Printf("avg travel time: %f\n", (avgTravelTime/NUM_QUERIES)*CENTISECONDS_TO_MINUTES)
	fmt.Printf("min travel time: %f\n", minTravelTime*CENTISECONDS_TO_MINUTES)
	fmt.Printf("max travel time: %f\n", maxTravelTime*CENTISECONDS_TO_MINUTES)

}
