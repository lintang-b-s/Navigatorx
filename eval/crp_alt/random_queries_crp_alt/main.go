// Package main provides runtime (and other evaluation metrics) benchmarks of several point-to-point shortest path algorithms.
package main

import (
	"bufio"
	"flag"
	"fmt"
	"io"
	stdLog "log"
	"math/rand"
	"os"
	"slices"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	numQueries            = 10000
	centisecondsToMinutes = 1.0 / 6000
)

func init() {
	config.InitRegionName("jateng_jabar", pkg.ROUTER)
}

func main() {
	flag.Parse()
	logger, err := log.New()
	if err != nil {
		stdLog.Fatal(err.Error())
	}

	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		logger.Fatal(err.Error())
	}
	err = config.ReadConfig(workingDir)
	if err != nil {
		logger.Fatal(err.Error())
	}

	re, err := engine.NewEngine[int32](logger)
	if err != nil {
		logger.Fatal(err.Error())
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
		logger.Fatal(err.Error())
	}
	defer fq.Close()

	br := bufio.NewReader(fq)
	line, err := util.ReadLine(br)
	if err != nil {
		logger.Fatal(err.Error())
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
			logger.Fatal(err2.Error())
		}
		t, err2 := da.ParseTextIndex(ff[1])
		if err2 != nil {
			logger.Fatal(err2.Error())
		}
		queries = append(queries, newSPParam(n, s, t))
		n++
	}

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

		now := time.Now()
		var spPath []da.Index
		var spcost int32
		if alt {

			crpQuery := routing.NewCRPALTQuery(re.GetRoutingEngine())
			spcost, spPath, _ = crpQuery.ShortestPathSearch(s, t)
			dur := time.Since(now).Milliseconds()
			durations += float64(dur)

			eff, numExploredVertices, queryRuntime, pathUnpackingRuntime := crpQuery.GetStats(len(spPath) + 1)
			qRuntime += float64(queryRuntime)
			puRuntime += float64(pathUnpackingRuntime)
			efficiency += eff
			totExploredVertices += numExploredVertices
			travelTimes = append(travelTimes, spcost)
		} else {

			crpQuery := routing.NewCRPQuery(re.GetRoutingEngine())
			spcost, spPath, _ = crpQuery.ShortestPathSearch(s, t)
			dur := time.Since(now).Milliseconds()
			durations += float64(dur)

			eff, numExploredVertices, queryRuntime, pathUnpackingRuntime := crpQuery.GetStats(len(spPath) + 1)
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

	logger.Sugar().Infof("benchmarking query phase combination of Customizable Route Planing (CRP) + ALT (A* Search, landmarks, and triangle inequality) Algorithm...")
	rdStartId := rd.Intn(len(queries) - 10000)
	for i, q := range queries[rdStartId : rdStartId+10000] {
		calcsSP(i, q, true)
	}

	slices.Sort(travelTimes)
	avgCost := 0.0
	minCost := float64(travelTimes[0])
	maxCost := float64(0)
	for _, tt := range travelTimes {

		if tt != util.Infinity[int32]() {
			maxCost = float64(tt)
			avgCost += float64(tt)
		}
	}

	fmt.Printf("Algoritma kueri kombinasi CRP dan ALT (with turn costs) : \n")
	fmt.Printf("avg query times: %f ms\n", durations/numQueries)
	fmt.Printf("avg efficiency: %f %%\n", (efficiency/numQueries)*100)
	fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/numQueries)
	fmt.Printf("avg query runtime: %f ms\n", qRuntime/numQueries)
	fmt.Printf("avg path unpacking runtime: %f ms\n", puRuntime/numQueries)
	fmt.Printf("avg travel time: %f minutes\n", (avgCost/numQueries)*centisecondsToMinutes)
	fmt.Printf("min travel time: %f minutes\n", minCost*centisecondsToMinutes)
	fmt.Printf("max travel time: %f minutes\n", maxCost*centisecondsToMinutes)

	durations = 0.0
	efficiency = 0.0
	qRuntime = 0.0
	puRuntime = 0.0
	totExploredVertices = 0
	travelTimes = travelTimes[:0]

	logger.Sugar().Infof("benchmarking query phase of Customizable Route Planing (CRP) ..")

	rdStartId = rd.Intn(len(queries) - 10000)
	for i, q := range queries[rdStartId : rdStartId+10000] {
		calcsSP(i, q, false)
	}

	slices.Sort(travelTimes)
	avgCost = 0.0
	minCost = float64(travelTimes[0])
	maxCost = 0
	for _, tt := range travelTimes {

		if tt != util.Infinity[int32]() {
			maxCost = float64(tt)
			avgCost += float64(tt)
		}
	}

	fmt.Printf("Algoritma kueri CRP (with turn costs): \n")
	fmt.Printf("avg query times: %f ms\n", durations/numQueries)
	fmt.Printf("avg efficiency: %f %%\n", (efficiency/numQueries)*100)
	fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/numQueries)
	fmt.Printf("avg query runtime: %f ms\n", qRuntime/numQueries)
	fmt.Printf("avg path unpacking runtime: %f ms\n", puRuntime/numQueries)
	fmt.Printf("avg travel time: %f minutes\n", (avgCost/numQueries)*centisecondsToMinutes)
	fmt.Printf("min travel time: %f minutes\n", minCost*centisecondsToMinutes)
	fmt.Printf("max travel time: %f minutes\n", maxCost*centisecondsToMinutes)

	// // gak support turn restrictions & turn costs
	// calcsSPWithoutTurnCosts := func(i int, p spParam, alt bool) any {

	// 	s := p.s
	// 	t := p.t

	// 	now := time.Now()
	// 	if alt {
	// 		crpQuery := routing.NewCRPALTQuery(re.GetRoutingEngine())
	// 		spcost, vertexPath, _ := crpQuery.ShortestPathSearch(s, t)
	// 		dur := time.Since(now).Milliseconds()
	// 		durations += float64(dur)

	// 		eff, numExploredVertices, queryRuntime, pathUnpackingRuntime := crpQuery.GetStats(len(vertexPath))
	// 		qRuntime += float64(queryRuntime)
	// 		puRuntime += float64(pathUnpackingRuntime)
	// 		efficiency += eff
	// 		totExploredVertices += numExploredVertices
	// 		travelTimes = append(travelTimes, spcost)
	// 	} else {

	// 		crpQuery := routing.NewCRPQuery(re.GetRoutingEngine())
	// 		spcost, vertexPath, _ := crpQuery.ShortestPathSearch(s, t)
	// 		dur := time.Since(now).Milliseconds()
	// 		durations += float64(dur)

	// 		eff, numExploredVertices, queryRuntime, pathUnpackingRuntime := crpQuery.GetStats(len(vertexPath))
	// 		qRuntime += float64(queryRuntime)
	// 		puRuntime += float64(pathUnpackingRuntime)
	// 		efficiency += eff
	// 		totExploredVertices += numExploredVertices
	// 		travelTimes = append(travelTimes, spcost)
	// 	}

	// 	if (i+1)%1000 == 0 {
	// 		logger.Sugar().Infof("done query %v", i+1)
	// 	}

	// 	return nil
	// }

	// durations = 0.0
	// efficiency = 0.0
	// qRuntime = 0.0
	// puRuntime = 0.0
	// totExploredVertices = 0
	// travelTimes = travelTimes[:0]

	// rdStartId = rd.Intn(len(queries) - 10000)
	// for i, q := range queries[rdStartId : rdStartId+10000] {
	// 	calcsSPWithoutTurnCosts(i, q, true)
	// }

	// sort.Slice(travelTimes, func(i, j int) bool {
	// 	return travelTimes[i] < travelTimes[j]
	// })
	// avgCost = 0.0
	// minCost = float64(travelTimes[0])
	// maxCost = 0
	// for _, tt := range travelTimes {

	// 	if tt != util.Infinity[int32]() {
	// 		maxCost = float64(tt)
	// 		avgCost += float64(tt)
	// 	}
	// }

	// fmt.Printf("Algoritma kueri kombinasi CRP dan ALT (without turn costs): \n")
	// fmt.Printf("avg query times: %f\n", durations/numQueries)
	// fmt.Printf("avg efficiency: %f\n", efficiency/numQueries)
	// fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/numQueries)
	// fmt.Printf("avg query runtime: %f\n", qRuntime/numQueries)
	// fmt.Printf("avg path unpacking runtime: %f\n", puRuntime/numQueries)
	// fmt.Printf("avg travel time: %f\n", (avgCost/numQueries)*centisecondsToMinutes)
	// fmt.Printf("min travel time: %f\n", minCost*centisecondsToMinutes)
	// fmt.Printf("max travel time: %f\n", maxCost*centisecondsToMinutes)

	// durations = 0.0
	// efficiency = 0.0
	// qRuntime = 0.0
	// puRuntime = 0.0
	// totExploredVertices = 0
	// travelTimes = travelTimes[:0]

	// rdStartId = rd.Intn(len(queries) - 10000)
	// for i, q := range queries[rdStartId : rdStartId+10000] {
	// 	calcsSPWithoutTurnCosts(i, q, false)
	// }

	// sort.Slice(travelTimes, func(i, j int) bool {
	// 	return travelTimes[i] < travelTimes[j]
	// })
	// avgCost = 0.0
	// minCost = float64(travelTimes[0])
	// maxCost = 0
	// for _, tt := range travelTimes {

	// 	if tt != util.Infinity[int32]() {
	// 		maxCost = float64(tt)
	// 		avgCost += float64(tt)
	// 	}
	// }

	// fmt.Printf("Algoritma kueri CRP (without turn costs): \n")
	// fmt.Printf("avg query times: %f\n", durations/numQueries)
	// fmt.Printf("avg efficiency: %f\n", efficiency/numQueries)
	// fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/numQueries)
	// fmt.Printf("avg query runtime: %f\n", qRuntime/numQueries)
	// fmt.Printf("avg path unpacking runtime: %f\n", puRuntime/numQueries)
	// fmt.Printf("avg travel time: %f\n", (avgCost/numQueries)*centisecondsToMinutes)
	// fmt.Printf("min travel time: %f\n", minCost*centisecondsToMinutes)
	// fmt.Printf("max travel time: %f\n", maxCost*centisecondsToMinutes)

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

	logger.Sugar().Infof("benchmarking ALT (A* Search, Landmarks, and Triangle Inequality) Algorithm ..")
	rdStartId = rd.Intn(len(queries) - 10000)
	for i, q := range queries[rdStartId : rdStartId+10000] {
		calcsSPALT(i, q)
	}

	slices.Sort(travelTimes)
	avgCost = 0.0
	minCost = float64(travelTimes[0])
	maxCost = 0
	for _, tt := range travelTimes {

		if tt != util.Infinity[int32]() {
			maxCost = float64(tt)
			avgCost += float64(tt)
		}
	}

	fmt.Printf("Algoritma kueri ALT untuk P2PSP (without turn costs): \n")
	fmt.Printf("avg query times: %f ms\n", durations/numQueries)
	fmt.Printf("avg efficiency: %f %%\n", (efficiency/numQueries)*100)
	fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/numQueries)
	fmt.Printf("avg query runtime: %f ms\n", qRuntime/numQueries)
	fmt.Printf("avg path unpacking runtime: %f ms\n", puRuntime/numQueries)
	fmt.Printf("avg travel time: %f minutes\n", (avgCost/numQueries)*centisecondsToMinutes)
	fmt.Printf("min travel time: %f minutes\n", minCost*centisecondsToMinutes)
	fmt.Printf("max travel time: %f minutes\n", maxCost*centisecondsToMinutes)

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

	logger.Sugar().Infof("benchmarking Dijkstra Algorithm for Point-to-point Shortest path..")
	rdStartId = rd.Intn(len(queries) - 10000)
	for i, q := range queries[rdStartId : rdStartId+10000] {
		calcsSPDijsktra(i, q)
	}

	slices.Sort(travelTimes)
	avgCost = 0.0
	minCost = float64(travelTimes[0])
	maxCost = 0
	for _, tt := range travelTimes {

		if tt != util.Infinity[int32]() {
			maxCost = float64(tt)
			avgCost += float64(tt)
		}
	}

	fmt.Printf("Algoritma kueri Dijkstra untuk P2PSP (without turn costs): \n")
	fmt.Printf("avg query times: %f ms\n", durations/numQueries)
	fmt.Printf("avg efficiency: %f %%\n", (efficiency/numQueries)*100)
	fmt.Printf("avg number of vertices explored: %d\n", totExploredVertices/numQueries)
	fmt.Printf("avg query runtime: %f ms\n", qRuntime/numQueries)
	fmt.Printf("avg path unpacking runtime: %f ms\n", puRuntime/numQueries)
	fmt.Printf("avg travel time: %f minutes\n", (avgCost/numQueries)*centisecondsToMinutes)
	fmt.Printf("min travel time: %f minutes\n", minCost*centisecondsToMinutes)
	fmt.Printf("max travel time: %f minutes\n", maxCost*centisecondsToMinutes)
}
