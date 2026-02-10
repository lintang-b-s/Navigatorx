package main

import (
	"bufio"
	"flag"
	"fmt"
	"io"
	"os"
	"strconv"
	"sync"
	"time"

	"net/http"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/util"

	_ "net/http/pprof"
)

var (
	timeDependent = flag.Bool("time_dependent", true, "Use Time-Dependent Customizable Route Planning")
)

const (
	graphFile        string = "./data/original.graph"
	overlayGraphFile string = "./data/overlay_graph.graph"
	metricsFile      string = "./data/metrics.txt"
)

func main() {
	flag.Parse()
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	tgprParser := osmparser.NewTPGRParser()

	_, osmWayProfile, err := tgprParser.ParseTGPRFile("./data/NY/NY.coordinate", "./data/NY/NY.tpgr",
		logger)
	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger, *timeDependent, osmWayProfile)
	if err != nil {
		panic(err)
	}

	g := re.GetRoutingEngine().GetGraph()

	type spParam struct {
		row int
		s   da.Index
		t   da.Index
	}
	newSPParam := func(row int, s, t da.Index) spParam {
		return spParam{row, s, t}
	}

	fq, err := os.Open("random_queries_1mil_ea_ny.txt")
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

	lock := sync.Mutex{}

	randfout, err := os.Create("rand_queries_result_1mil_ea_ny4.csv")
	if err != nil {
		panic(err)
	}
	defer randfout.Close()
	w := bufio.NewWriter(randfout)
	defer w.Flush()

	go func() {
		http.ListenAndServe("localhost:6060", nil)
	}()

	calcsSP := func(p spParam) any {

		s := p.s
		t := p.t
		row := p.row

		as := g.GetExitOffset(s) + g.GetOutDegree(s) - 1
		at := g.GetEntryOffset(t) + g.GetInDegree(t) - 1

		rowRec := make([]string, len(hours)*2)
		for j, hour := range hours {
			before := time.Now()
			crpQuery := routing.NewTDCRPUnidirectionalSearch(re.GetRoutingEngine())
			sp, _, _, _, _ := crpQuery.ShortestPathSearch(as, at, float64(hour*3600))
			after := time.Now()
			duration := after.Sub(before)
			rowRec[j] = strconv.FormatFloat(sp, 'f', -1, 64)
			rowRec[j+len(hours)] = strconv.Itoa(int(duration.Milliseconds()))
		}

		lock.Lock()

		for j, v := range rowRec {
			if _, err := fmt.Fprintf(w, "%s", v); err != nil {
				panic(err)
			}
			if j < len(rowRec)-1 {
				fmt.Fprint(w, " ")
			}
		}
		fmt.Fprintf(w, "\n")

		lock.Unlock()
		if (row+1)%1000 == 0 {
			logger.Sugar().Infof("done query %v", row+1)
		}

		return nil
	}

	workers := concurrent.NewWorkerPool[spParam, any](500, n)

	for _, q := range queries {
		workers.AddJob(q)
	}

	workers.Close()
	workers.Start(calcsSP)
	workers.Wait()
}
