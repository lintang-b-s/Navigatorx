package main

import (
	"encoding/csv"
	"flag"
	"fmt"
	"os"
	"strconv"
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
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

	// n := g.NumberOfVertices()
	n := 2000

	outRecs := make([][]string, n*24)
	for v := 0; v < n*24; v++ {
		outRecs[v] = make([]string, n)
	}

	lock := sync.Mutex{}
	type spParam struct {
		s    da.Index
		hour int
	}
	newSPParam := func(s da.Index, hour int) spParam {
		return spParam{s, hour}
	}

	// TODO: all pairs sp dibikin batch (setiap batch dirun 1 goroutine pake goroutine pool),
	// setiap batch (source, to other vertices), tapi other vertices nya gak semua V (karna kalau semua CRP one-to-many bakal sama kaya plain dijkstra) & dibikin random
	// idk belum kepikiran caranya
	
	calcsSP := func(p spParam) any {
		s := p.s
		hour := p.hour
		if s%500 == 0 {
			logger.Sugar().Infof(fmt.Sprintf("calculating earliest arrival query, from: %v, hour: %v ...\n", s, hour))
		}

		atIds := make([]da.Index, 0, n)
		as := g.GetExitOffset(s) + g.GetOutDegree(s) - 1

		for t := da.Index(0); t < da.Index(n); t++ {
			if s == t {
				continue
			}

			at := g.GetEntryOffset(t) + g.GetInDegree(t) - 1

			atIds = append(atIds, at)
		}

		crpQuery := routing.NewCRPUniDijkstraOneToMany(re.GetRoutingEngine())

		spLengths, _, _, _ := crpQuery.ShortestPathOneToManySearch(as, atIds)
		for t, atId := range atIds {
			spLength := spLengths[atId]
			offset := n * hour
			row := int(s) + offset
			lock.Lock()

			outRecs[row][t] = strconv.FormatFloat(spLength, 'f', -1, 64)
			lock.Unlock()
		}

		if s%500 == 0 {
			logger.Sugar().Infof(fmt.Sprintf("done calculating earliest arrival query, from: %v, hour: %v ...\n", s, hour))
		}

		return nil
	}

	workers := concurrent.NewWorkerPool[spParam, any](50, 24*n*n)

	for hour := 0; hour < 24; hour++ {
		for s := da.Index(0); s < da.Index(n); s++ {
			workers.AddJob(newSPParam(s, hour))
		}
	}

	workers.Close()
	workers.Start(calcsSP)
	workers.Wait()

	fout, err := os.Create("allpairs_ea_ny.csv")
	if err != nil {
		panic(err)
	}
	defer fout.Close()

	writer := csv.NewWriter(fout)
	defer writer.Flush()

	if err := writer.WriteAll(outRecs); err != nil {
		panic(err)
	}

}
