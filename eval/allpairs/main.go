package main

import (
	"encoding/csv"
	"flag"
	"fmt"
	"math/rand"
	"os"
	"strconv"
	"sync"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
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

	// EPSILON_IMAI_IRI_APPROX_PWL=10%
	tgprParser := osmparser.NewTPGRParser()

	_, osmWayProfile, err := tgprParser.ParseTGPRFile("./data/NY/NY.coordinate", "./data/NY/NY.tpgr",
		logger)
	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger, *timeDependent, osmWayProfile)
	if err != nil {
		panic(err)
	}

	g := re.GetRoutingEngine().GetGraph()

	// n := g.NumberOfVertices()
	n := 1000

	outres := make([][]string, 24*n)

	type spParam struct {
		s    da.Index
		hour int
	}
	newSPParam := func(s da.Index, hour int) spParam {
		return spParam{s, hour}
	}
	fout, err := os.Create("allpairs_ea_ny.csv")
	if err != nil {
		panic(err)
	}
	defer fout.Close()

	batchSize := 500
	lock := sync.Mutex{}

	type target struct {
		tid  da.Index
		atId da.Index
	}

	calcsSP := func(p spParam) any {
		rd := rand.New(rand.NewSource(time.Now().UnixNano()))

		s := p.s
		hour := p.hour
		logger.Sugar().Infof(fmt.Sprintf("calculating earliest arrival query, from: %v, hour: %v ...\n", s, hour))

		as := g.GetExitOffset(s) + g.GetOutDegree(s) - 1

		ts := make([]target, n)
		for t := 0; t < n; t++ {
			at := g.GetEntryOffset(da.Index(t)) + g.GetInDegree(da.Index(t)) - 1
			ts[t] = target{da.Index(t), at}
		}

		rd.Shuffle(n, func(i, j int) {
			ts[i], ts[j] = ts[j], ts[i]
		})
		offset := n * hour
		row := int(s) + offset

		outSpLengths := make([]float64, n)

		for i := 0; i < n; i += batchSize {
			bts := ts[i:util.MinInt(i+batchSize, n)]

			atIds := make([]da.Index, 0, len(bts))
			for _, at := range bts {
				atIds = append(atIds, at.atId)
			}
			crpQuery := routing.NewTDCRPUnidirectionalOneToManySearch(re.GetRoutingEngine())

			spLengths, _, _, _ := crpQuery.ShortestPathOneToManySearch(as, atIds, float64(hour*3600))
			for _, tt := range bts {
				spLength, ok := spLengths[tt.atId]
				if !ok {
					spLength = 2 * pkg.INF_WEIGHT
				}

				outSpLengths[tt.tid] = spLength
			}
		}

		outRecs := make([]string, n)

		for col, sp := range outSpLengths {
			outRecs[col] = strconv.FormatFloat(sp, 'f', -1, 64)
		}

		lock.Lock()
		outres[row] = outRecs
		lock.Unlock()

		logger.Sugar().Infof(fmt.Sprintf("done calculating earliest arrival query, from: %v, hour: %v ...\n", s, hour))

		return nil
	}

	workers := concurrent.NewWorkerPool[spParam, any](50, 24*n)

	for hour := 0; hour < 24; hour += 4 {
		for s := da.Index(0); s < da.Index(n); s++ {
			workers.AddJob(newSPParam(s, hour))
		}
	}

	workers.Close()
	workers.Start(calcsSP)
	workers.Wait()

	writer := csv.NewWriter(fout)
	defer writer.Flush()

	for i := 0; i < 24*n; i++ {
		if outres[i] == nil {
			panic(fmt.Errorf("harusnya gak nil %v", i))
		}
		if err := writer.Write(outres[i]); err != nil {
			panic(err)
		}
	}
}
