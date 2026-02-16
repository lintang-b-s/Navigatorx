package main

import (
	"bufio"
	"flag"
	"fmt"
	"math"
	"math/rand"
	"os"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
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

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger)
	if err != nil {
		panic(err)
	}

	g := re.GetRoutingEngine().GetGraph()

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	V := g.NumberOfVertices()

	fout, err := os.Create("random_queries_1mil_sp_crp_alt_coords.txt")
	if err != nil {
		panic(err)
	}
	defer fout.Close()

	w := bufio.NewWriterSize(fout, 1<<20)
	defer w.Flush()

	n := int(math.Pow(10, 6))
	qset := make(map[uint64]struct{})

	bitpack := func(i, j int) uint64 {
		return uint64(i) | (uint64(j) << 30)
	}

	i := 0
	for i < n {
		s := rd.Intn(V)
		t := rd.Intn(V)

		if s == t {
			continue
		}
		if _, ok := qset[bitpack(s, t)]; ok {
			continue
		}

		if !g.VerticeUandVAreConnected(da.Index(s), da.Index(t)) {
			continue
		}

		qset[bitpack(s, t)] = struct{}{}
		sv := g.GetVertex(da.Index(s))
		tv := g.GetVertex(da.Index(t))
		fmt.Fprintf(w, "%f %f %f %f\n", sv.GetLat(), sv.GetLon(), tv.GetLat(), tv.GetLon())
		i++
	}

}
