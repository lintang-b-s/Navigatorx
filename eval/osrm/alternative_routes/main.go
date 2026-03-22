package main

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math/rand"
	"net/http"
	"os"
	"strconv"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type queryParam struct {
	srcLat  float64
	srcLon  float64
	destLat float64
	destLon float64
}

type OSRMResponse struct {
	Code   string  `json:"code"`
	Routes []Route `json:"routes"`
}

type Route struct {
	Legs []Leg `json:"legs"`
}

type Leg struct {
	Steps []Step `json:"steps"`
}

type Step struct {
	Geometry      string         `json:"geometry"`
	Maneuver      Maneuver       `json:"maneuver"`
	Mode          string         `json:"mode"`
	DrivingSide   string         `json:"driving_side"`
	Name          string         `json:"name"`
	Intersections []Intersection `json:"intersections"`
	Weight        float64        `json:"weight"`
	Duration      float64        `json:"duration"`
	Distance      float64        `json:"distance"`
}

type Maneuver struct {
	BearingAfter  int       `json:"bearing_after"`
	BearingBefore int       `json:"bearing_before"`
	Location      []float64 `json:"location"`
	Type          string    `json:"type"`
	Modifier      string    `json:"modifier,omitempty"`
}

type Intersection struct {
	Out      int       `json:"out,omitempty"`
	In       int       `json:"in,omitempty"`
	Entry    []bool    `json:"entry"`
	Bearings []int     `json:"bearings"`
	Location []float64 `json:"location"`
}

func newQueryParam(srcLat, srcLon, destLat, destLon float64) queryParam {
	return queryParam{srcLat: srcLat, srcLon: srcLon, destLat: destLat, destLon: destLon}
}

func loadQueries(path string) ([]queryParam, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	var queries []queryParam
	br := bufio.NewReader(file)

	for {
		line, err := util.ReadLine(br)
		if err != nil && !errors.Is(err, io.EOF) {
			return []queryParam{}, err
		} else if errors.Is(err, io.EOF) {
			break
		}
		ss := util.Fields(line)

		srcLat, err := strconv.ParseFloat(ss[0], 64)
		if err != nil {
			return []queryParam{}, err
		}
		srcLon, err := strconv.ParseFloat(ss[1], 64)
		if err != nil {
			return []queryParam{}, err
		}
		destLat, err := strconv.ParseFloat(ss[2], 64)
		if err != nil {
			return []queryParam{}, err
		}
		destLon, err := strconv.ParseFloat(ss[3], 64)
		if err != nil {
			return []queryParam{}, err
		}
		queries = append(queries, newQueryParam(srcLat, srcLon, destLat, destLon))
	}

	return queries, nil
}

const (
	numQueries = 5000
)

func main() {
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))

	queries, err := loadQueries("./data/random_queries_1mil_sp_crp_alt_coords.txt")
	if err != nil {
		panic(err)
	}

	successRate := 0.0
	for i := 0; i < numQueries; i++ {

		q := queries[rd.Intn(len(queries))]

		url := fmt.Sprintf(
			"http://localhost:5000/route/v1/driving/%v,%v;%v,%v?overview=false&alternatives=true&steps=true",
			q.srcLon,
			q.srcLat,
			q.destLon,
			q.destLat,
		)

		req, err := http.NewRequest("GET", url, nil)
		if err != nil {
			panic(err)
		}

		req.Header.Set("Content-Type", "application/json")
		req.Header.Set("Accept", "application/json")

		client := &http.Client{}

		resp, err := client.Do(req)
		if err != nil {
			panic(err)
		}
		defer resp.Body.Close()

		var osrmResp OSRMResponse

		err = json.NewDecoder(resp.Body).Decode(&osrmResp)
		if err != nil {
			panic(err)
		}
		if (i+1)%100 == 0 {
			fmt.Printf("processed %d queries\n", i+1)
		}
		if len(osrmResp.Routes) > 1 { 
			// `http://localhost:5000/route/v1/driving/${randomQuery.srcLon},${randomQuery.srcLat};${randomQuery.destLon},${randomQuery.destLat}?overview=false&alternatives=true&steps=true
			// di endpoint osrm diatas, rutes ke 1 adalah shortest path
			// rute 2 dan seterusnya adalah rute alternatives
			successRate++
		}
	}

	successRate /= numQueries
	fmt.Printf("success rate: %f\n", successRate)
}


