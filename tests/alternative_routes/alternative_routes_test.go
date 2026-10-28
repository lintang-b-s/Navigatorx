package alternativeroutes

import (
	"math/rand"
	"testing"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/lintang-b-s/Navigatorx/tests"
)

// TestAlternativeRoutes cuma cek apakah stretch, diversity, success rate dari alternative routes diatas threshold
//
//	https://dl.acm.org/doi/epdf/10.1145/3567421 , https://dl.acm.org/doi/abs/10.1145/2444016.2444019
//
// go test ./tests/alternative_routes -run  TestAlternativeRoutes  -v -timeout=0  -count=1
func TestAlternativeRoutes(t *testing.T) {
	testCases := []struct {
		name               string
		mapFileName        string
		minDiversity       float64
		minSuccessRate     float64
		maxStretch         float64
		maxDistanceSharing float64
		minLocalOptimality float64
		maxRuntime         float64 // in ms
	}{
		{
			name:               "alternative routes yogyakarta Openstreetmap",
			mapFileName:        "yogyakarta",
			minDiversity:       0.1,
			minSuccessRate:     75,
			maxStretch:         25, // dari parameter epsilon di car.yaml/default.yaml, 1+epsilon. maxStretch in %
			maxDistanceSharing: 60, // max distance sharing in %
			minLocalOptimality: 25, // min local optimality in %
			maxRuntime:         10,
		},
		{
			name:               "alternative routes solo Openstreetmap",
			mapFileName:        "solo",
			minDiversity:       0.1,
			minSuccessRate:     75,
			maxStretch:         25, // dari parameter epsilon di car.yaml/default.yaml, 1+epsilon. maxStretch in %
			maxDistanceSharing: 60,
			minLocalOptimality: 25,
			maxRuntime:         10,
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {

			eng, _, _ := tests.Setup(t, tc.mapFileName)
			re := eng.GetRoutingEngine()

			g := re.GetGraph()

			n := 5000
			qset := make(map[uint64]struct{})

			type query struct {
				s, t da.Index
			}

			newQuery := func(s, t da.Index) query {
				return query{
					s: s,
					t: t,
				}
			}

			queries := make([]query, 0, n)

			bitpack := func(i, j da.Index) uint64 {
				return uint64(i) | (uint64(j) << 31)
			}

			rd := rand.New(rand.NewSource(time.Now().UnixNano()))
			V := g.NumberOfVertices()
			i := 0
			for i < n {
				source := da.Index(rd.Intn(V))
				target := da.Index(rd.Intn(V))
				if source == target {
					continue
				}
				if !g.PathExists(source, target) {
					continue
				}
				if _, ok := qset[bitpack(source, target)]; ok {
					continue
				}

				qset[bitpack(source, target)] = struct{}{}
				queries = append(queries, newQuery(source, target))
				i++
			}

			successRate := 0.0
			stretch := 0.0
			locOpt := 0.0
			distSharing := 0.0
			diversity := 0.0
			runtime := 0.0

			foundAltCount := 0

			for i := 0; i < len(queries); i++ {
				source := queries[i].s
				target := queries[i].t

				crpQuery := routing.NewCRPQuery(re) // better via-vertices
				ars := routing.NewAlternativeRouteSearch(re)
				param := ars.ParameterByRequest(source, target)
				crpQuery.SetUpperBound(1.25)
				crpQuery.ForAlternatives(true)
				cost, optPath, found := crpQuery.ShortestPathSearch(source, target)
				if !found {
					continue
				}
				optCost := util.WeightToSeconds(cost)
				alts, dur := ars.ComputeAlternativeRoutes(source, target, 4, false, 0, optCost, optPath, crpQuery, param)
				crpQuery.ForAlternatives(false)
				crpQuery.Done()
				if (i+1)%100 == 0 {
					t.Logf("processed %d queries\n", i+1)
				}
				runtime += float64(dur)

				if len(alts) == 0 {
					continue
				}

				stretch += ars.GetStretch(alts, optCost)
				diversity += ars.GetDiversity(alts)
				locOpt += ars.GetLocalOptimality(alts, optCost)
				distSharing += ars.GetDistanceSharing(alts)
				successRate += 1.0
				foundAltCount++
			}

			// read table I in https://dl.acm.org/doi/epdf/10.1145/2444016.2444019
			successRate /= float64(len(queries))
			successRate *= 100
			stretch /= float64(foundAltCount)
			distSharing /= float64(foundAltCount)
			locOpt /= float64(foundAltCount)
			diversity /= float64(foundAltCount)
			runtime /= float64(len(queries))

			t.Logf("mapfile %s , success rate: %v %%, avg. stretch: %v %%, avg. diversity: %v, avg. distance sharing: %v %%, avg. local optimality: %v %%, avg. runtime: %v ms,\n min success rate: %v %%, min diversity: %v, max stretch: %v %%, max runtime: %v\n",
				tc.mapFileName, successRate, stretch, diversity, distSharing, locOpt, runtime, tc.minSuccessRate, tc.minDiversity, tc.maxStretch, tc.maxRuntime)
			if util.Lt(successRate, tc.minSuccessRate) {
				t.Errorf("want minimum success rate: %v, got: %v", tc.minSuccessRate, successRate)
			}

			if util.Lt(diversity, tc.minDiversity) {
				t.Errorf("want minimum diversity: %v, got: %v", tc.minDiversity, diversity)
			}

			if util.Gt(stretch, tc.maxStretch) {
				t.Errorf("want maximum stretch: %v, got: %v", tc.maxStretch, stretch)
			}

			if util.Lt(locOpt, tc.minLocalOptimality) {
				t.Errorf("want minimum local optimality: %v, got: %v", tc.minLocalOptimality, locOpt)
			}

			if util.Gt(distSharing, tc.maxDistanceSharing) {
				t.Errorf("want maximum distance sharing: %v, got: %v", tc.maxDistanceSharing, distSharing)
			}

			if util.Gt(runtime, tc.maxRuntime) {
				t.Errorf("want maximum runtime: %v, got: %v", tc.maxRuntime, runtime)
			}

		})
	}
}
