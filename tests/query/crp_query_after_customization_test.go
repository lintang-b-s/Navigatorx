package query

import (
	"context"
	"fmt"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/http/router"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/tests"
)

const (
	segmentFile = "dummy_test_speed_segments.csv"
)

// go test ./tests/query  -v -run TestCRPQueryAfterCustomizationUsingSegmentSpeedsFile
func TestCRPQueryAfterCustomizationUsingSegmentSpeedsFile(t *testing.T) {
	const filename = "yogyakarta"
	eng, logger, cust := tests.Setup(t, filename)
	re := eng.GetRoutingEngine()

	rtree := spatialindex.NewRtree()
	altSearch := routing.NewAlternativeRouteSearch(re)

	g := re.GetGraph()
	rtree.Build(re.GetGraph(), logger)

	routingService, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.05, true)
	ctx, cleanup := router.NewContext(re, routingService)
	defer cleanup()
	routingService.InitBackgroundWorker(ctx)

	if err != nil {
		panic(err)
	}
	type segmentSpeedData struct {
		fromOsmId int64
		toOsmId   int64
		speed     float64 // in km/h
	}

	type queryData struct {
		originLat, originLon float64
		destLat, destLon     float64
	}

	testCases := []struct {
		name                                    string
		updatedSegmentSpeeds                    []segmentSpeedData
		query                                   queryData
		wantToPassThroughTheUpdatedRoadSegments bool
	}{
		{
			name:                                    "blokade jalan Margo Mulyo, query dari pogung ke alun-alun utara jogja",
			wantToPassThroughTheUpdatedRoadSegments: false,
			query:                                   queryData{-7.756559828833062, 110.3765519138064, -7.804635959658114, 110.3651867172938},
			updatedSegmentSpeeds: []segmentSpeedData{
				// jalan margo mulyo
				{
					fromOsmId: 2601239374, // https://www.openstreetmap.org/node/2601239374
					toOsmId:   1472574635,
					speed:     0.1, // 0.1 km/h
				},
				{
					fromOsmId: 1472574635,
					toOsmId:   7650861628,
					speed:     0.1,
				},
				{
					fromOsmId: 7650861628,
					toOsmId:   7650861633,
					speed:     0.1,
				},

				{
					fromOsmId: 7650861633,
					toOsmId:   1664661816,
					speed:     0.1,
				},

				{
					fromOsmId: 1664661816,
					toOsmId:   7652244593,
					speed:     0.1,
				},

				{
					fromOsmId: 7652244593,
					toOsmId:   269398839,
					speed:     0.1,
				},
				{
					fromOsmId: 269398839,
					toOsmId:   7648974730,
					speed:     0.1,
				},
				{
					fromOsmId: 7648974730,
					toOsmId:   3628566588,
					speed:     0.1,
				},
				{
					fromOsmId: 3628566588,
					toOsmId:   12441110867,
					speed:     0.1,
				},
			},
		},

		{
			name:                                    "blokade jalan Jalan Cornelis Simanjuntak, query dari pogung ke gramedia",
			wantToPassThroughTheUpdatedRoadSegments: false,
			query:                                   queryData{-7.756559828833062, 110.3765519138064, -7.783173, 110.374513},
			updatedSegmentSpeeds: []segmentSpeedData{
				// jalan cornelis simanjuntak
				{
					fromOsmId: 13398102142,
					toOsmId:   8474428438,
					speed:     0.5, // 0.5 km/h
				},

				{
					fromOsmId: 8474428438,
					toOsmId:   2564065341,
					speed:     0.5,
				},

				{
					fromOsmId: 2564065341,
					toOsmId:   8482215551,
					speed:     0.5,
				},
				{
					fromOsmId: 8482215551,
					toOsmId:   9963297623,
					speed:     0.5,
				},
				{
					fromOsmId: 9963297623,
					toOsmId:   1664585410,
					speed:     0.5,
				},

				{
					fromOsmId: 1664585410,
					toOsmId:   1664585415,
					speed:     0.5,
				},

				{
					fromOsmId: 1664585415,
					toOsmId:   2647208599,
					speed:     0.5,
				},

				{
					fromOsmId: 2647208599,
					toOsmId:   275864222,
					speed:     0.5,
				},

				{
					fromOsmId: 275864222,
					toOsmId:   2941473421,
					speed:     0.5,
				},

				{
					fromOsmId: 2941473421,
					toOsmId:   5632067096,
					speed:     0.5,
				},
				{
					fromOsmId: 5632067096,
					toOsmId:   10292278109,
					speed:     0.5,
				},
				{
					fromOsmId: 1389399208,
					toOsmId:   5636354958,
					speed:     0.5,
				},
				{
					fromOsmId: 5636354958,
					toOsmId:   5636354955,
					speed:     0.5,
				},
				{
					fromOsmId: 5636354955,
					toOsmId:   7648639908,
					speed:     0.5,
				},
			},
		},
		/*

		 */
	}

	isSame := func(a []uint64, b []uint64) bool {
		if len(a) != len(b) {
			return false
		}

		for i := 0; i < len(a); i++ {
			if a[i] != b[i] {
				return false
			}
		}
		return true
	}

	// path= list of edgeIds
	// kalau tc.wantPassThrough.. == false, kita cek apakah path contains updatedSegmentSpeeds, kalau contains -> return false (incorrect/gak pass test nya)
	isCorrect := func(path []da.Index, updatedSegmentSpeeds []segmentSpeedData, wantPassThrough bool) ([]uint64, bool) {
		containsUpdatedSegments := false
		fullPathOsmIds := []uint64{}
		for i := 0; i < len(path); i++ {

			subPath := make([]uint64, 0, 2)

			edgeId := path[i]

			from := g.GetTailOfOutedge(edgeId)
			to := g.GetHeadOfOutEdge(edgeId)

			fromOsmId := g.GetVertexOsmId(from)
			toOsmId := g.GetVertexOsmId(to)

			subPath = append(subPath, fromOsmId)
			subPath = append(subPath, toOsmId)
			// (fromOsmNodeId, toOsmNodeId)
			if i == 0 {
				fullPathOsmIds = append(fullPathOsmIds, fromOsmId)
			}

			fullPathOsmIds = append(fullPathOsmIds, toOsmId)

			for j := 0; j < len(updatedSegmentSpeeds); j++ {
				updatedSegments := make([]uint64, 0, 2)
				updatedSegments = append(updatedSegments, uint64(updatedSegmentSpeeds[j].fromOsmId))
				updatedSegments = append(updatedSegments, uint64(updatedSegmentSpeeds[j].toOsmId))

				if !wantPassThrough && isSame(subPath, updatedSegments) {
					return subPath, false
				}
				if isSame(subPath, updatedSegments) {
					containsUpdatedSegments = true
				}
			}
		}

		if wantPassThrough && !containsUpdatedSegments {
			return fullPathOsmIds, false
		}

		return []uint64{0}, true
	}

	resetSegmentSpeeds := func() {
		cust.SetEdgeSpeedsFilePath([]string{})
		_, err = cust.Customize()
		if err != nil {
			t.Fatal(err)
		}
	}
	valhallaVisualizerPolylineUrl := func(polyline string) string {
		return fmt.Sprintf("https://valhalla.github.io/demos/polyline/?unescape=true&polyline6=false#%s", polyline)
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			q := tc.query

			// sebelum kustomisasi pakai segment speeds csv file...
			// masih bisa lewat jalan yang diblokade

			// reset kustomisasi every test cases
			resetSegmentSpeeds()

			// cek shortest path route
			_, _, spPolyline, drivingDirections, _, _ := routingService.ShortestPath(context.Background(), q.originLat, q.originLon,
				q.destLat, q.destLon, false, 0)

			spPathEdgeIds := make([]da.Index, 0, len(drivingDirections))
			for _, dd := range drivingDirections {
				legEdgeIds := dd.GetEdgesIds()
				spPathEdgeIds = append(spPathEdgeIds, legEdgeIds...)
			}

			beforeCustomizationWantPassThroughTheUpdatedRoadSegments := true
			if _, correct := isCorrect(spPathEdgeIds, tc.updatedSegmentSpeeds, beforeCustomizationWantPassThroughTheUpdatedRoadSegments); !correct {
				gotPassThrough := false
				if beforeCustomizationWantPassThroughTheUpdatedRoadSegments == false {
					gotPassThrough = true
				}
				t.Errorf("expected pass through the updated road segments: %t, got: %t.\n updated road segments: %v .\n reason: %v", beforeCustomizationWantPassThroughTheUpdatedRoadSegments, gotPassThrough,
					tc.updatedSegmentSpeeds, valhallaVisualizerPolylineUrl(spPolyline))
			}

			// kustomisasi  kustomisasi pakai segment speeds csv file...
			cUpdatedSegments := make([]customizer.UpdatedSegment, len(tc.updatedSegmentSpeeds))
			for k := 0; k < len(tc.updatedSegmentSpeeds); k++ {
				ups := tc.updatedSegmentSpeeds[k]
				cUpdatedSegments[k] = customizer.NewUpdatedSegment(ups.fromOsmId, ups.toOsmId, ups.speed)
			}
			err = customizer.WriteUpdatedSegmentsToCSV(segmentFile, cUpdatedSegments)
			if err != nil {
				t.Fatal(err)
			}

			cust.SetEdgeSpeedsFilePath([]string{segmentFile})
			_, err = cust.Customize()
			if err != nil {
				t.Fatal(err)
			}

			logger.Sugar().Infof("waiting for the routing engine to update its metrics...")
			time.Sleep(2 * time.Second) // biarkan  engine_background_worker.go bekerja dulu
			// setelah kustomisasi, rute yang direturn harus gak lewat jalan jalan yang diblokade (tc.updatedSegments)

			// cek shortest path route
			_, _, _, drivingDirections, _, _ = routingService.ShortestPath(context.Background(), q.originLat, q.originLon,
				q.destLat, q.destLon, false, 0)

			spPathEdgeIdsAfterCustomization := make([]da.Index, 0, len(drivingDirections))
			for _, dd := range drivingDirections {
				legEdgeIds := dd.GetEdgesIds()
				spPathEdgeIdsAfterCustomization = append(spPathEdgeIdsAfterCustomization, legEdgeIds...)
			}

			if reason, correct := isCorrect(spPathEdgeIdsAfterCustomization, tc.updatedSegmentSpeeds, tc.wantToPassThroughTheUpdatedRoadSegments); !correct {
				gotPassThrough := false
				if tc.wantToPassThroughTheUpdatedRoadSegments == false {
					gotPassThrough = true
				}
				t.Errorf("exected pass through the updated road segments: %t, got: %t.\n updated road segments: %v .\n reason: %v.", tc.wantToPassThroughTheUpdatedRoadSegments, gotPassThrough,
					tc.updatedSegmentSpeeds, reason)
			}
		})
	}
}
