package controllers

import (
	"testing"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/stretchr/testify/assert"
)

func TestDTOS(t *testing.T) {
	t.Run("NewDrivingDirection", func(t *testing.T) {
		ins := da.Instruction{}
		ins.SetSuggestAlternatives(true)
		ann := da.NewAnnotation([]float64{}, []float64{}, da.Coordinates{}, []da.Index{})
		d := da.NewDrivingDirection(ins, "turn left", 60.0, 100.0, []da.Index{1}, "polyline", 90.0, ann)

		dd := NewDrivingDirection(d, true)
		assert.Equal(t, "turn left", dd.Instruction)
		assert.Equal(t, 1.0, dd.TravelTime) // 60s / 60
		assert.Equal(t, 100.0, dd.Distance)
		assert.Equal(t, "polyline", dd.Polyline)
		assert.Equal(t, 90.0, dd.TurnBearing)
		assert.True(t, dd.SuggestAlternatives)
	})

	t.Run("NewAlternativeRoutesResponse", func(t *testing.T) {
		alt := routing.AlternativeRoute{}
		alt.SetDrivingTravelTime(120.0)
		alt.SetDist(2000.0)
		alt.SetPolylinePath("polyline")
		ins := da.Instruction{}
		ann := da.NewAnnotation([]float64{}, []float64{}, da.Coordinates{}, []da.Index{})
		alt.SetDrivingDirections([]da.DrivingDirection{
			da.NewDrivingDirection(ins, "continue", 30.0, 50.0, []da.Index{1}, "step", 0.0, ann),
		})

		res := NewAlternativeRoutesResponse([]routing.AlternativeRoute{alt}, true)
		assert.Len(t, res.Routes, 1)
		assert.Equal(t, 2.0, res.Routes[0].TravelTime) // 120s / 60
		assert.Equal(t, 2000.0, res.Routes[0].Dist)
		assert.Len(t, res.Routes[0].DrivingDirections, 1)
	})

	t.Run("gps_ToDataGPS", func(t *testing.T) {
		now := time.Now()
		g := gps{Lat: yogyakartaOriginLat, Lon: yogyakartaOriginLon, Time: now, Speed: 10.0, DeltaTime: 1.0, DeadReckoning: true}
		dg := g.ToDataGPS()
		assert.Equal(t, yogyakartaOriginLat, dg.Lat())
		assert.Equal(t, yogyakartaOriginLon, dg.Lon())
		assert.Equal(t, 10.0, dg.Speed())
	})

	t.Run("NewCandidate", func(t *testing.T) {
		c := NewCandidate(1, 10.5, 100.0)
		assert.Equal(t, da.Index(1), c.EdgeId)
		assert.Equal(t, 10.5, c.Weight)
	})

	t.Run("ToOnlineCandidates", func(t *testing.T) {
		cands := []*Candidate{{EdgeId: 1, Weight: 10.0, Length: 100.0}}
		oCands := ToOnlineCandidates(cands)
		assert.Len(t, oCands, 1)
		assert.Equal(t, da.Index(1), oCands[0].EdgeId())
	})

	t.Run("mapMatchRequest_GetBearing", func(t *testing.T) {
		req := mapMatchRequest{LastBearing: 45.0}
		assert.Equal(t, 45.0, req.GetBearing())
	})

	t.Run("NewMapmatchingResponse With Candidate", func(t *testing.T) {
		now := time.Date(2026, 4, 29, 10, 0, 0, 0, time.UTC)
		gpsPoint := da.NewGPSPoint(yogyakartaOriginLat, yogyakartaOriginLon, now, 10.0, 1.0, false)
		matched := da.NewMatchedGPSPoint(gpsPoint, 1, da.NewCoordinate(yogyakartaOriginLat, yogyakartaOriginLon), 90.0)
		candidates := []*ma.Candidate{ma.NewCandidate(1, 12.0, 120.0)}

		res := NewMapmatchingResponse(matched, candidates, 3.0, 1.5, 90.0)

		assert.Equal(t, da.Index(1), res.MatchedGpsPoint.EdgeId)
		assert.Len(t, res.Candidates, 1)
		assert.Equal(t, 12.0, res.Candidates[0].Weight)
		assert.Equal(t, 3.0, res.SpeedMeanK)
	})
}
