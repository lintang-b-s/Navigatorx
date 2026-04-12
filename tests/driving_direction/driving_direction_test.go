package drivingdirection

import (
	"context"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/tests"
)

// cd tests/driving_direction &&  go test -v .
func TestDrivingDirection(t *testing.T) {

	const fileUrl = "https://docs.google.com/uc?export=download&id=1mO1xPsFrGwYrT4DQFMtN5ajvvQOgP0M7"
	const filename = "solo"

	eng, logger, _ := tests.Setup(t, filename, fileUrl)
	re := eng.GetRoutingEngine()
	rtree := spatialindex.NewRtree()
	altSearch := routing.NewAlternativeRouteSearch(re)

	rtree.Build(re.GetGraph(), logger)

	routingService, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.05, true, true)
	if err != nil {
		panic(err)
	}
	testCases := []struct {
		name              string
		qOriginCoord      da.Coordinate
		qDestinationCoord da.Coordinate
		wantDirections    []string
	}{
		{
			// https://www.google.com/maps/dir/-7.550422162941061,+110.78207364612717/-7.575219943065335,+110.8267931836022/@-7.5622046,110.7945988,15z/am=t/data=!3m1!4b1!4m20!4m19!1m13!2m2!1d110.7820736!2d-7.5504222!3m4!1m2!1d110.8066484!2d-7.5629727!3s0x2e7a1429e8e625bd:0x3f548995f93a09f3!3m4!1m2!1d110.8241434!2d-7.5755624!3s0x2e7a166434dc6a4d:0xa9ffed9ee89235cd!1m3!2m2!1d110.8267932!2d-7.5752199!3e0?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D
			// https://valhalla.github.io/demos/polyline/?unescape=true&polyline6=false#%60uam%40_cdcTAFCN%3F%3FGd%40Qp%40%3F%3FYQOIIEICYEUC%3F%3FuAG%3F%3FXoA%3F%3F%7BDoA%3F%3FKC%3F%3FME%3F%3FFW%3F%3FR_A%3F%3FJa%40%3F%3FNu%40%3F%3FNs%40%3F%3FJm%40%3F%3FLk%40%3F%3FLm%40%3F%3FDM%3F%3FJe%40%3F%3FJa%40%3F%3FFY%3F%3FDS%3F%3FLm%40%3F%3FVoA%3F%3FXkA%3F%3F%5C%5C_B%3F%3FJg%40%3F%3FJa%40%3F%3FPs%40%3F%3FLo%40%3F%3FH%5D%3F%3FJe%40%3F%3FNq%40Lk%40%3F%3FH%5D%3F%3FPw%40%3F%3F%60%40iB%3F%3FJc%40%3F%3FR%7B%40%3F%3FHY%3F%3FFY%3F%3FLc%40%3F%3FF%5B%3F%3FHc%40%3F%3F%5C%5CsA%3F%3FLs%40%3F%3FNi%40%3F%3F%40G%3F%3F%5C%5CwA%3F%3FR%7B%40%3F%3Fz%40%7BDTgA%7C%40yD%3F%3Fd%40sB%3F%3FT%7B%40%3F%3FXkA%3F%3FH%5DDU%3F%3FRy%40%3F%3FTgA%3F%3FFY%3F%3FPs%40%40E%3F%3FXgA%3F%3FZeA%3F%3Ff%40_B%3F%3FVu%40%3F%3F%5C%5CqA%3F%3FVu%40Le%40%3F%3F%3FC%3F%3FAC%40E%40CBCDC%3F%3F%40%3FB%3F%3F%3FPs%40b%40gB%3F%3F%7C%40gC%3F%3FJE%3F%3F%40EDKZ_A%3F%3F%5EmA%3F%3F%5Ey%40%3F%3FN_%40%3F%3FFK%3F%3FVk%40%3F%3FLY%3F%3F%5E%7B%40%3F%3FBO%3F%3F%7C%40oB%3F%3FpB%7BD%3F%3FvCmFpAeC%3F%3FT_%40%3F%3F%40%3F%40ABA%3F%3F%3F%3F%3F%3FpAeC%5Es%40LUHKJKLGNCL%3F%3F%3FZBbBVtHpA%3F%3F%40%3F%3F%3F%5C%5C%40%3F%3FfFbA%3F%3FhEz%40%3F%3FjB%5C%5CzAV%3F%3FlDf%40DBDB%3F%3FH%40%3F%3Fb%40qBd%40yB%3F%3FDU%40E%5C%5C_B%40I%3F%3F%40C%3F%3FZwA%3F%3FFY%3F%3Fh%40eCTiA%3F%3FJm%40%3F%3FDM%3F%3F%60%40wB%3F%3Fh%40eC%3F%3F%5C%5CaB%3F%3FNm%40Ns%40%40E%3F%3FPy%40%3F%3FHa%40%3F%3FPq%40%3F%3F%60%40qB%3F%3FBK%3F%3FLk%40%3F%3FLs%40BO%3F%3FLc%40F%5B%3F%3FPy%40%40G%3F%3FBM%40CHa%40%3F%3FLi%40%3F%3FDS%3F%3F%5EeB%3F%3F%40G%3F%3FH%5D%3F%3FTmA%3F%3FBI%3F%3FLk%40%3F%3FPu%40BQ%3F%3FNu%40DU%3F%3F%3F%3F%40C%3F%3FDU%5C%5C%7BA%3F%3FZcB%5EgB%3F%3FRw%40Ha%40BM%3F%3FXsA%3F%3FRaA%3F%3FVkA%3F%3FTcA%3F%3Fx%40kE%3F%3Ff%40cC%3F%3FDWDS%3F%3FFW%3F%3FDQ%3F%3FHe%40%3F%3FJc%40%5EiB%3F%3FXuA%3F%3FBK%3F%3FF%40D%40D%40%3F%3F~%40Vt%40R%3F%3FbAV%7CAb%40%3F%3FzA%60%40%3F%3FxA%60%40%3F%3Ft%40T%3F%3FhAZ%3F%3F~%40X%3F%3F%7CA%5E%3F%3FZaB%3F%3Fh%40%7DC%3F%3Ff%40H%3F%3FTkB%3F%3Fh%40sE%3F%3Ff%40cELeA%3F%3F%7BB%5BW%7CA
			name:              "Fastest path Karangasem -> Pasar Klewer Solo ",
			qOriginCoord:      da.NewCoordinate(-7.550422162941061, 110.78207364612717),
			qDestinationCoord: da.NewCoordinate(-7.575219943065335, 110.8267931836022),
			wantDirections: []string{
				"Head West",
				"Turn sharp right",
				"Turn right",
				"Turn left",
				"Turn right onto Jalan Adi Sucipto",
				"At Roundabout, take the exit point 2 clockwise onto Jalan Adi Sucipto",
				"Keep right continue on Fly Over Manahan",
				"Turn slight left onto Fly Over Manahan",
				"Merge onto onto Jalan Dokter Muwardi",
				"Turn left onto Jalan Brigadir Jenderal Slamet Riyadi",
				"Turn right onto Jalan Yos Sudarso",
				"Turn left",
				"Turn right onto Jalan Reksoniten",
				"Turn left",
				"Turn left",
				"you have arrived at your destination",
			},
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			orig := tc.qOriginCoord
			dest := tc.qDestinationCoord
			_, _, _, drivingDirections, _, _ := routingService.ShortestPath(context.Background(), orig.GetLat(), orig.GetLon(),
				dest.GetLat(), dest.GetLon())
			if len(drivingDirections) != len(tc.wantDirections) {
				t.Errorf("expected driving directions length: %v, got: %v", len(tc.wantDirections), len(drivingDirections))
			}
		})
	}
}
