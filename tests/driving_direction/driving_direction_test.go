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
			// https://www.google.com/maps/dir/-7.550422162941061,+110.78207364612717/-7.575219943065335,+110.8267931836022/@-7.5685064,110.813721,16.35z/am=t/data=!4m21!4m20!1m13!2m2!1d110.7820736!2d-7.5504222!3m4!1m2!1d110.8066484!2d-7.5629727!3s0x2e7a1429e8e625bd:0x3f548995f93a09f3!3m4!1m2!1d110.8241434!2d-7.5755624!3s0x2e7a166434dc6a4d:0xa9ffed9ee89235cd!1m3!2m2!1d110.8267932!2d-7.5752199!3e0!5i1?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D
			// https://valhalla.github.io/demos/polyline/?unescape=true&polyline6=false#%60uam%40_cdcTAFCN%3F%3Fs%40OKESEeAa%40%5BM%3F%3FMZ%3F%3F%7BDoA%3F%3FKC%3F%3FME%3F%3FFW%3F%3FR_A%3F%3FJa%40%3F%3FNu%40%3F%3FNs%40%3F%3FJm%40%3F%3FLk%40%3F%3FLm%40%3F%3FDM%3F%3FJe%40%3F%3FJa%40%3F%3FFY%3F%3FDS%3F%3FLm%40%3F%3FVoA%3F%3FXkA%3F%3F%5C%5C_B%3F%3FJg%40%3F%3FJa%40%3F%3FPs%40%3F%3FLo%40%3F%3FH%5D%3F%3FJe%40%3F%3FNq%40Lk%40%3F%3FH%5D%3F%3FPw%40%3F%3F%60%40iB%3F%3FJc%40%3F%3FR%7B%40%3F%3FHY%3F%3FFY%3F%3FLc%40%3F%3FF%5B%3F%3FHc%40%3F%3F%5C%5CsA%3F%3FLs%40%3F%3FNi%40%3F%3F%40G%3F%3F%5C%5CwA%3F%3FR%7B%40%3F%3Fz%40%7BDTgA%7C%40yD%3F%3Fd%40sB%3F%3FT%7B%40%3F%3FXkA%3F%3FH%5DDU%3F%3FRy%40%3F%3FTgA%3F%3FFY%3F%3FPs%40%40E%3F%3FXgA%3F%3FZeA%3F%3Ff%40_B%3F%3FVu%40%3F%3F%5C%5CqA%3F%3FVu%40Le%40%3F%3F%3FC%3F%3FAC%40E%40CBCDC%3F%3F%40%3FB%3F%3F%3FPs%40b%40gB%3F%3F%7C%40gC%3F%3FJE%3F%3F%40EDKZ_A%3F%3F%5EmA%3F%3F%5Ey%40%3F%3FN_%40%3F%3FFK%3F%3FVk%40%3F%3FLY%3F%3F%5E%7B%40%3F%3FBO%3F%3F%7C%40oB%3F%3FpB%7BD%3F%3FvCmFpAeC%3F%3FT_%40%3F%3F%40%3F%40ABA%3F%3F%3F%3F%3F%3FpAeC%5Es%40LUHKJKLGNCL%3F%3F%3FZBbBVtHpA%3F%3F%40%3F%3F%3F%5C%5C%40%3F%3FfFbA%3F%3FhEz%40%3F%3FjB%5C%5CzAV%3F%3FlDf%40DBDB%3F%3FH%40%3F%3Fb%40qBd%40yB%3F%3FDU%40E%5C%5C_B%40I%3F%3F%40C%3F%3FZwA%3F%3FFY%3F%3Fh%40eCTiA%3F%3FJm%40%3F%3FDM%3F%3F%60%40wB%3F%3Fh%40eC%3F%3F%5C%5CaB%3F%3FNm%40Ns%40%40E%3F%3FPy%40%3F%3FHa%40%3F%3FPq%40%3F%3F%60%40qB%3F%3FBK%3F%3FLk%40%3F%3FLs%40BO%3F%3FLc%40F%5B%3F%3FPy%40%40G%3F%3FBM%40CHa%40%3F%3FLi%40%3F%3FDS%3F%3F%5EeB%3F%3F%40G%3F%3FH%5D%3F%3FTmA%3F%3FBI%3F%3FLk%40%3F%3FPu%40BQ%3F%3FNu%40DU%3F%3F%3F%3F%40C%3F%3FDU%5C%5C%7BA%3F%3FZcB%5EgB%3F%3FRw%40Ha%40BM%3F%3FXsA%3F%3FRaA%3F%3FVkA%3F%3FTcA%3F%3Fx%40kE%3F%3Ff%40cC%3F%3FDWDS%3F%3FFW%3F%3FDQ%3F%3FHe%40%3F%3FJc%40%5EiB%3F%3FXuA%3F%3FBK%3F%3FF%40D%40D%40%3F%3F~%40Vt%40R%3F%3FbAV%7CAb%40%3F%3FzA%60%40%3F%3FxA%60%40%3F%3Ft%40T%3F%3FhAZ%3F%3F~%40X%3F%3F%7CA%5E%3F%3FZaB%3F%3Fh%40%7DC%3F%3Ff%40H%3F%3FTkB%3F%3Fh%40sE%3F%3Ff%40cELeA%3F%3F%7BB%5BW%7CA
			name:              "Fastest path Karangasem -> Pasar Klewer Solo ",
			qOriginCoord:      da.NewCoordinate(-7.550373, 110.782061),
			qDestinationCoord: da.NewCoordinate(-7.575219943065335, 110.8267931836022),
			wantDirections: []string{
				"Head West",
				"Turn right",
				"Turn left",
				"Turn right",
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
				"Turn left onto Jalan Dokter Rajiman Widiodiningrat",
				"you have arrived at your destination",
			},
		},
		{
			// https://www.google.com/maps/dir/Sans+Guest+House+2,+Jl.+Mulwo,+Karangasem,+Kec.+Laweyan,+Kota+Surakarta,+Jawa+Tengah+57145/-7.554152,110.8269207/@-7.5552169,110.8072014,14.76z/am=t/data=!4m13!4m12!1m5!1m1!1s0x2e7a14403c5830dd:0x5a2e99d453ee8b46!2m2!1d110.7819826!2d-7.5504398!1m0!3e0!6m3!1i0!2i0!3i6?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D
			// https://valhalla.github.io/demos/polyline/?unescape=true&polyline6=false#%60uam%40_cdcTAFCN%3F%3Fs%40OKESEeAa%40%5BM%3F%3FMZ%3F%3F%7BDoA%3F%3FKC%3F%3FME%3F%3FFW%3F%3FR_A%3F%3FJa%40%3F%3FNu%40%3F%3FNs%40%3F%3FJm%40%3F%3FLk%40%3F%3FLm%40%3F%3FDM%3F%3FJe%40%3F%3FJa%40%3F%3FFY%3F%3FDS%3F%3FLm%40%3F%3FVoA%3F%3FXkA%3F%3F%5C%5C_B%3F%3FJg%40%3F%3FJa%40%3F%3FPs%40%3F%3FLo%40%3F%3FH%5D%3F%3FJe%40%3F%3FNq%40Lk%40%3F%3FH%5D%3F%3FPw%40%3F%3F%60%40iB%3F%3FJc%40%3F%3FR%7B%40%3F%3FHY%3F%3FFY%3F%3FLc%40%3F%3FF%5B%3F%3FHc%40%3F%3F%5C%5CsA%3F%3FLs%40%3F%3FNi%40%3F%3F%40G%3F%3F%5C%5CwA%3F%3FR%7B%40%3F%3Fz%40%7BDTgA%7C%40yD%3F%3Fd%40sB%3F%3FT%7B%40%3F%3FXkA%3F%3FH%5DDU%3F%3FRy%40%3F%3FTgA%3F%3FFY%3F%3FPs%40%40E%3F%3FXgA%3F%3FZeA%3F%3Ff%40_B%3F%3FVu%40%3F%3F%5C%5CqA%3F%3FVu%40Le%40%3F%3F%3FC%3F%3FOW%3F%3FOWi%40o%40%3F%3Fy%40aA%3F%3Fy%40aASU%3F%3FW_%40%3F%3FAC%3F%3FKWI%5BAG%3F%3FEU%3F%3FQeA%3F%3Fg%40oC%3F%3FIg%40%3F%3FMq%40%3F%3FOy%40%3F%3FI%5Di%40%7BC%3F%3FaAcFSuA%3F%3FC%5DC_AB_ARcB%3F%3F%5EoCPgBHcAA%7D%40%3F%3FCUMiAYiAOe%40%3F%3FY%7D%40Y%7D%40%3F%3FCM%3F%3FcBmFEOCE%3F%3Fc%40wA%3F%3Fu%40aC%3F%3Fa%40iA%3F%3FEQGSCWCy%40%3Fi%40D%7B%40%3F%3FNkB%40G%3F%3FFmA%3F%3FF%7D%40%3F%3F%40%5D%3F%3FJaBLkA%3F%3FF%5Dl%40iC%3F%3FLg%40h%40aCBI%3F%3F%5EcBBM%3F%3FLk%40%60AkDHYd%40%7BAt%40gBDMLU%3F%3F%3F%3F%3F%3FHMDEDI%3F%3FNMLIFG%3F%3FRQp%40u%40%3F%3FFIJSP%5D%3F%3F%5C%5C_A%3F%3FPq%40ViA%3F%3FJa%40%3F%3F%40GJO%3F%3FFYBM%3F%3FBMHm%40%3F%3F%40QDi%40Dk%40%3F%3FJyA%3F%3FDo%40%3F%3F%3FO%3F%3FB%5BDg%40%3F%3FDs%40%3F%3F%40I%3F%3FHa%40%3F%3Ff%40%7DAViA
			name:              "Fastest path Karangasem -> Masjid Syekh Zayed ",
			qOriginCoord:      da.NewCoordinate(-7.550373, 110.782061),
			qDestinationCoord: da.NewCoordinate(-7.5543778590367765, 110.82681779917101),
			wantDirections: []string{
				"Head West",
				"Turn right",
				"Turn left",
				"Turn right",
				"Turn right onto Jalan Adi Sucipto",
				"At Roundabout, take the exit point 1 clockwise onto Jalan Jenderal Achmad Yani",
				"you have arrived at your destination",
			},
		},
		{
			// https://www.google.com/maps/dir/Sans+Guest+House+2,+Jl.+Mulwo,+Karangasem,+Kec.+Laweyan,+Kota+Surakarta,+Jawa+Tengah+57145/Pusat+Bersejarah+Yahudi+Solo,+FR43%2B5X9,+Jl.+Kutai+Utara,+Sumber,+Kec.+Banjarsari,+Kota+Surakarta,+Jawa+Tengah+57138/@-7.5479247,110.7887684,16z/am=t/data=!3m1!4b1!4m15!4m14!1m5!1m1!1s0x2e7a14403c5830dd:0x5a2e99d453ee8b46!2m2!1d110.7819826!2d-7.5504398!1m5!1m1!1s0x2e7a150057096b2b:0xdd08bc5e5019dae!2m2!1d110.8049463!2d-7.5444278!3e0!5i1?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D
			// https://valhalla.github.io/demos/polyline/?unescape=true&polyline6=false#%60uam%40_cdcTAFCN%3F%3Fs%40OKESEeAa%40%5BM%3F%3FMZ%3F%3F%7BDoA%3F%3FKC%3F%3FME%3F%3FFW%3F%3FR_A%3F%3FJa%40%3F%3FNu%40%3F%3FNs%40%3F%3FJm%40%3F%3FLk%40%3F%3FLm%40%3F%3FDM%3F%3FJe%40%3F%3FJa%40%3F%3FFY%3F%3FDS%3F%3FLm%40%3F%3FVoA%3F%3FXkA%3F%3F%5C%5C_B%3F%3FJg%40%3F%3FJa%40%3F%3FPs%40%3F%3FLo%40%3F%3FH%5D%3F%3FJe%40%3F%3FNq%40Lk%40%3F%3FH%5D%3F%3FPw%40%3F%3F%60%40iB%3F%3FJc%40%3F%3FR%7B%40%3F%3FHY%3F%3FFY%3F%3FLc%40%3F%3FF%5B%3F%3FHc%40%3F%3F%5C%5CsA%3F%3FLs%40%3F%3FNi%40%3F%3F%40G%3F%3F%5C%5CwA%3F%3FR%7B%40%3F%3Fz%40%7BDTgA%7C%40yD%3F%3Fd%40sB%3F%3FT%7B%40%3F%3FXkA%3F%3FH%5DDU%3F%3FRy%40%3F%3FTgA%3F%3FFY%3F%3FPs%40%40E%3F%3FXgA%3F%3FZeA%3F%3Ff%40_B%3F%3FVu%40%3F%3F%5C%5CqA%3F%3FVu%40Le%40%3F%3F%3FC%3F%3FOW%3F%3FOWi%40o%40%3F%3Fy%40aA%3F%3Fy%40aASU%3F%3FW_%40%3F%3FAC%3F%3FKWI%5BAG%3F%3Fm%40c%40%3F%3FsAgAKU%3F%3FKG%3F%3FEC%3F%3FSM%3F%3FCC%3F%3Fy%40q%40q%40i%40%3F%3FcBoA%3F%3FAAsA_Ai%40a%40UMWMeAg%40%3F%3FGA%3F%3FsBu%40%3F%3Fi%40SwAi%40%3F%3F_A_%40%3F%3Fw%40_%40%3F%3Fy%40a%40%3F%3FmAi%40%3F%3Fa%40O%3F%3F%7BBcAw%40_%40%3F%3F_%40~%40Ob%40%5DjA%3F%3FEJ%3F%3FQGcDcAFO
			name:              "Fastest path Karangasem -> Tembok Ratapan Solo ",
			qOriginCoord:      da.NewCoordinate(-7.550373, 110.782061),
			qDestinationCoord: da.NewCoordinate(-7.544482918882859, 110.80498053177885),
			wantDirections: []string{
				"Head West",
				"Turn right",
				"Turn left",
				"Turn right",
				"Turn right onto Jalan Adi Sucipto",
				"At Roundabout, take the exit point 1 clockwise onto Jalan Jenderal Achmad Yani",
				"Turn slight left onto Jalan Letnan Jenderal Suprapto",
				"Turn left onto Jalan Kutai Raya",
				"Turn right onto Jalan Kutai 8",
				"Turn right onto Jalan Kutai Utara",
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

			for i := 0; i < len(drivingDirections); i++ {
				if drivingDirections[i].GetInstruction() != tc.wantDirections[i] {
					t.Errorf("expected ith driving direction instruction: %v, got: %v", tc.wantDirections[i], drivingDirections[i].GetInstruction())
				}
			}
		})
	}
}
