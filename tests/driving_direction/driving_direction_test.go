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

	const filename = "solo"

	eng, logger, _ := tests.Setup(t, filename)
	re := eng.GetRoutingEngine()
	rtree := spatialindex.NewRtree()
	altSearch := routing.NewAlternativeRouteSearch(re)

	rtree.Build(re.GetGraph(), logger)

	routingService, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.05, true)
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

			// https://valhalla.github.io/demos/polyline/?unescape=true&polyline6=false#tram%40_cdcTD%40JDr%40N%3F%3FBOJg%40%3F%3FPy%40%3F%3FJw%40BW%3F%3FFa%40%3F%3FRiA%3F%3FrAP%3F%3FPB%3F%3Fn%40L%3F%3FrDl%40%3F%3FTB%3F%3F%7CAR%3F%3F%60%40H%3F%3FjAL%3F%3FXD%3F%3F%5C%5CFXDb%40D%3F%3FR%40%3F%3FbAD%3F%3FrBL%3F%3FXB%3F%3Fz%40LPDRDNDt%40%60%40%3F%3FHN%3F%3FJ%3FJCb%40KfA%5BPG%5EAZ%3Ft%40B%3F%3FJ%40%3F%3F%5C%5CDl%40F%3F%3F%40%3FH%40%3F%3FBMH_%40RcA%3F%3FVeA%3F%3Fd%40sB%3F%3FH%5DJe%40%3F%3FLg%40%3F%3F%3FEHa%40%3F%3FLi%40Je%40%3F%3FZoALm%40H_%40%3F%3FDMJa%40RcAFW%3F%3FF%5D%3F%3FTqA%3F%3FFY%3F%3FDU%3F%3FDS%3F%3FJg%40TaAR%7B%40Nm%40%3F%3FDO%3F%3FLq%40Lo%40Ps%40%3F%3FBOR_AJe%40%3F%3FXqAHY%3F%3FDS%3F%3FDM%3F%3FlAuE%3F%3Fb%40kBxDkRnBqK%5C%5CgB%3F%3FPaALk%40%3F%3F%3FA%3F%3F%3FC%3F%3FJi%40%3F%3F%5C%5C%7DA%3F%3FDS%3F%3FBK%3F%3FP_%40%3F%3FDU%5EeB%3F%3FVsA%3F%3FHi%40%3F%3FH_%40N_A%3F%3FJg%40%3F%3FBO%40K%3F%3FBMFW%3FC%3F%3FDQ%3F%3FLm%40%3F%3FTkA%3F%3FN%7D%40%3F%3FFU%3F%3Fb%40_C%3F%3FT_A%3F%3Fd%40aC%60%40mB%3F%3FFY%3F%3FFWLm%40Li%40%3F%3FXsA%3F%3Fb%40qBd%40yB%3F%3FDU%40E%3F%3F%5C%5C_B%40I%3F%3F%40C%3F%3FZwA%3F%3FFY%3F%3Fh%40eCTiA%3F%3FJm%40%3F%3FDM%3F%3F%60%40wB%3F%3Fh%40eC%3F%3F%5C%5CaB%3F%3FNm%40Ns%40%40E%3F%3FPy%40%3F%3FHa%40%3F%3FPq%40%3F%3F%60%40qB%3F%3FBK%3F%3FLk%40%3F%3FLs%40BO%3F%3FLc%40F%5B%3F%3FPy%40%40G%3F%3FBM%40CHa%40%3F%3FLi%40%3F%3FDS%3F%3F%5EeB%3F%3F%40G%3F%3FH%5D%3F%3FTmA%3F%3FBI%3F%3FLk%40%3F%3FPu%40BQ%3F%3FNu%40DU%3F%3F%3F%3F%40C%3F%3FDU%5C%5C%7BA%3F%3FZcB%5EgB%3F%3FRw%40Ha%40BM%3F%3FXsA%3F%3FRaA%3F%3FVkA%3F%3FTcA%3F%3Fx%40kE%3F%3Ff%40cC%3F%3FDWDS%3F%3FFW%3F%3FDQ%3F%3FHe%40%3F%3FJc%40%3F%3F%5EiB%3F%3FXuA%3F%3FBK%3F%3FF%40D%40D%40%3F%3F~%40V%3F%3Ft%40R%3F%3FbAV%3F%3F%7CAb%40%3F%3FzA%60%40%3F%3FxA%60%40%3F%3Ft%40T%3F%3FhAZ%3F%3F~%40X%3F%3F%7CA%5E%3F%3FZaB%3F%3Fh%40%7DC%3F%3Ff%40H%3F%3FTkB%3F%3Fh%40sE%3F%3Ff%40cELeA%3F%3F%40ERiB%40I%3F%3F%7DAW%3F%3FUE%3F%3FMp%40%3F%3FeAU
			name:              "Fastest path Karangasem -> Pasar Klewer Solo ",
			qOriginCoord:      da.NewCoordinate(-7.550373, 110.782061),
			qDestinationCoord: da.NewCoordinate(-7.575219943065335, 110.8267931836022),
			wantDirections: []string{
				"Head West",
				"Turn left",
				"Turn left",
				"Turn right",
				"Turn left",
				"Turn right",
				"Turn left",
				"Turn right onto Jalan Sawo Raya",
				"Turn left onto Jalan Jenderal Ahmad Yani",
				"Continue onto Jalan Brigadir Jenderal Slamet Riyadi",
				"Keep right",
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
			qDestinationCoord: da.NewCoordinate(-7.5544165068745315, 110.82690771093945),
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
		{

			//https://valhalla.github.io/demos/polyline/?unescape=true&polyline6=false#%60uam%40ybdcTA%40CN%3F%3FGd%40Qp%40%3F%3FdAL%3F%3FbAN%3F%3Fp%40J%3F%3FRw%40%3FA%40%3F%40%3F%3F%3F%40AB%40B%3FB%3F%3F%3FF%40d%40Pj%40Rx%40X%3F%3FHDH%40v%40H%3F%3Fn%40H%3F%3Fv%40HLB%3F%3F%60%40F%3F%3FnAT%3F%3FfBT%3F%3FRwA%3FA%3F%3FD_%40%3F%3Fp%40Ar%40GLA%3F%3FzDR%3F%3Fl%40iB%3F%3FVq%40%3F%3FJ%3FJCb%40KfA%5BPG%5EAZ%3Ft%40B%3F%3FJ%40%3F%3F%5C%5CDl%40F%3F%3F%40%3FH%40%3F%3FBMH_%40RcA%3F%3FVeA%3F%3Fd%40sB%3F%3FH%5DJe%40%3F%3FLg%40%3F%3F%3FEHa%40%3F%3FLi%40Je%40%3F%3FZoALm%40H_%40%3F%3FDMJa%40RcAFW%3F%3FF%5D%3F%3FTqA%3F%3FFY%3F%3FDU%3F%3FDS%3F%3FJg%40TaAR%7B%40Nm%40%3F%3FDO%3F%3FLq%40Lo%40Ps%40%3F%3FBOR_AJe%40%3F%3FXqAHY%3F%3FDS%3F%3FDM%3F%3FlAuE%3F%3Fb%40kBxDkRnBqK%5C%5CgB%3F%3FPaALk%40%3F%3F%3FA%3F%3F%3FC%3F%3FJi%40%3F%3F%5C%5C%7DA%3F%3FDS%3F%3FBK%3F%3FP_%40%3F%3FDU%5EeB%3F%3FVsA%3F%3FHi%40%3F%3FH_%40N_A%3F%3FJg%40%3F%3FBO%40K%3F%3FBMFW%3FC%3F%3FDQ%3F%3FLm%40%3F%3FTkA%3F%3FN%7D%40%3F%3FFU%3F%3Fb%40_C%3F%3FT_A%3F%3Fd%40aC%60%40mB%3F%3FFY%3F%3FFWLm%40Li%40%3F%3FXsA%3F%3Fb%40qBd%40yB%3F%3FDU%40E%3F%3F%5C%5C_B%40I%3F%3F%40C%3F%3FZwA%3F%3FFY%3F%3Fh%40eCTiA%3F%3FJm%40%3F%3FDM%3F%3F%60%40wB%3F%3Fh%40eC%3F%3F%5C%5CaB%3F%3FNm%40Ns%40%40E%3F%3FPy%40%3F%3FHa%40%3F%3FPq%40%3F%3F%60%40qB%3F%3FBK%3F%3FLk%40%3F%3FLs%40BO%3F%3FLc%40F%5B%3F%3FPy%40%40G%3F%3FBM%40CHa%40%3F%3FLi%40%3F%3FDS%3F%3F%5EeB%3F%3F%40G%3F%3FH%5D%3F%3FTmA%3F%3FBI%3F%3FLk%40%3F%3FPu%40BQ%3F%3FNu%40DU%3F%3F%3F%3F%40C%3F%3FDU%5C%5C%7BA%3F%3FZcB%5EgB%3F%3FRw%40Ha%40BM%3F%3FXsA%3F%3FRaA%3F%3FVkA%3F%3FTcA%3F%3Fx%40kE%3F%3Ff%40cC%3F%3FDWDS%3F%3FFW%3F%3FDQ%3F%3FHe%40%3F%3FJc%40%3F%3F%5EiB%3F%3FXuA%3F%3FBK%3F%3FF%40D%40D%40%3F%3F~%40V%3F%3Ft%40R%3F%3FbAV%3F%3F%7CAb%40%3F%3FzA%60%40%3F%3FxA%60%40%3F%3Ft%40T%3F%3FhAZ%3F%3F~%40X%3F%3F%7CA%5E%3F%3FrBf%40%3F%3F%7CD~%40%3F%3FpBf%40%3F%3FzCr%40%3F%3FdB%60%40%3F%3FPD%3F%3F%60AT%3F%3F~%40T%3F%3Fz%40R%3F%3FVF%3F%3FrCp%40%3F%3Fb%40J%3F%3FbDr%40%3F%3FvBd%40%3F%3Fn%40Ll%40L%3F%3FjEhA%3F%3FnBd%40%3F%3FpEdA%3F%3FtAZ%3F%3FTF%3F%3Fx%40R%3F%3FRH%3F%3F%40%3F%3F%3FTH%3F%3F%40%3F%3F%3F%7C%40ZbA%5C%5C%3F%3FbBj%40%3F%3F%60Ct%40LFND%3F%3FvDvA%3F%3FVL%3F%3Fl%40T%3F%3Fj%40Vj%40R%3F%3Fj%40Rd%40N%3F%3FJD%3F%3Fb%40HPBPBT%3F%3F%3Fp%40%40T%40R%40%60%40Dr%40H%3F%3Fj%40Jl%40J%3F%3FNF%3F%3FJB%3F%3FLF%3F%3FPDND%3F%3FVFb%40J%3F%3F%60B%60%40%60B%5E%3F%3F%7CBf%40%3F%3F%60Dr%40%3F%3F~%40R%3F%3F%60B%5C%5C%3F%3FLB%3F%3FjBb%40%3F%3Fh%40J%3F%3FtAX%3F%3F%60AT%3F%3FND%3F%3Fv%40P%3F%3FVD%3F%3FPD%3F%3F%5C%5CH%3F%3Ff%40J%3F%3FDU%3F%3FBS%3F%3FF%40%3F%3FRB%3F%3FXDD%3F%3F%3FJcA%3F%3FLeA%3F%3FUC%3F%3F%40O%3F%3FBM%3F%3FDQB%3F
			name:              "Fastest path Karangasem -> The Park Solo Baru ",
			qOriginCoord:      da.NewCoordinate(-7.550373, 110.782061),
			qDestinationCoord: da.NewCoordinate(-7.598614511487071, 110.81610040313558),
			wantDirections: []string{
				"Head West",
				"Turn left",
				"Turn left",
				"Turn right",
				"Turn left",
				"Turn right",
				"Turn left",
				"Turn right onto Jalan Sawo Raya",
				"Turn left onto Jalan Jenderal Ahmad Yani",
				"Continue onto Jalan Brigadir Jenderal Slamet Riyadi",
				"Keep right",
				"Turn right onto Jalan Yos Sudarso",
				"Turn left",
				"Turn right",
				"Turn left",
				"Turn left",
				"Turn right",
				"you have arrived at your destination",
			},
		},

		{

			// https://valhalla.github.io/demos/polyline/?unescape=true&polyline6=false#hldm%40q%60scTGBQCOO%3F%3FAFFV%40ZAX%3F%3FL%40%3F%3FR%3Fv%40C%5EA%3F%3FVQLWBG%3F%3FFU%3F%3FDS%3F%3FBSHw%40Fm%40%3F%3FRQ%3F%3FBg%40%3F_A%3Fg%40%3F%5B%3FK%3F%3F%3FI%40MDKDIDGFCFEHCHCNAP%40ND%3F%3F%7CAJ%3F%3Fh%40D%3F%3FNBf%40LPFRL%3F%3FRPDH%40D%40DABADEFKF%3F%3FjBD%3F%3FFAHAX%40b%40BXBVFRFJFFDDD%3F%3FFLBL%40LAR%3F%3FKh%40%3F%3F%3FD%3F%3FOjAGn%40%3F%3FG%60ACP%3FX%3F%3F%40P%3F%3FBZ%5C%5CxD%3F%3FBV%3F%3FFl%40%3F%3Ff%40vF%3F%3FLnA%3F%3FZxC%3F%3FRlB%3F%3FHv%40%3F%3FFp%40%3F%3FHt%40%3F%3FPtA%3F%3FBN%3F%3FFj%40%3F%3FHd%40%3F%3Fd%40dD%3F%3Ft%40hF%3F%3FDZ%3F%3FDX%3F%3FDV%3F%3FBT%3F%3FD%5E%3F%3FJt%40%3F%3Fn%40%60E%3F%3F%40JDZ%3F%3FRpA%3F%3FTtABN%3F%3F%5E%7CAHXBR%3F%3FA%5C%5CQ%60AET%3F%3FENQv%40%3F%3FOt%40Mb%40%3F%3FIZ%3F%3FWz%40%3F%3F%5BhA%3F%3FSx%40%3F%3FGN%3F%3Fa%40vA%3F%3Fc%40%60BGN%3F%3FEVCLS~%40%3F%3FO%60A%3F%3FKd%40%3F%3F_%40%60B%3F%3F%5BtA%3F%3FEL%3F%3FI%5C%5CCJ%3F%3FOn%40%3F%3FK%60%40%3F%3Fu%40zC%3F%3F%7B%40hD%3F%3FER%3F%3FU~%40%3F%3FIZ%3F%3Fo%40dC%3F%3FEN%3F%3Fo%40lC%3F%3FGT%3F%3Fc%40fB%3F%3FCNq%40nC%3F%3Fg%40tB%3F%3FERCJ%3F%3FIX%5BtAK%60%40%3F%3FCF%3F%3FAF%3F%3FOd%40mAvE%3F%3FYlA%3F%3FYnA%3F%3FYdA%3F%3FiBvG%3F%3FgArE%3F%3F%5BxAe%40fB%3F%3FK%5EENKf%40%3F%3FEREVE%60%40Gj%40%3F%3FCVEf%40%3F%3F%5B%7CC%3F%3Fa%40xC%3F%3F_%40zC%3F%3FQ%60AMn%40%3F%3FQ~%40A%40%3F%3FG%5C%5C%3F%3FAF%3F%3FAT%3F%3F%3FF%3F%3F%3F%40DHDFHD%3F%3FFDrAh%40%3F%3Fj%40RJFLHLJ%3F%3F%5B%5EMP%3F%3Fg%40j%40OP_%40%5C%5C%3F%3FSP%3F%3F_%40Z%3F%3Fc%40%5C%5CUR%3F%3Fa%40%5C%5Ca%40%60%40KJGL%3F%3FUb%40AB%3F%3FUh%40EL%3F%3Fo%40tA%3F%3FQd%40%5Bf%40%3F%3Fa%40p%40MV%3F%3FUZ%3F%3F%5Bj%40%3F%3FW%5E%3F%3FMPEF%3F%3FOT%3F%3FWb%40%3F%3FEF%3F%3Fk%40t%40%3F%3Fo%40~%40IJ%3F%3FKL%3F%3FW%5E%3F%3Fg%40r%40u%40%60AW%5EW%5E%3F%3F%5Db%40%3F%3FUZ%3F%3Fs%40%7C%40_%40h%40%3F%3Fg%40p%40%3F%3Fm%40v%40%3F%3FOR%3F%3FQP%3F%3FDbBAF%3F%3F%40v%40Dr%40%3F%3FJ%60AJjA%60%40pC%3F%3FRtA%3F%3Fj%40jF%3F%3FThBVnB%3F%3FP~A%3F%3FTdB%3F%3FFd%40BN%3F%3FDX%3F%3FBTFn%40VxB%3F%3FVzAp%40rF%3F%3FN%7C%40%3F%3Fn%40xD%3F%3F%60%40~C%3F%3FTzA%3F%3FDNZfBd%40bC%3F%3FP%7C%40%40F%3F%3F%40%40%3F%3FFZFb%40Pf%40HJPR%3F%3FHHHFJDNH%40%40%3F%3FD%40%40%40VF%3F%3FPD%5EHr%40P%60%40J%3F%3Fj%40Ld%40J%3F%3Fv%40RVD%3F%3FTD%3F%3FCb%40%3F%3FaAzE%3F%3FU~%40%3F%3FaBdJcEvScArE%3F%3FwApF%3F%3FEN%3F%3F%5D%60BKb%40Qz%40%3F%3FAD%3F%3FEPU~%40Id%40Ml%40%3F%3FEP%3F%3FEV%3F%3Fc%40hB%3F%3FOp%40Kh%40%3F%3FCHGTCPAF%3F%3FOt%40Ot%40GX%3F%3FWhAOj%40%3F%3FI%5EWpA%3F%3FMh%40Kd%40%3F%3F%5BpA%3F%3FWnA%3F%3FGZ%3F%3F%5DtA%3F%3FOr%40%3F%3FS%7C%40%3F%3FMp%40%3F%3FKb%40%3F%3FCLCN%3F%3FI%5C%5C%3F%3FQx%40%3F%3FOp%40%3F%3FCNI%5C%5CET%3F%3FET%3F%3FKl%40MfAEj%40AV%3F%3F%3Fd%40AT%3FnA%3F%3F%3FL%3F%3F%3FX%3F%3F%3F%40%3F%3F%40N%3F%3F%40JLdC%3F%3FPdCLrA%3F%3FVtC%3F%3FB%5C%5C%3F%3FX%7CBVzB%3F%3FPnA%3F%3FN%60A%3F%3FVbB%3F%3FZdB%3F%3FVzA%3F%3FL%60A%3F%3FJl%40%3F%3F%40L%3F%3FHj%40%3F%3FHl%40%3F%3FHn%40%3F%3FRdB%3F%3F%40J%3F%3FDRTpAFb%40h%40%7CCN%7C%40DV%3F%3FHf%40PdAJl%40%3F%3FBJ%3F%3FF%5EPz%40%3F%3FJn%40Lr%40%3F%3FFV%3F%3F%40N%3F%3FNp%40ThA%3F%3Fh%40lD%3F%3FX~A%3F%3FDVDT%3F%3FP%7C%40Ff%40%40L%3F%3FBPDl%40BlA%3F%5E%3FZ%3F%3FAx%40Ap%40%3F%3F%3FP%3F%3FA%5C%5C%3F%3FMbE%3FD%3F%3FGrB%3F%3FAN%3F%3FCnAKlBGlACT%3FB%3F%3FIh%40%3F%3FGd%40%3F%3FAHOz%40_%40fB%3F%3FER%3F%3Fc%40%60B_AlDEJ%3F%3FIV%5DnAUp%40%3F%3FKTKX%3F%3F%5Dx%40%3F%3FYh%40%3F%3FWj%40IP%3F%3Fu%40zA%3F%3FwAzC%3F%3FMV%3F%3FOZ%3F%3FKR%3F%3Fi%40hAi%40nA%3F%3Fq%40~AMX%3F%3F%7B%40xB%3F%3FKT%3F%3FuAdDEL%3F%3FKVaAhC%3F%3FyB%60G%3F%3FqA%7CC%3F%3Fk%40vAKV%3F%3FqAbD%3F%3Fe%40hA%3F%3F%7BArD%3F%3FYv%40%3F%3Fc%40jA%3F%3F%7D%40xB%3F%3Fs%40jB%3F%3F%5Bx%40CFIP%3F%3FKTM%5C%5C%3F%3FITsBjF%3F%3F%7D%40xB%3F%3FSd%40%3F%3FUp%40GPOh%40Ql%40%3F%3FMl%40Kb%40Mr%40%5BjB%3F%3FW%7CAQrA%3F%3FCP%3F%3FANAD%40%60%40AR%3F%3FB%40%3F%3F%40F%3F%3F%40%40
			name:              "Fastest path Solo Safari -> Tugu Kartasura ",
			qOriginCoord:      da.NewCoordinate(-7.564346891692738, 110.8585580026218),
			qDestinationCoord: da.NewCoordinate(-7.550419577060595, 110.73671629762856),
			wantDirections: []string{
				"Head North",
				"Turn left",
				"Turn left onto Jalan Ki Hadjar Dewantara",
				"Turn slight left",
				"Turn slight right",
				"Turn left",
				"Turn slight right",
				"Turn slight right",
				"Turn left onto Jalan Insinyur Haji Juanda Kartawijaya",
				"Continue onto Jalan Sutan Sjahrir",
				"Continue onto Jalan Sultan Syahrir",
				"Turn right onto Jalan R.M. Said",
				"Turn left onto Jalan Hasanuddin",
				"Turn right",
				"Keep right",
				"Keep right",
				"Continue onto Jalan Jenderal Ahmad Yani",
				"Enter the roundabout",
				"you have arrived at your destination",
			},
		},
		// { // aneh di laptop pass, di test.yml github actions gak
		// 	// https://valhalla.github.io/demos/polyline/?unescape=true&polyline6=false#ho~l%40uhjcT%60AQ%3F%3FMw%40%3F%3FtGeA%3F%3FIg%40%3F%3FhFeA%3F%3FBIBQ%3FS%40%5B%3F%3FNB%3F%3FpAJ%3F%3Fx%40B%3F%3FNgANoA%3F%3FLuB%3F%3FFs%40%3F%3FByA%3F%3F%40uCBw%40%3F%3Ft%40P%3F%3FrA%5E%3F%3FrBb%40%3F%3Fh%40JRBH%40F%3F%3F%3FDCBIBGBMTmA%3F%3Fp%40mD%3F%3Ff%40kC%3F%3FJm%40%3F%3FPBN%3F%3F%3FNCLELG%3F%3FLQJSBU%3F%3FrAL%3F%3FtBT%3F%3F~CP%3F%3FzAH%3F%3FhBD%3F%3F%60B%40%3F%3FdBGVAb%40G%3F%3Fl%40I%3F%3Fj%40MBA%3F%3FdAY%3F%3F%7C%40O%3F%3Fx%40M%3F%3F%7C%40K%3F%3FjAS%3F%3F~AU%3F%3FzAU%3F%3FNC%3F%3F%40AnAU%3F%3FTE%60%40G%40A%3F%3FDA%40%3F%3F%3FlAW%3F%3FfCk%40%3F%3FLC%3F%3FPC%3F%3Fp%40M%3F%3FNINEFA%3F%3FPCLAFAJ%3FJ%40%3F%3F%5EF%3F%3Ff%40NFB%3F%3FdA%5C%5CPF%3F%3FDB%3F%3F%5C%5CL%3F%3FB%40B%40B%40JDF%40%3F%3FNMLIFG%3F%3FRQp%40u%40%3F%3FFIJSP%5D%3F%3F%5C%5C_A%3F%3FPq%40ViA%3F%3FP%40%3F%3FD%3FDAB%3FJC%3F%3FNC%3F%3FrBg%40%3F%3FJI%3F%3FREx%40U%3F%3Fz%40QTG%3F%3F%60AU%3F%3FHAPE%3F%3FbASHA%40Ap%40Q%3F%3FPCBADA%7C%40O%3F%3Fj%40Gb%40C%60%40CXA%3F%3FT%3F%3F%3F%5C%5C%3Fl%40%3FR%40%3F%3FD%40%3F%3FLB%3F%3F%40%3FtBB%3F%3Fr%40%40f%40%3F%3F%3FlA%40%3F%3FnEF%3F%3FpA%40%3F%3FfED%3F%3FdA%3F%3F%3FV%40RBVF%3F%3F%7C%40R%3F%3Fn%40R%3F%3FJD%3F%3Fp%40R%3F%3F%5C%5CL%3F%3F%5E%7BC%3F%3F%60%40yC%3F%3FZ%7DC%3F%3F%7CAB%3F%3F%7C%40AR%40JAHE%3F%3FZU%3F%3FBC%3F%3FTK%3F%3FLI%3F%3F%60%40Qj%40UTK%3F%3FGh%40El%40%3F%3FAH%3F%3FE%60AD%7C%40%3F%3Ft%40Dr%40B%3F%3FBS%3F%3FvAN%3F%3FdBN%3F%3FJi%40%3F%3FrB%5EB%40%3F%3FLB%3F%3Fd%40H%3F%3FHg%40N%7D%40%3F%3Fz%40Rr%40Nf%40H%3F%3Fc%40~A%3F%3FtBd%40%3F%3F%60%40H%3F%3Fr%40P%3F%3FbAR%3F%3FlBf%40%3F%3FNA%3F%3FND%3F%3FF%40D%40D%40%3F%3F~%40V%3F%3Ft%40R%3F%3FbAV%3F%3F%7CAb%40%3F%3FzA%60%40%3F%3FxA%60%40%3F%3Ft%40T%3F%3FhAZ%3F%3F~%40X%3F%3F%7CA%5E%3F%3FrBf%40%3F%3F%7CD~%40%3F%3FpBf%40%3F%3FzCr%40%3F%3FdB%60%40%3F%3FPD%3F%3F%60AT%3F%3F~%40T%3F%3Fz%40R%3F%3FVF%3F%3FrCp%40%3F%3Fb%40J%3F%3FbDr%40%3F%3FvBd%40%3F%3Fn%40Ll%40L%3F%3FjEhA%3F%3FnBd%40%3F%3FpEdA%3F%3FtAZ%3F%3FTF%3F%3Fx%40R%3F%3FRH%3F%3F%40%3F%3F%3FTH%3F%3F%40%3F%3F%3F%7C%40ZbA%5C%5C%3F%3FbBj%40%3F%3F%60Ct%40LFND%3F%3FvDvA%3F%3FVL%3F%3Fl%40T%3F%3Fj%40Vj%40R%3F%3Fj%40Rd%40N%3F%3FJD%3F%3Fb%40HPBPBT%3F%3F%3Fp%40%40T%40R%40%60%40Dr%40H%3F%3Fj%40Jl%40J%3F%3FNF%3F%3FJB%3F%3FLF%3F%3FPDND%3F%3FVFb%40J%3F%3F%60B%60%40%60B%5E%3F%3F%7CBf%40%3F%3F%60Dr%40%3F%3F~%40R%3F%3F%60B%5C%5C%3F%3FLB%3F%3FjBb%40%3F%3Fh%40J%3F%3FtAX%3F%3F%60AT%3F%3FND%3F%3Fv%40P%3F%3FVD%3F%3FPD%3F%3F%5C%5CH%3F%3Ff%40J%3F%3FH%40%3F%3Ft%40PvCl%40%3F%3FTD%3F%3FGZ%3F%3FENAD%3FDCFWnA%3F%3FCJ%3F%3FJDd%40Jj%40N%3F%3Fw%40nDo%40%7CB
		// 	name:              "Fastest path Kadipiro -> Solobaru ",
		// 	qOriginCoord:      da.NewCoordinate(-7.534139343718934, 110.8136607220971),
		// 	qDestinationCoord: da.NewCoordinate(-7.5990772297996045, 110.8121090871445),
		// 	wantDirections: []string{
		// 		"Head South toward Jalan Tulang Bawang Utara 1",
		// 		"Turn left",
		// 		"Turn right onto Jalan Tulang Bawang 4",
		// 		"Turn left onto Jalan Tulang Bawang Selatan",
		// 		"Turn right",
		// 		"Turn left",
		// 		"Turn right",
		// 		"Turn left onto Jalan Pamugaran Utama",
		// 		"Turn sharp right onto Jalan Pamugaran Utama",
		// 		"Turn left onto Jalan Pamugaran Utama",
		// 		"Turn right onto Simpang Joglo",
		// 		"Turn right onto Jalan Kapten Pierre Tendean",
		// 		"Continue onto Jembatan Kalianyar",
		// 		"Turn left onto Jalan Jenderal Achmad Yani",
		// 		"Turn right onto Jalan Letnan Jenderal S. Parman",
		// 		"Turn left onto Jalan Sutan Sjahrir",
		// 		"Turn right",
		// 		"Turn sharp right onto Jalan Saharjo",
		// 		"Turn left",
		// 		"Turn left onto Jalan Saparua 3",
		// 		"Turn right",
		// 		"Turn left onto Jalan MGR. Sugiyopranoto",
		// 		"Turn right",
		// 		"Turn left onto Jalan Ambon",
		// 		"Turn right",
		// 		"Turn right onto Jalan Ronggowarsito",
		// 		"Turn left onto Jalan Kyai Haji Ahmad Dahlan",
		// 		"Continue onto Jalan Yos Sudarso",
		// 		"Turn right",
		// 		"Turn left",
		// 		"Turn right onto Jalan Paris 1",
		// 		"you have arrived at your destination",
		// 	},
		// },
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			orig := tc.qOriginCoord
			dest := tc.qDestinationCoord // todo: aneh setelah tambahin multiple via-way turn restrictions jadi gak kedetect turn instruction buat roundabout manahan (DONE)
			_, _, _, drivingDirections, _, _ := routingService.ShortestPath(context.Background(), orig.GetLat(), orig.GetLon(),
				dest.GetLat(), dest.GetLon(), false, 0)
			if len(drivingDirections) != len(tc.wantDirections) {
				t.Errorf("expected driving directions length: %v, got: %v", len(tc.wantDirections), len(drivingDirections))
			}

			for i := 0; i < len(drivingDirections); i++ {
				if drivingDirections[i].GetInstruction() != tc.wantDirections[i] {
					t.Errorf("expected %v-th driving direction instruction: %v, got: %v", i, tc.wantDirections[i], drivingDirections[i].GetInstruction())
				}
			}
		})
	}
}
