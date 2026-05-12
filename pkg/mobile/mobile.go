// Package mobile provides online map matching mobile app library.
package mobile

import (
	"bytes"
	"encoding/json"
	"sync"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"go.uber.org/zap"
	_ "golang.org/x/mobile/bind"
	_ "golang.org/x/mod/modfile"
)

/*
ini terinspirasi dari: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e?gi=0f02a4844208
golang library untuk client-side (mobile app) realtime map matching
run: gomobile bind -v -target=android -androidapi 21 -o online_mapmatcher.aar ./pkg/mobile
copy online_mapmatcher.aar ke react native project directory "<rn_working_dir>/modules/map-matcher/android/libs/"

inti dari file ini dan ref artikel diatas:
1. client side mobile app (android & ios) secara berkala merequest MapElements (roadNetworkGraph) ke backend setiap kali user pindah cell/tile geohash .
2. MapMatchingGraph (go localization library) bakal ngebuat/rebuild MapMatchGraph dari beberapa MapElements yang dekat dengan lokasi user (dari geohash Cell user dan its geohash neighbor cells).
3. Melakukan real time map matching (https://eng.lyft.com/a-new-real-time-map-matching-algorithm-at-lyft-da593ab7b006) atau mapmatch_mht.go pakai MobileMapMatcher (go localization library) di dalam mobile app navigatorx pakai MapMatchGraph yang dibuild dari MapElements tadi .

note:
1. edgeIds candidates and matchedRoadSegment yang direturn dari MobileMapMatcher.onlineMapMatch() adalah roadNetworkGraph EdgeId (graph.go)
2. arguments edgeIds candidates dari OnlineMapMatchMHTClient.onlineMapMatch() adalah MapMatchGraph (mapmatch_graph.go) EdgeId
kenapa harus gini?
karena kita secara berkala rebuild MapMatchGraph yang mana storage nya adalah Compressed Sparse Row (CSR) (see  C++ Boost libary: https://www.boost.org/doc/libs/latest/libs/graph/doc/compressed_sparse_row.html)
edgeIds MapMatchGraph yang terbaru yang baru aja di Rebuild() akan beda dari edgeIds MapMatchGraph sebelum Rebuild()
tapi RoadNetworkGraph EdgeId akan selalu sama karena RoadNetworkGraph dibuild sama routing engine saat Customizable Route Planning (CRP) Preprocessing phase (preprocessor/main.go).
dan MapElements yang dikirim routing engine ke client (mobile app) berisi RoadNetworkGraph Edges...

contoh kalau kita lihat di gambar (diambil dari ref artikel diatas): https://miro.medium.com/v2/resize:fit:720/format:webp/1*QoOcp6rHVwMFxPpy2tcvLw.png atau https://private-user-images.githubusercontent.com/109817421/587064233-6ad92c92-e8f1-4ab6-b821-74ff3c46f03b.webp?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3Nzg0MTAzNjgsIm5iZiI6MTc3ODQxMDA2OCwicGF0aCI6Ii8xMDk4MTc0MjEvNTg3MDY0MjMzLTZhZDkyYzkyLWU4ZjEtNGFiNi1iODIxLTc0ZmYzYzQ2ZjAzYi53ZWJwP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDUxMCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA1MTBUMTA0NzQ4WiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9NTYxNmRhYTBkNTc5NjM5MjI3NGJlMjQ4YzdjYWY3MThkOTA1OWJmNGQ3MDFlZDg4ZTQ1MzBmZTFlYzdkOTY2ZiZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QmcmVzcG9uc2UtY29udGVudC10eXBlPWltYWdlJTJGd2VicCJ9.U0qbR39Qi-bqFjspPwfiZAnsVnOCpw-SO9gwnuvUe2w
misal user/driver pindah dari cell s0 ke s1 kanan..
mobile app fetch MapElements (untuk cells s1,s2) ke routing engine backend (disini kita pakai endpoint /tile/:userGeohash), dan rebuild MapMatchGraph di mobile app nya pakai golang library ini dengan memanggil method RebuildGraph()
road segment candidates (yang jadi argument dari onlineMapMatch()/ road segments yang mungkin ditempati user) bisa aja sama kaya sebelum RebuildGraph, karena di inisialisasi & setiap rebuildGraph kita fetch MapElemnents geohash cell dari user & 8 neighbor geohashes (atau 12 s2 cells dari kasus diatas)
kita butuh RoadNetworkGraph EdgeIds di .onlineMapMatch karena edge transition count matrix (om.N) column dan row nya adalah RoadNetworkGraph EdgeId

setelah RebuildGraph() (setelah pindah ke cell s1), MapMatchGraph edgeIds berubah (see MapMatchingGraph.Rebuild()), dan kalau candidate edgeIds nya di client (mobile app) disimpan sebagai MapMatchGraph edgeId yang lama (sebelum Rebuild()), OnlineMapMatch() bakal fail karena MapMatchGraph EdgeIds yang baru/setelah rebuild beda sama yang lama (atau bahkan bisa panic kalau old MapMatchGraph edgeId > jumlah edges di new MapMatchGraph)
tapi karena di mobile app road segment candidates edgeIds nya adalah RoadNetworkGraph EdgeIds, dan saat user pindah ke sel s1 kita rebuild graph pakai cell s1 dan 11 cell disekitarnya dan kita bikin hashmap yang memetakan RoadNetworkGraph edgeId -> MapMatchEdgeId, .onlineMapMatch() gak akan fail .kalau candidate EdgeIds nya masih di cell s0 pun juga gak akan fail karena di new MapMatchGraph hasil Rebuild masih include RoadNetworkGraph edgeIds ini


*/

// MobileMapMatcher provides a high-level API for mobile real time map matching.
type MobileMapMatcher struct {
	om     *online.OnlineMapMatchMHTClient
	graph  *da.MapMatchingGraph
	rt     *spatialindex.Rtree
	matrix *da.SparseMatrix[int]
	logger *zap.Logger
	mut    sync.RWMutex
}

func NewMobileMapMatcher() *MobileMapMatcher {
	config := zap.NewDevelopmentConfig()
	logger, _ := config.Build()
	return &MobileMapMatcher{
		rt:     spatialindex.NewRtree(),
		logger: logger,
		mut:    sync.RWMutex{},
	}
}

func (m *MobileMapMatcher) InitializeGraph(numVertices int) {
	m.mut.Lock()
	defer m.mut.Unlock()
	m.graph = da.InitializeMapMatchingGraph(numVertices)
}

func (m *MobileMapMatcher) RebuildGraph(tileBytes []byte) error {
	m.mut.Lock()
	defer m.mut.Unlock()

	if err := m.graph.RebuildMapMatchGraphFromReader(bytes.NewReader(tileBytes)); err != nil {
		return err
	}
	// Rebuild the R-tree with the new tile data
	m.rt.Reset()

	m.rt.BuildMapMatch(m.graph, m.logger)
	m.om = online.NewOnlineMapMatchMHTClient(
		m.graph, m.rt,
		8.33333,   // initialSpeedMean (m/s )
		8.3333,    // initialSpeedStd
		0.0001,    // posteriorThreshold
		5.0,       // gpsStd (meters)
		0.0000001, // lp
		0.04,      // lc (km ~40m search radius)
		3.0,       // accelerationStd
		m.matrix,
	)

	return nil
}

func (m *MobileMapMatcher) SetMatrix(matrixBytes []byte) error {
	matrix, err := da.ReadSparseMatrixFromReader[int](bytes.NewReader(matrixBytes), int(0),
		func(a, b int) bool { return a == b })
	if err != nil {
		return err
	}
	m.matrix = matrix
	return nil
}

// CandidateDTO is a JSON-serializable version of mapmatcher.Candidate
type CandidateDTO struct {
	RoadNetworkEdgeId uint32  `json:"roadnetwork_edge_id"`
	Weight            float64 `json:"weight"`
	Length            float64 `json:"length"`
}

// MatchedGPSPointDTO is a JSON-serializable version of datastructure.MatchedGPSPoint
type MatchedGPSPointDTO struct {
	GPSPoint           GPSPointDTO `json:"gps_point"`
	RoadNetworkEdgeId  uint32      `json:"roadnetwork_edge_id"`
	MatchedCoord       CoordDTO    `json:"matched_coord"`
	PredictedGPSCoord  CoordDTO    `json:"predicted_gps_coord"`
	EdgeInitialBearing float64     `json:"edge_initial_bearing"`
}

type GPSPointDTO struct {
	Lat           float64 `json:"lat"`
	Lon           float64 `json:"lon"`
	TimeUnix      int64   `json:"time_unix"`
	Speed         float64 `json:"speed"`
	DeltaTime     float64 `json:"delta_time"`
	DeadReckoning bool    `json:"dead_reckoning"`
}

type CoordDTO struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

// MatchResult holds the output of the map matching process to be returned as JSON.
type MatchResult struct {
	MatchedGPSPoint *MatchedGPSPointDTO `json:"matched_gps_point"`
	NewCandidates   []CandidateDTO      `json:"new_candidates"`
	NewSpeedMean    float64             `json:"new_speed_mean"`
	NewSpeedStd     float64             `json:"new_speed_std"`
}

// toCandidateDTO converts a mapmatcher.Candidate to a JSON-serializable CandidateDTO
func (m *MobileMapMatcher) toCandidateDTO(c *ma.Candidate) CandidateDTO {
	// c.EdgeId() masih MapMatchGraph edgeId
	roadnetworkEdgeId := m.graph.GetRoadnetworkEdgeId(c.EdgeId())
	return CandidateDTO{
		RoadNetworkEdgeId: uint32(roadnetworkEdgeId),
		Weight:            c.Weight(),
		Length:            c.Length(),
	}
}

// toMatchedGPSPointDTO converts a datastructure.MatchedGPSPoint to a JSON-serializable MatchedGPSPointDTO
func (m *MobileMapMatcher) toMatchedGPSPointDTO(mgps *da.MatchedGPSPoint) *MatchedGPSPointDTO {
	// mgps.GetEdgeId() masih MapMatchGraph edgeId
	matchedMapMatchEdgeId := mgps.GetEdgeId()
	matchedRoadNetworkEdgeId := da.INVALID_EDGE_ID

	if matchedMapMatchEdgeId != da.INVALID_EDGE_ID {
		matchedRoadNetworkEdgeId = m.graph.GetRoadnetworkEdgeId(matchedMapMatchEdgeId)
	}

	gps := mgps.GetGpsPoint()

	gpsDTO := GPSPointDTO{
		Lat:           gps.Lat(),
		Lon:           gps.Lon(),
		TimeUnix:      gps.Time().Unix(),
		Speed:         gps.Speed(),
		DeltaTime:     gps.DeltaTime(),
		DeadReckoning: gps.GetDeadReckoning(),
	}

	matchedCoord := mgps.GetMatchedCoord()
	predCoord := mgps.GetPredictedGpsCoord()

	return &MatchedGPSPointDTO{
		GPSPoint:          gpsDTO,
		RoadNetworkEdgeId: uint32(matchedRoadNetworkEdgeId),
		MatchedCoord: CoordDTO{
			Lat: matchedCoord.GetLat(),
			Lon: matchedCoord.GetLon(),
		},
		PredictedGPSCoord: CoordDTO{
			Lat: predCoord.GetLat(),
			Lon: predCoord.GetLon(),
		},
		EdgeInitialBearing: mgps.GetBearing(),
	}
}

func (m *MobileMapMatcher) Match(
	lat float64,
	lon float64,
	timeUnix int64,
	gpsSpeed float64,
	deltaTime float64,
	deadReckoning bool,
	k int,
	candidatesJSON string,
	speedMeanK float64,
	speedStdK float64,
	lastBearing float64,
) (string, error) {
	m.mut.RLock()
	defer m.mut.RUnlock()

	gps := da.NewGPSPoint(
		lat,
		lon,
		time.Unix(timeUnix, 0),
		gpsSpeed,
		deltaTime,
		deadReckoning,
	)

	// Unmarshal into DTOs
	var inputCandidates []CandidateDTO
	err := json.Unmarshal([]byte(candidatesJSON), &inputCandidates)
	if err != nil {
		return "", err
	}

	// Convert DTOs to Candidate objects
	candidates := make([]*ma.Candidate, 0, len(inputCandidates))

	for _, dto := range inputCandidates {
		roadnetworkEdgeId := da.Index(dto.RoadNetworkEdgeId)
		mapmatchEdgeId, ok := m.graph.GetMapMatchEdgeId(roadnetworkEdgeId)
		if !ok {
			// edgeId gak included di current graph tile
			continue
		}

		candidates = append(candidates, ma.NewCandidate(mapmatchEdgeId, dto.Weight, dto.Length))
	}

	matched, newCandidates, newSpeedmeanK, newSpeedStdK := m.om.OnlineMapMatch(gps, k, candidates, speedMeanK, speedStdK, lastBearing)

	dtoMatched := m.toMatchedGPSPointDTO(matched)
	dtoCandidates := make([]CandidateDTO, len(newCandidates))
	for i, c := range newCandidates {
		dtoCandidates[i] = m.toCandidateDTO(c)
	}

	result := MatchResult{
		MatchedGPSPoint: dtoMatched,
		NewCandidates:   dtoCandidates,
		NewSpeedMean:    newSpeedmeanK,
		NewSpeedStd:     newSpeedStdK,
	}
	res, err := json.Marshal(result)
	if err != nil {
		return "", err
	}

	return string(res), nil
}
