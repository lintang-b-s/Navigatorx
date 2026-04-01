package controllers

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
)

type shortestPathRequest struct {
	OriginLat      float64 `json:"origin_lat" validate:"required,min=-90,max=90"`
	OriginLon      float64 `json:"origin_lon" validate:"required,min=-180,max=180"`
	DestinationLat float64 `json:"destination_lat" validate:"required,min=-90,max=90"`
	DestinationLon float64 `json:"destination_lon" validate:"required,min=-180,max=180"`
}

type alternativeRoutesRequest struct {
	OriginLat      float64 `json:"origin_lat" validate:"required,min=-90,max=90"`
	OriginLon      float64 `json:"origin_lon" validate:"required,min=-180,max=180"`
	DestinationLat float64 `json:"destination_lat" validate:"required,min=-90,max=90"`
	DestinationLon float64 `json:"destination_lon" validate:"required,min=-180,max=180"`
	K              int64   `json:"k" validate:"required,min=1"`
}

type shortestPathResponse struct {
	TravelTime        float64            `json:"travel_time"`
	Path              string             `json:"path"`
	Dist              float64            `json:"distance"`
	DrivingDirections []drivingDirection `json:"driving_directions"`
}

type alternativeRoutesResponse struct {
	Routes []shortestPathResponse `json:"alternative_routes"`
}

type drivingDirection struct {
	Instruction string        `json:"instruction"`
	Point       da.Coordinate `json:"turn_point"`
	StreetName  string        `json:"street_name"`
	TravelTime  float64       `json:"travel_time"`
	Distance    float64       `json:"distance"`
	EdgeIds     []da.Index    `json:"edge_ids"`
	Polyline    string        `json:"polyline"`
	TurnBearing float64       `json:"turn_bearing"`
	TurnType    string        `json:"turn_type"`
}

func NewDrivingDirection(d da.DrivingDirection) drivingDirection {
	return drivingDirection{
		Instruction: d.GetInstruction(),
		Point:       d.GetPoint(),
		StreetName:  d.GetStreetName(),
		TravelTime:  d.GetTravelTime(),
		Distance:    d.GetDistance(),
		EdgeIds:     d.GetEdgesIds(),
		Polyline:    d.GetPolyline(),
		TurnBearing: d.GetTurnBearing(),
		TurnType:    d.GetTurnType(),
	}
}

func NewDrivingDirections(d []da.DrivingDirection) []drivingDirection {
	drivingDirections := make([]drivingDirection, len(d))
	for i, dir := range d {
		drivingDirections[i] = NewDrivingDirection(dir)
	}
	return drivingDirections
}

func NewShortestPathResponse(travelTime, dist float64, path string, drivingDirections []drivingDirection) shortestPathResponse {
	return shortestPathResponse{
		TravelTime:        travelTime,
		Path:              path,
		Dist:              dist,
		DrivingDirections: drivingDirections,
	}
}

func NewAlternativeRoutesResponse(alts []routing.AlternativeRoute) alternativeRoutesResponse {
	altRes := alternativeRoutesResponse{}
	for _, alt := range alts {
		altRes.Routes = append(altRes.Routes, NewShortestPathResponse(
			alt.GetDrivingTravelTime(),
			alt.GetDist(), alt.GetPolylinePath(), NewDrivingDirections(alt.GetDrivingDirections()),
		))
	}
	return altRes
}

type errorResponse struct {
	Error struct {
		Code    string `json:"code"`
		Message string `json:"message"`
	} `json:"error"`
}

type gps struct {
	Lon           float64   `json:"lon" validate:"required,min=-180,max=180"`
	Lat           float64   `json:"lat" validate:"required,min=-90,max=90"`
	Time          time.Time `json:"time"`
	Speed         float64   `json:"speed" validate:"min=0"` // 0 if time step k=0
	DeltaTime     float64   `json:"delta_time" validate:"min=0"`
	DeadReckoning bool      `json:"dead_reckoning"`
}

func newGPS(lat, lon float64, time time.Time, speed, deltaTime float64) *gps {
	return &gps{Lat: lat, Lon: lon, Time: time, Speed: speed, DeltaTime: deltaTime}
}

func (g *gps) ToDataGPS() *da.GPSPoint {
	return da.NewGPSPoint(g.Lat, g.Lon, g.Time, g.Speed,
		g.DeltaTime, g.DeadReckoning)
}

type Candidate struct {
	EdgeId da.Index `json:"edge_id" validate:"min=0"`
	Weight float64  `json:"weight"`
	Length float64  `json:"length"`
}

func NewCandidate(eId da.Index, weight float64, length float64) Candidate {
	return Candidate{EdgeId: eId, Weight: weight, Length: length}
}

func ToOnlineCandidates(cands []*Candidate) []*ma.Candidate {
	oCands := make([]*ma.Candidate, len(cands))
	for i, cand := range cands {

		oCands[i] = ma.NewCandidate(cand.EdgeId, cand.Weight, cand.Length)
	}
	return oCands
}

type mapMatchRequest struct {
	Gps         gps          `json:"gps_point"`
	K           int          `json:"k" validate:"min=1"`
	Candidates  []*Candidate `json:"candidates"`
	SpeedMeanK  float64      `json:"speed_mean_k" validate:"min=0"`
	SpeedStdK   float64      `json:"speed_std_k" validate:"min=0"`
	LastBearing float64      `json:"last_bearing" validate:"min=0"`
}

func (mr *mapMatchRequest) GetBearing() float64 {
	return mr.LastBearing
}

type MatchedGPSPoint struct {
	GpsPoint       *gps          `json:"gps_point"`
	EdgeId         da.Index      `json:"edge_id"`
	MatchedCoord   da.Coordinate `json:"matched_coord"`
	PredictedCoord da.Coordinate `json:"predicted_gps_coord"`
	Bearing        float64       `json:"edge_initial_bearing"`
}

func NewMatchedGPSPoint(gpsPoint *gps, edgeId da.Index, matchedCoord, predictedCoord da.Coordinate, initialBearing float64) *MatchedGPSPoint {
	return &MatchedGPSPoint{
		GpsPoint:       gpsPoint,
		EdgeId:         edgeId,
		MatchedCoord:   matchedCoord,
		Bearing:        initialBearing,
		PredictedCoord: predictedCoord,
	}
}

type mapmatchingResponse struct {
	MatchedGpsPoint *MatchedGPSPoint `json:"matched_gps_point"`
	Candidates      []Candidate      `json:"candidates"`
	SpeedMeanK      float64          `json:"speed_mean_k"`
	SpeedStdK       float64          `json:"speed_std_k"`
	Bearing         float64          `json:"edge_initial_bearing"`
}

func NewMapmatchingResponse(matchedPoint *da.MatchedGPSPoint, candidates []*ma.Candidate, speedMeanK float64, speedStdK, bearing float64) *mapmatchingResponse {
	mgps := matchedPoint.GetGpsPoint()
	matchedGpsPoint := NewMatchedGPSPoint(newGPS(mgps.Lat(), mgps.Lon(), mgps.Time(), mgps.Speed(),
		mgps.DeltaTime()), matchedPoint.GetEdgeId(), matchedPoint.GetMatchedCoord(), matchedPoint.GetPredictedGpsCoord(), matchedPoint.GetBearing())
	cands := make([]Candidate, len(candidates))
	for i, cand := range candidates {
		cands[i] = NewCandidate(cand.EdgeId(), cand.Weight(), cand.Length())
	}
	return &mapmatchingResponse{
		MatchedGpsPoint: matchedGpsPoint,
		Candidates:      cands,
		SpeedMeanK:      speedMeanK,
		SpeedStdK:       speedStdK,
		Bearing:         bearing,
	}
}

type offlineMatchRequest struct {
	GpsTraj []gps `json:"gps_points"`
}

func (of *offlineMatchRequest) ToDataGpsTraj() []*da.GPSPoint {
	gpsTraj := make([]*da.GPSPoint, len(of.GpsTraj))
	for i := 0; i < len(of.GpsTraj); i++ {
		gpsTraj[i] = of.GpsTraj[i].ToDataGPS()
	}

	return gpsTraj
}

type offlineMapmatchingResponse struct {
	MatchedGpsPoint      []*MatchedGPSPoint `json:"matched_gps_points"`
	MatchedRoutePolyline string             `json:"matched_route_polyline"`
}

func NewOfflineMapmatchingResponse(matchedPoints []*da.MatchedGPSPoint, polyline string) *offlineMapmatchingResponse {
	matchedGpsResp := make([]*MatchedGPSPoint, len(matchedPoints))
	for i := 0; i < len(matchedPoints); i++ {
		matchedPoint := matchedPoints[i]
		mgps := matchedPoint.GetGpsPoint()
		matchedGpsPoint := NewMatchedGPSPoint(newGPS(mgps.Lat(), mgps.Lon(), mgps.Time(), mgps.Speed(),
			mgps.DeltaTime()), matchedPoint.GetEdgeId(), matchedPoint.GetMatchedCoord(), matchedPoint.GetPredictedGpsCoord(), matchedPoint.GetBearing())
		matchedGpsResp = append(matchedGpsResp, matchedGpsPoint)
	}
	return &offlineMapmatchingResponse{
		MatchedGpsPoint:      matchedGpsResp,
		MatchedRoutePolyline: polyline,
	}
}
