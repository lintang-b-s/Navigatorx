package controllers

import (
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
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
	Instruction string                   `json:"instruction"`
	Point       datastructure.Coordinate `json:"turn_point"`
	StreetName  string                   `json:"street_name"`
	TravelTime  float64                  `json:"travel_time"`
	Distance    float64                  `json:"distance"`
	EdgeIds     []datastructure.Index    `json:"edge_ids"`
	Polyline    string                   `json:"polyline"`
	TurnBearing float64                  `json:"turn_bearing"`
	TurnType    string                   `json:"turn_type"`
}

func NewDrivingDirection(d datastructure.DrivingDirection) drivingDirection {
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

func NewDrivingDirections(d []datastructure.DrivingDirection) []drivingDirection {
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

func NewAlternativeRoutesResponse(alts []*routing.AlternativeRoute) alternativeRoutesResponse {
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
	Lon   float64   `json:"lon"`
	Lat   float64   `json:"lat"`
	Time  time.Time `json:"time"`
	Speed float64   `json:"speed"` // 0 if time step k=0
}

func newGPS(lat, lon float64, time time.Time, speed float64) *gps {
	return &gps{lat, lon, time, speed}
}

func (g *gps) ToDataGPS() *datastructure.GPSPoint {
	return datastructure.NewGPSPoint(g.Lat, g.Lon, g.Time, g.Speed)
}

type Candidate struct {
	EdgeId datastructure.Index `json:"edge_id"`
	Weight float64             `json:"weight"`
	Length float64             `json:"length"`
}

func NewCandidate(eId datastructure.Index, weight float64, length float64) Candidate {
	return Candidate{EdgeId: eId, Weight: weight, Length: length}
}

func ToOnlineCandidates(cands []*Candidate) []*online.Candidate {
	oCands := make([]*online.Candidate, len(cands))
	for i, cand := range cands {
		oCands[i] = online.NewCandidate(cand.EdgeId, cand.Weight, cand.Length)
	}
	return oCands
}

type mapMatchRequest struct {
	Gps        gps          `json:"gps_point"`
	K          int          `json:"k"`
	Candidates []*Candidate `json:"candidates"`
	SpeedMeanK float64      `json:"speed_mean_k"`
	SpeedStdK  float64      `json:"speed_std_k"`
}

type MatchedGPSPoint struct {
	GpsPoint     *gps                     `json:"gps_point"`
	EdgeId       datastructure.Index      `json:"edge_id"`
	MatchedCoord datastructure.Coordinate `json:"matched_coord"`
}

func NewMatchedGPSPoint(gpsPoint *gps, edgeId datastructure.Index, matchedCoord datastructure.Coordinate) *MatchedGPSPoint {
	return &MatchedGPSPoint{
		GpsPoint:     gpsPoint,
		EdgeId:       edgeId,
		MatchedCoord: matchedCoord,
	}
}

type mapmatchingResponse struct {
	MatchedGpsPoint *MatchedGPSPoint `json:"matched_gps_point"`
	Candidates      []Candidate      `json:"candidates"`
	SpeedMeanK      float64          `json:"speed_mean_k"`
	SpeedStdK       float64          `json:"speed_std_k"`
}

func NewMapmatchingResponse(matchedPoint *datastructure.MatchedGPSPoint, candidates []*online.Candidate, speedMeanK float64, speedStdK float64) *mapmatchingResponse {
	mgps := matchedPoint.GetGpsPoint()
	matchedGpsPoint := NewMatchedGPSPoint(newGPS(mgps.Lat(), mgps.Lon(), mgps.Time(), mgps.Speed()), matchedPoint.GetEdgeId(), matchedPoint.GetMatchedCoord())
	cands := make([]Candidate, len(candidates))
	for i, cand := range candidates {
		cands[i] = NewCandidate(cand.EdgeId(), cand.Weight(), cand.Length())
	}
	return &mapmatchingResponse{
		MatchedGpsPoint: matchedGpsPoint,
		Candidates:      cands,
		SpeedMeanK:      speedMeanK,
		SpeedStdK:       speedStdK,
	}
}
