package controllers

import (
	"encoding/xml"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
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

type boundingBoxResponse struct {
	MinLat float64 `json:"min_lat"`
	MinLon float64 `json:"min_lon"`
	MaxLat float64 `json:"max_lat"`
	MaxLon float64 `json:"max_lon"`
}

type annotation struct {
	Duration       []float64  `json:"duration"`
	Distance       []float64  `json:"distance"`
	Geometry       string     `json:"geometry"`
	EdgeIds        []da.Index `json:"edge_ids"`
	EdgeGeomOffset []da.Index `json:"edge_geometry_offset"`
}

func NewAnnotation(duration, distance []float64, geometry string, edgeGeomOffset, edgeIds []da.Index) annotation {
	return annotation{Duration: duration, Distance: distance, Geometry: geometry, EdgeGeomOffset: edgeGeomOffset, EdgeIds: edgeIds}
}

type drivingDirection struct {
	Instruction         string             `json:"instruction"`
	Annotation          annotation         `json:"annotation,omitzero"`
	Point               da.FloatCoordinate `json:"turn_point"`
	StreetName          string             `json:"street_name"`
	TravelTime          float64            `json:"travel_time"`
	Distance            float64            `json:"distance"`
	TurnBearing         float64            `json:"turn_bearing"`
	TurnType            string             `json:"turn_type"`
	SuggestAlternatives bool               `json:"suggest_alternatives"`
}

func NewAnnotationDTO(ann da.Annotation, edgeIds []da.Index) annotation {
	return NewAnnotation(
		ann.GetDuration(),
		ann.GetDistance(),
		da.GooglePoylineFromCoords(ann.GetGeometry()),
		ann.GetEdgeGeomOffset(),
		edgeIds,
	)
}

func NewDrivingDirection(d da.DrivingDirection, useAnnotation bool) drivingDirection {

	ann := annotation{}
	if useAnnotation {
		ann = NewAnnotationDTO(d.GetAnnotation(), d.GetEdgesIds())
	}
	return drivingDirection{
		Instruction:         d.GetInstruction(),
		Point:               d.GetPoint().ToFloatCoordinate(),
		StreetName:          d.GetStreetName(),
		TravelTime:          util.SecondsToMinutes(d.GetTravelTime()),
		Distance:            d.GetDistance(),
		TurnBearing:         d.GetTurnBearing(),
		TurnType:            d.GetTurnTableId(),
		SuggestAlternatives: d.GetSuggestAlternatives(),
		Annotation:          ann,
	}
}

func NewDrivingDirections(d []da.DrivingDirection, useAnnotation bool) []drivingDirection {
	drivingDirections := make([]drivingDirection, len(d))
	for i, dir := range d {
		drivingDirections[i] = NewDrivingDirection(dir, useAnnotation)
	}
	return drivingDirections
}

// NewShortestPathResponse. create new sp response
// travelTime in seconds we need to convert it in minutes
func NewShortestPathResponse(travelTime, dist float64, path string, drivingDirections []drivingDirection) shortestPathResponse {
	return shortestPathResponse{
		TravelTime:        util.SecondsToMinutes(travelTime), // in
		Path:              path,
		Dist:              dist,
		DrivingDirections: drivingDirections,
	}
}

func NewAlternativeRoutesResponse(alts []routing.AlternativeRoute, useAnnotation bool) alternativeRoutesResponse {
	altRes := alternativeRoutesResponse{Routes: make([]shortestPathResponse, 0)}
	for _, alt := range alts {
		altRes.Routes = append(altRes.Routes, NewShortestPathResponse(
			alt.GetDrivingTravelTime(),
			alt.GetDist(), alt.GetPolylinePath(), NewDrivingDirections(alt.GetDrivingDirections(), useAnnotation),
		))
	}
	return altRes
}

func NewBoundingBox(bb da.BoundingBox) boundingBoxResponse {
	return boundingBoxResponse{MinLat: bb.GetMinLat(), MinLon: bb.GetMinLon(), MaxLat: bb.GetMaxLat(), MaxLon: bb.GetMaxLon()}
}

type gps struct {
	Lon       float64   `json:"lon" validate:"required,min=-180,max=180"`
	Lat       float64   `json:"lat" validate:"required,min=-90,max=90"`
	Time      time.Time `json:"time"`
	Speed     float64   `json:"speed" validate:"min=0"`      // 0 if time step k=0  m/s
	DeltaTime float64   `json:"delta_time" validate:"min=0"` // in seconds

}

func newGPS(lat, lon float64, time time.Time, speed, deltaTime float64) *gps {
	return &gps{Lat: lat, Lon: lon, Time: time, Speed: speed, DeltaTime: deltaTime}
}

func (g *gps) ToDataGPS() *da.GPSPoint {
	return da.NewGPSPoint(g.Lat, g.Lon, g.Time, g.Speed,
		g.DeltaTime)
}

type Candidate struct {
	EdgeId da.Index `json:"edge_id" validate:"min=0"`
	Weight float64  `json:"weight" validate:"min=0"`
	Length float64  `json:"length" validate:"min=0"`
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
	SpeedMeanK  float64      `json:"speed_mean_k" validate:"min=0"` // m/s
	SpeedStdK   float64      `json:"speed_std_k" validate:"min=0"`  // m/s
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

type startClientSideRealtimeMapMatchingResponse struct {
	NumberOfVertices int `json:"number_of_vertices"`
}

func NewStartClientSideRealtimeMapMatchingResponse(n int) *startClientSideRealtimeMapMatchingResponse {
	return &startClientSideRealtimeMapMatchingResponse{NumberOfVertices: n}
}

type GPX struct {
	XMLName xml.Name `xml:"gpx"`
	Trk     Trk      `xml:"trk"`
}

type Trk struct {
	XMLName xml.Name `xml:"trk"`
	TrkSeg  TrkSeg   `xml:"trkseg"`
}

type TrkSeg struct {
	XMLName xml.Name `xml:"trkseg"`
	TrkPts  []TrkPt  `xml:"trkpt"`
}

type TrkPt struct {
	XMLName xml.Name `xml:"trkpt"`
	Lat     float64  `xml:"lat,attr"`
	Lon     float64  `xml:"lon,attr"`
	Time    string   `xml:"time,omitempty"`
}

type offlineMapMatchingResponse struct {
	MatchedPoints []*MatchedGPSPoint `json:"matched_points"`
	RoutePath     []da.Coordinate    `json:"route_path"`
}

func NewOfflineMapMatchingResponse(matchedPoints []*da.MatchedGPSPoint, routePath []da.Coordinate) *offlineMapMatchingResponse {
	points := make([]*MatchedGPSPoint, len(matchedPoints))
	for i, mp := range matchedPoints {
		mgps := mp.GetGpsPoint()
		points[i] = NewMatchedGPSPoint(
			&gps{
				Lat:  mgps.Lat(),
				Lon:  mgps.Lon(),
				Time: mgps.Time(),
			},
			mp.GetEdgeId(),
			mp.GetMatchedCoord(),
			mp.GetPredictedGpsCoord(),
			mp.GetBearing(),
		)
	}
	return &offlineMapMatchingResponse{
		MatchedPoints: points,
		RoutePath:     routePath,
	}
}
