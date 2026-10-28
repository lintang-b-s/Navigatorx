package controllers

import (
	"encoding/xml"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type shortestPathRequest struct {
	OriginLat      float64 `json:"origin_lat" validate:"required,min=-90,max=90"`
	OriginLon      float64 `json:"origin_lon" validate:"required,min=-180,max=180"`
	DestinationLat float64 `json:"destination_lat" validate:"required,min=-90,max=90"`
	DestinationLon float64 `json:"destination_lon" validate:"required,min=-180,max=180"`
}

type shortestPathResponse struct {
	Cost              float64            `json:"travel_time"`
	Path              string             `json:"path"`
	Dist              float64            `json:"distance"`
	DrivingDirections []DrivingDirection `json:"driving_directions"`
}

type shortestPathEnvelope struct {
	Data shortestPathResponse `json:"data"`
}

// AlternativeRoutesResponse contains alternative route results for an API response.
type AlternativeRoutesResponse struct {
	Routes []shortestPathResponse `json:"routes"`
}

type alternativeRoutesEnvelope struct {
	Data AlternativeRoutesResponse `json:"data"`
}

type boundingBoxEnvelope struct {
	Data boundingBoxResponse `json:"data"`
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
	SegmentIds     []da.Index `json:"segment_ids"`
	EdgeGeomOffset []da.Index `json:"segment_geometry_offset"`
}

// newAnnotation creates and returns annotation.
func newAnnotation(duration, distance []float64, geometry string, edgeGeomOffset, segmentIds []da.Index) annotation {
	return annotation{Duration: duration, Distance: distance, Geometry: geometry, EdgeGeomOffset: edgeGeomOffset, SegmentIds: segmentIds}
}

// DrivingDirection describes one instruction along a route.
type DrivingDirection struct {
	Instruction         string             `json:"instruction"`
	Annotation          annotation         `json:"annotation,omitzero"`
	Point               da.FloatCoordinate `json:"turn_point"`
	StreetName          string             `json:"street_name"`
	Cost                float64            `json:"travel_time"`
	Distance            float64            `json:"distance"`
	TurnBearing         float64            `json:"turn_bearing"`
	TurnType            string             `json:"turn_type"`
	SuggestAlternatives bool               `json:"suggest_alternatives"`
}

// newAnnotationDTO creates and returns annotation dto.
func newAnnotationDTO(ann da.Annotation, segmentIds []da.Index) annotation {
	return newAnnotation(
		ann.GetDuration(),
		ann.GetDistance(),
		da.GooglePoylineFromCoords(ann.GetGeometry()),
		ann.GetEdgeGeomOffset(),
		segmentIds,
	)
}

// NewDrivingDirection creates and returns driving direction.
func NewDrivingDirection(d da.DrivingDirection, useAnnotation bool) DrivingDirection {

	ann := annotation{}
	if useAnnotation {
		ann = newAnnotationDTO(d.GetAnnotation(), d.GetSegmentsIds())
	}
	return DrivingDirection{
		Instruction:         d.GetInstruction(),
		Point:               d.GetPoint().ToFloatCoordinate(),
		StreetName:          d.GetStreetName(),
		Cost:                util.SecondsToMinutes(d.GetCost()),
		Distance:            d.GetDistance(),
		TurnBearing:         d.GetTurnBearing(),
		TurnType:            d.GetTurnTableId(),
		SuggestAlternatives: d.GetSuggestAlternatives(),
		Annotation:          ann,
	}
}

// NewDrivingDirections converts driving directions to the response DTO.
func NewDrivingDirections(d []da.DrivingDirection, useAnnotation bool) []DrivingDirection {
	drivingDirections := make([]DrivingDirection, len(d))
	if useAnnotation {
		for i, dir := range d {
			drivingDirections[i] = NewDrivingDirection(dir, true)
		}
		return drivingDirections
	}
	for i, dir := range d {
		drivingDirections[i] = NewDrivingDirection(dir, false)
	}
	return drivingDirections
}

// newShortestPathResponse creates and returns shortest path response.
// travelTime in seconds we need to convert it in minutes
func newShortestPathResponse(travelTime, dist float64, path string, drivingDirections []DrivingDirection) shortestPathResponse {
	return shortestPathResponse{
		Cost:              util.SecondsToMinutes(travelTime), // in
		Path:              path,
		Dist:              dist,
		DrivingDirections: drivingDirections,
	}
}

// NewAlternativeRoutesResponse creates and returns alternative routes response.
func NewAlternativeRoutesResponse(routeCosts []float64, routeDists []float64, routePolylines []string, routeDrivingDirections [][]da.DrivingDirection, useAnnotation bool) AlternativeRoutesResponse {
	altRes := AlternativeRoutesResponse{Routes: make([]shortestPathResponse, len(routeCosts))}
	for i := range routeCosts {
		altRes.Routes[i] = newShortestPathResponse(
			routeCosts[i],
			routeDists[i], routePolylines[i], NewDrivingDirections(routeDrivingDirections[i], useAnnotation),
		)
	}
	return altRes
}

// newBoundingBox creates and returns bounding box.
func newBoundingBox(bb da.BoundingBox) boundingBoxResponse {
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

// Candidate represents road segment candidate.
type Candidate struct {
	SegmentId da.Index `json:"segment_id" validate:"min=0"`
	Weight    float64  `json:"weight" validate:"min=0"`
	Length    float64  `json:"length" validate:"min=0"`
}

// NewCandidate creates and returns road segment candidate.
func NewCandidate(eId da.Index, weight float64, length float64) Candidate {
	return Candidate{SegmentId: eId, Weight: weight, Length: length}
}

// ToOnlineCandidates handles to online road segment candidates.
func ToOnlineCandidates(cands []*Candidate) []*ma.Candidate {
	oCands := make([]*ma.Candidate, len(cands))
	for i, cand := range cands {

		oCands[i] = ma.NewCandidate(cand.SegmentId, cand.Weight, cand.Length)
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

// MatchedGPSPoint represents matched gps point.
type MatchedGPSPoint struct {
	GpsPoint       *gps               `json:"gps_point"`
	SegmentId      da.Index           `json:"segment_id"`
	MatchedCoord   da.FloatCoordinate `json:"matched_coord"`
	PredictedCoord da.FloatCoordinate `json:"predicted_gps_coord"`
	Bearing        float64            `json:"segment_initial_bearing"`
}

// NewMatchedGPSPoint creates and returns matched gps point.
func NewMatchedGPSPoint(gpsPoint *gps, segmentId da.Index, matchedCoord, predictedCoord da.Coordinate, initialBearing float64) *MatchedGPSPoint {
	return &MatchedGPSPoint{
		GpsPoint:       gpsPoint,
		SegmentId:      segmentId,
		MatchedCoord:   matchedCoord.ToFloatCoordinate(),
		Bearing:        initialBearing,
		PredictedCoord: predictedCoord.ToFloatCoordinate(),
	}
}

// MapmatchingResponse contains the result of an online map-matching request.
type MapmatchingResponse struct {
	MatchedGpsPoint *MatchedGPSPoint `json:"matched_gps_point"`
	Candidates      []Candidate      `json:"candidates"`
	SpeedMeanK      float64          `json:"speed_mean_k"`
	SpeedStdK       float64          `json:"speed_std_k"`
	Bearing         float64          `json:"edge_initial_bearing"`
}

// NewMapmatchingResponse creates and returns mapmatching response.
func NewMapmatchingResponse(matchedPoint *da.MatchedGPSPoint, candidates []*ma.Candidate, speedMeanK float64, speedStdK, bearing float64) *MapmatchingResponse {
	mgps := matchedPoint.GetGpsPoint()
	matchedGpsPoint := NewMatchedGPSPoint(newGPS(mgps.Lat(), mgps.Lon(), mgps.Time(), mgps.Speed(),
		mgps.DeltaTime()), matchedPoint.GetSegmentId(), matchedPoint.GetMatchedCoord(), matchedPoint.GetPredictedGpsCoord(), matchedPoint.GetBearing())
	cands := make([]Candidate, len(candidates))
	for i, cand := range candidates {
		cands[i] = NewCandidate(cand.GetSegmentId(), cand.GetWeight(), cand.GetLength())
	}
	return &MapmatchingResponse{
		MatchedGpsPoint: matchedGpsPoint,
		Candidates:      cands,
		SpeedMeanK:      speedMeanK,
		SpeedStdK:       speedStdK,
		Bearing:         bearing,
	}
}

// GPX represents gpx.
type GPX struct {
	XMLName xml.Name `xml:"gpx"`
	Trk     Trk      `xml:"trk"`
}

// Trk represents trk.
type Trk struct {
	XMLName xml.Name `xml:"trk"`
	TrkSeg  TrkSeg   `xml:"trkseg"`
}

// TrkSeg represents trk seg.
type TrkSeg struct {
	XMLName xml.Name `xml:"trkseg"`
	TrkPts  []TrkPt  `xml:"trkpt"`
}

// TrkPt represents trk pt.
type TrkPt struct {
	XMLName xml.Name `xml:"trkpt"`
	Lat     float64  `xml:"lat,attr"`
	Lon     float64  `xml:"lon,attr"`
	Time    string   `xml:"time,omitempty"`
}

type offlineMapMatchingEnvelope struct {
	Data OfflineMapMatchingResponse `json:"data"`
}

// OfflineMapMatchingResponse contains matched points and the route path.
type OfflineMapMatchingResponse struct {
	MatchedPoints []*MatchedGPSPoint   `json:"matched_points"`
	RoutePath     []da.FloatCoordinate `json:"route_path"`
}

// NewOfflineMapMatchingResponse creates and returns offline map matching response.
func NewOfflineMapMatchingResponse(matchedPoints []*da.MatchedGPSPoint, routePath []da.FloatCoordinate) *OfflineMapMatchingResponse {
	points := make([]*MatchedGPSPoint, len(matchedPoints))
	for i, mp := range matchedPoints {
		mgps := mp.GetGpsPoint()
		points[i] = NewMatchedGPSPoint(
			&gps{
				Lat:  mgps.Lat(),
				Lon:  mgps.Lon(),
				Time: mgps.Time(),
			},
			mp.GetSegmentId(),
			mp.GetMatchedCoord(),
			mp.GetPredictedGpsCoord(),
			mp.GetBearing(),
		)
	}
	return &OfflineMapMatchingResponse{
		MatchedPoints: points,
		RoutePath:     routePath,
	}
}
