package datastructure

import "time"

type GPSPoint struct {
	lon           float64
	lat           float64
	time          time.Time
	speed         float64 // 0 if time step k=1
	deltaTime     float64 // in minute
	deadReckoning bool
}

func NewGPSPoint(lat, lon float64, t time.Time, speed, deltaTime float64,
	deadReckoning bool) *GPSPoint {
	return &GPSPoint{
		lon:           lon,
		lat:           lat,
		time:          t,
		speed:         speed,
		deltaTime:     deltaTime,
		deadReckoning: deadReckoning,
	}
}

func (gp *GPSPoint) Lon() float64 {
	return gp.lon
}

func (gp *GPSPoint) Lat() float64 {
	return gp.lat
}

func (gp *GPSPoint) Time() time.Time {
	return gp.time
}

func (gp *GPSPoint) Speed() float64 {
	return gp.speed
}

func (gp *GPSPoint) DeltaTime() float64 {
	return gp.deltaTime
}

func (gp *GPSPoint) GetDeadReckoning() bool {
	return gp.deadReckoning
}

func (gp *GPSPoint) SetCoord(lat, lon float64) {
	gp.lat, gp.lon = lat, lon
}

type MatchedGPSPoint struct {
	gpsPoint          *GPSPoint
	edgeId            Index
	matchedCoord      Coordinate
	predictedGpsCoord Coordinate
	bearing           float64 // bearing dari road segment yang ke match
}

func NewMatchedGPSPoint(gpsPoint *GPSPoint, edgeId Index, matchedCoord Coordinate, bearing float64) *MatchedGPSPoint {
	return &MatchedGPSPoint{
		gpsPoint:     gpsPoint,
		edgeId:       edgeId,
		matchedCoord: matchedCoord,
		bearing:      bearing,
	}
}

func (m *MatchedGPSPoint) SetPredictedGpsCoord(pred Coordinate) {
	m.predictedGpsCoord = pred
}

func (m *MatchedGPSPoint) GetPredictedGpsCoord()Coordinate {
	return m.predictedGpsCoord
}

func (m *MatchedGPSPoint) GetGpsPoint() *GPSPoint {
	return m.gpsPoint
}

func (m *MatchedGPSPoint) GetEdgeId() Index {
	return m.edgeId
}

func (m *MatchedGPSPoint) GetMatchedCoord() Coordinate {
	return m.matchedCoord
}

func (m *MatchedGPSPoint) GetBearing() float64 {
	return m.bearing
}
