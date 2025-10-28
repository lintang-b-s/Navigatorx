package datastructure

import "time"

type GPSPoint struct {
	lon   float64
	lat   float64
	time  time.Time
	speed float64 // 0 if time step k=0
}

func NewGPSPoint(lat, lon float64, t time.Time, speed float64) *GPSPoint {
	return &GPSPoint{
		lon:   lon,
		lat:   lat,
		time:  t,
		speed: speed,
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

type MatchedGPSPoint struct {
	gpsPoint     *GPSPoint
	edgeId       Index
	matchedCoord Coordinate
}

func NewMatchedGPSPoint(gpsPoint *GPSPoint, edgeId Index, matchedCoord Coordinate) *MatchedGPSPoint {
	return &MatchedGPSPoint{
		gpsPoint:     gpsPoint,
		edgeId:       edgeId,
		matchedCoord: matchedCoord,
	}
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
