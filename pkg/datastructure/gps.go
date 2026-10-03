package datastructure

import "time"

type GPSPoint struct {
	lon            float64
	lat            float64
	time           time.Time
	speed          float64 // 0 if time step k=1, speed in m/s
	deltaTime      float64 // in seconds
	directionAngle float64
}

func NewGPSPoint(lat, lon float64, t time.Time, speed, deltaTime float64) *GPSPoint {
	return &GPSPoint{
		lon:       lon,
		lat:       lat,
		time:      t,
		speed:     speed,
		deltaTime: deltaTime,
	}
}

func (gp *GPSPoint) Lon() float64 {
	return gp.lon
}

func (gp *GPSPoint) Lat() float64 {
	return gp.lat
}

func (gp *GPSPoint) GetCoordinate() Coordinate {
	return NewCoordinate(gp.lat, gp.lon)
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

func (gp *GPSPoint) GetDirectionAngle() float64 {
	return gp.directionAngle
}

func (gp *GPSPoint) SetCoord(lat, lon float64) {
	gp.lat, gp.lon = lat, lon
}

// SetDirectionAngle. directionAngle in degrees
func (gp *GPSPoint) SetDirectionAngle(directionAngle float64) {
	gp.directionAngle = directionAngle
}

type MatchedGPSPoint struct {
	gpsPoint          *GPSPoint
	segmentId         Index
	matchedCoord      Coordinate
	predictedGpsCoord Coordinate
	bearing           float64 // bearing dari road segment yang ke match
	observationId     uint32
}

func NewMatchedGPSPoint(gpsPoint *GPSPoint, segmentId Index, matchedCoord Coordinate, bearing float64, observationId uint32) *MatchedGPSPoint {
	return &MatchedGPSPoint{
		gpsPoint:      gpsPoint,
		segmentId:     segmentId,
		matchedCoord:  matchedCoord,
		bearing:       bearing,
		observationId: observationId,
	}
}

func (m *MatchedGPSPoint) SetPredictedGpsCoord(pred Coordinate) {
	m.predictedGpsCoord = pred
}

func (m *MatchedGPSPoint) GetPredictedGpsCoord() Coordinate {
	return m.predictedGpsCoord
}

func (m *MatchedGPSPoint) GetGpsPoint() *GPSPoint {
	return m.gpsPoint
}

func (m *MatchedGPSPoint) GetSegmentId() Index {
	return m.segmentId
}

func (m *MatchedGPSPoint) SetSegmentId(segmentId Index) {
	m.segmentId = segmentId
}

func (m *MatchedGPSPoint) GetMatchedCoord() Coordinate {
	return m.matchedCoord
}

func (m *MatchedGPSPoint) GetBearing() float64 {
	return m.bearing
}

func (m *MatchedGPSPoint) GetObservationId() uint32 {
	return m.observationId
}
