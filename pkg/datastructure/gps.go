package datastructure

import "time"

// GPSPoint represents gpspoint.
type GPSPoint struct {
	lon            float64
	lat            float64
	time           time.Time
	speed          float64 // 0 if time step k=1, speed in m/s
	deltaTime      float64 // in seconds
	directionAngle float64
}

// NewGPSPoint creates and returns gpspoint.
func NewGPSPoint(lat, lon float64, t time.Time, speed, deltaTime float64) *GPSPoint {
	return &GPSPoint{
		lon:       lon,
		lat:       lat,
		time:      t,
		speed:     speed,
		deltaTime: deltaTime,
	}
}

// Lon handles lon.
func (gp *GPSPoint) Lon() float64 {
	return gp.lon
}

// Lat handles lat.
func (gp *GPSPoint) Lat() float64 {
	return gp.lat
}

// GetCoordinate returns coordinate.
func (gp *GPSPoint) GetCoordinate() Coordinate {
	return NewCoordinate(gp.lat, gp.lon)
}

// Time handles time.
func (gp *GPSPoint) Time() time.Time {
	return gp.time
}

// Speed handles speed.
func (gp *GPSPoint) Speed() float64 {
	return gp.speed
}

// DeltaTime handles delta time.
func (gp *GPSPoint) DeltaTime() float64 {
	return gp.deltaTime
}

// GetDirectionAngle returns direction angle.
func (gp *GPSPoint) GetDirectionAngle() float64 {
	return gp.directionAngle
}

// SetCoord sets coord.
func (gp *GPSPoint) SetCoord(lat, lon float64) {
	gp.lat, gp.lon = lat, lon
}

// SetDirectionAngle sets direction angle.
// SetDirectionAngle. directionAngle in degrees
func (gp *GPSPoint) SetDirectionAngle(directionAngle float64) {
	gp.directionAngle = directionAngle
}

// MatchedGPSPoint represents matched gpspoint.
type MatchedGPSPoint struct {
	gpsPoint          *GPSPoint
	segmentId         Index // routing engine segment id in graph.go
	matchedCoord      Coordinate
	predictedGpsCoord Coordinate
	bearing           float64 // bearing dari road segment yang ke match
}

// NewMatchedGPSPoint creates and returns matched gpspoint.
func NewMatchedGPSPoint(gpsPoint *GPSPoint, segmentId Index, matchedCoord Coordinate, bearing float64) *MatchedGPSPoint {
	return &MatchedGPSPoint{
		gpsPoint:     gpsPoint,
		segmentId:    segmentId,
		matchedCoord: matchedCoord,
		bearing:      bearing,
	}
}

// SetPredictedGpsCoord sets predicted gps coord.
func (m *MatchedGPSPoint) SetPredictedGpsCoord(pred Coordinate) {
	m.predictedGpsCoord = pred
}

// GetPredictedGpsCoord returns predicted gps coord.
func (m *MatchedGPSPoint) GetPredictedGpsCoord() Coordinate {
	return m.predictedGpsCoord
}

// GetGpsPoint returns gps point.
func (m *MatchedGPSPoint) GetGpsPoint() *GPSPoint {
	return m.gpsPoint
}

// GetSegmentId returns segment id.
func (m *MatchedGPSPoint) GetSegmentId() Index {
	return m.segmentId
}

// SetSegmentId sets segment id.
func (m *MatchedGPSPoint) SetSegmentId(segmentId Index) {
	m.segmentId = segmentId
}

// GetMatchedCoord returns matched coord.
func (m *MatchedGPSPoint) GetMatchedCoord() Coordinate {
	return m.matchedCoord
}

// GetBearing returns bearing.
func (m *MatchedGPSPoint) GetBearing() float64 {
	return m.bearing
}
