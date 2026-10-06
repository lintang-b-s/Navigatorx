// Package mapmatcher provides data structures and logic for online map matching.
package mapmatcher

import da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

type Candidate struct {
	stateId int

	segmentId        da.Index // the road segment id in graph.go (for offline map matching) or dynamic_graph.go (for online map matching)
	rnId             da.Index // road segment in in graph.go when doing online map matching
	distanceFromHead float64  // in meter
	costFromTail     float64
	costFromHead     float64
	segmentBearing   float64 // in degrees

	weight                     float64 // posterior probability dari this road segment candidate at time step k
	length                     float64 //
	projectedLat, projectedLon float64
	dist                       float64 // distance to current gps point in meter
	distr                      float64 // distance from tail vertex to projected gps point in meter

}

// segmentId return DynamicGraph segmentId dari candidate
func (c *Candidate) GetSegmentId() da.Index {
	return c.segmentId
}

func (c *Candidate) GetWeight() float64 {
	return c.weight
}

func (c *Candidate) GetLength() float64 {
	return c.length
}

func (c *Candidate) SetWeight(w float64) {
	c.weight = w
}

func (c *Candidate) SetLength(l float64) {
	c.length = l
}

func (c *Candidate) SetStateId(sid int) {
	c.stateId = sid
}

func (c *Candidate) GetStateId() int {
	return c.stateId
}

// NewCandidate create new map matching road segment candidate
// segmentId is the road segment id in graph.go (for offline map matching) or dynamic_graph.go (for online map matching)
// weight is the duration of the road segment
// length is the legth in meters of the road segment
func NewCandidate(segmentId da.Index, weight, length float64,
) *Candidate {
	return &Candidate{
		segmentId: segmentId,
		weight:    weight,
		length:    length,
	}
}

func (c *Candidate) SetProjectedCoord(lat, lon float64) {
	c.projectedLat, c.projectedLon = lat, lon
}

func (c *Candidate) SetSegmentBearing(segmentBearingDeg float64) {
	c.segmentBearing = segmentBearingDeg
}

func (c *Candidate) SetRoadNetworkId(rnId da.Index) {
	c.rnId = rnId
}

func (c *Candidate) GetRoadNetworkId() da.Index {
	return c.rnId
}

func (c *Candidate) GetSegmentBearing() float64 {
	return c.segmentBearing
}

func (c *Candidate) GetProjectedCoord() da.Coordinate {
	return da.NewCoordinate(c.projectedLat, c.projectedLon)
}

func (c *Candidate) SetDist(dist float64) {
	c.dist = dist
}

func (c *Candidate) SetDistr(distr float64) {
	c.distr = distr
}

func (c *Candidate) GetDist() float64 {
	return c.dist
}

// GetDistr get distance from tail vertex to projected gps point
func (c *Candidate) GetDistr() float64 {
	return c.distr
}

func (c *Candidate) SetDistanceFromHead(dist float64) {
	c.distanceFromHead = dist
}

func (c *Candidate) GetDistanceFromHead() float64 {
	return c.distanceFromHead
}

func (c *Candidate) SetCostFromTail(dist float64) {
	c.costFromTail = dist
}

func (c *Candidate) GetCostFromTail() float64 {
	return c.costFromTail
}

func (c *Candidate) SetCostFromHead(dist float64) {
	c.costFromHead = dist
}

func (c *Candidate) GetCostFromHead() float64 {
	return c.costFromHead
}
