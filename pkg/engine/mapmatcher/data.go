package mapmatcher

import da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

type Candidate struct {
	stateId            int
	projectionId       da.Index
	edgeId             da.Index
	distanceFromTail   float64
	distanceFromHead   float64
	travelTimeFromTail float64
	travelTimeFromHead float64

	weight                     float64
	length                     float64
	projectedLat, projectedLon float64
	dist                       float64 // distance to current gps point
	distr                      float64 // distance from tail vertex to projected gps point
}

func (c *Candidate) EdgeId() da.Index {
	return c.edgeId
}

func (c *Candidate) Weight() float64 {
	return c.weight
}

func (c *Candidate) Length() float64 {
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

func NewCandidate(edgeId da.Index, weight, length float64,
) *Candidate {
	return &Candidate{
		edgeId:       edgeId,
		weight:       weight,
		length:       length,
		projectionId: da.INVALID_VERTEX_ID,
	}
}

func (c *Candidate) SetProjectedCoord(lat, lon float64) {
	c.projectedLat, c.projectedLon = lat, lon
}

func (c *Candidate) GetProjectedCoord() da.Coordinate {
	return da.NewCoordinate(c.projectedLat, c.projectedLon)
}

func (c *Candidate) GetProjectionId() da.Index {
	return c.projectionId
}

func (c *Candidate) SetProjectionId(pId da.Index) {
	c.projectionId = pId
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

func (c *Candidate) GetDistr() float64 {
	return c.distr
}

func (c *Candidate) SetDistanceFromTail(dist float64) {
	c.distanceFromTail = dist
}

func (c *Candidate) GetDistanceFromTail() float64 {
	return c.distanceFromTail
}

func (c *Candidate) SetDistanceFromHead(dist float64) {
	c.distanceFromHead = dist
}

func (c *Candidate) GetDistanceFromHead() float64 {
	return c.distanceFromHead
}


func (c *Candidate) SetTravelTimeFromTail(dist float64) {
	c.travelTimeFromTail = dist
}

func (c *Candidate) GetTravelTimeFromTail() float64 {
	return c.travelTimeFromTail
}

func (c *Candidate) SetTravelTimeFromHead(dist float64) {
	c.travelTimeFromHead = dist
}

func (c *Candidate) GetTravelTimeFromHead() float64 {
	return c.travelTimeFromHead
}
