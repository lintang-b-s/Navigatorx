package online

import "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

type Candidate struct {
	edgeId                     datastructure.Index
	weight                     float64
	length                     float64
	projectedLat, projectedLon float64
	dist                       float64 // distance to current gps point
	distr                      float64 // distance from tail vertex to projected gps point
}

func (c *Candidate) EdgeId() datastructure.Index {
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

func NewCandidate(edgeId datastructure.Index, weight, length float64,
) *Candidate {
	return &Candidate{
		edgeId: edgeId,
		weight: weight,
		length: length,
	}
}

func (c *Candidate) SetProjectedCoord(lat, lon float64) {
	c.projectedLat, c.projectedLon = lat, lon
}

func (c *Candidate) GetProjectedCoord() datastructure.Coordinate {
	return datastructure.NewCoordinate(c.projectedLat, c.projectedLon)
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
