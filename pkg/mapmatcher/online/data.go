package online

import "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

type Candidate struct {
	edgeId datastructure.Index
	weight float64
	length float64
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

func NewCandidate(edgeId datastructure.Index, weight, length float64) *Candidate {
	return &Candidate{
		edgeId: edgeId,
		weight: weight,
		length: length,
	}
}
