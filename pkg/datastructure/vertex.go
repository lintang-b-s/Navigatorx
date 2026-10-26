package datastructure

import "math"

type Vertex struct {
	coord Coordinate
	pvPtr Index // pointer index to cellNumbers slice

	firstOut Index // index of the first outEdge of this vertex in the flattened graph.heads array (see CSR Graph nya C++ Boost Library: https://www.boost.org/doc/libs/1_61_0/libs/graph/doc/compressed_sparse_row.html)
	firstIn  Index // index of the first inEdge of this vertex in the flattened graph.tails array
	id       Index
}

func NewVertex(lat, lon float64, id Index) Vertex {
	coordinate := NewCoordinate(lat, lon)
	return Vertex{
		coord: coordinate,
		id:    id,
	}
}

func NewEmptyVertex() Vertex {
	invCoord := float64(math.MaxInt32-1) / CoordinatePrecision
	return Vertex{
		coord: NewCoordinate(invCoord, invCoord),
		id:    INVALID_VERTEX_ID,
	}
}

func (v *Vertex) SetFirstOut(firstOut Index) {
	v.firstOut = firstOut
}

func (v *Vertex) IncrementFirstOut() {
	v.firstOut++
}

func (v *Vertex) AddFirstOut(add Index) {
	v.firstOut += add
}

func (v *Vertex) GetFirstOut() Index {
	return v.firstOut
}

func (v *Vertex) GetFirstIn() Index {
	return v.firstIn
}

func (v *Vertex) AddFirstIn(add Index) {
	v.firstIn += add
}

func (v *Vertex) DecrementFirstOut() {
	v.firstOut--
}

func (v *Vertex) IncrementFirstIn() {
	v.firstIn++
}

func (v *Vertex) DecrementFirstIn() {
	v.firstIn--
}

func (v *Vertex) SetFirstIn(firstIn Index) {
	v.firstIn = firstIn
}

func (v *Vertex) GetID() Index {
	return v.id
}

func (v *Vertex) GetLat() float64 {
	return float64(v.coord.lat) / CoordinatePrecision
}

func (v *Vertex) GetLon() float64 {
	return float64(v.coord.lon) / CoordinatePrecision
}

func (v *Vertex) GetCoordinate() Coordinate {
	return NewFixedCoordinate(v.coord.lat, v.coord.lon)
}
