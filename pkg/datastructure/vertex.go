package datastructure

import "math"

// Vertex represents a node in the graph with coordinate, degree, and edge offset info.
type Vertex struct {
	coord Coordinate
	pvPtr Index // pointer index to cellNumbers slice

	firstOut Index // index of the first outEdge of this vertex in the flattened graph.heads array (see CSR Graph nya C++ Boost Library: https://www.boost.org/doc/libs/1_61_0/libs/graph/doc/compressed_sparse_row.html)
	firstIn  Index // index of the first inEdge of this vertex in the flattened graph.tails array
	id       Index
}

// NewVertex creates a new Vertex with coordinates and an identifier.
func NewVertex(lat, lon float64, id Index) Vertex {
	coordinate := NewCoordinate(lat, lon)
	return Vertex{
		coord: coordinate,
		id:    id,
	}
}

// NewEmptyVertex creates an empty Vertex initialized with invalid/sentinel values.
func NewEmptyVertex() Vertex {
	invCoord := float64(math.MaxInt32-1) / CoordinatePrecision
	return Vertex{
		coord: NewCoordinate(invCoord, invCoord),
		id:    InvalidVertexId,
	}
}

// SetFirstOut sets the index of the first outgoing edge.
func (v *Vertex) SetFirstOut(firstOut Index) {
	v.firstOut = firstOut
}

// IncrementFirstOut increments the first outgoing edge offset by 1.
func (v *Vertex) IncrementFirstOut() {
	v.firstOut++
}

// AddFirstOut adds the specified value to the first outgoing edge offset.
func (v *Vertex) AddFirstOut(add Index) {
	v.firstOut += add
}

// GetFirstOut returns the index of the first outgoing edge.
func (v *Vertex) GetFirstOut() Index {
	return v.firstOut
}

// GetFirstIn returns the index of the first incoming edge.
func (v *Vertex) GetFirstIn() Index {
	return v.firstIn
}

// AddFirstIn adds the specified value to the first incoming edge offset.
func (v *Vertex) AddFirstIn(add Index) {
	v.firstIn += add
}

// DecrementFirstOut decrements the first outgoing edge offset by 1.
func (v *Vertex) DecrementFirstOut() {
	v.firstOut--
}

// IncrementFirstIn increments the first incoming edge offset by 1.
func (v *Vertex) IncrementFirstIn() {
	v.firstIn++
}

// DecrementFirstIn decrements the first incoming edge offset by 1.
func (v *Vertex) DecrementFirstIn() {
	v.firstIn--
}

// SetFirstIn sets the index of the first incoming edge.
func (v *Vertex) SetFirstIn(firstIn Index) {
	v.firstIn = firstIn
}

// GetId returns the identifier of the vertex.
func (v *Vertex) GetId() Index {
	return v.id
}

// GetLat returns the latitude of the vertex as a float64.
func (v *Vertex) GetLat() float64 {
	return float64(v.coord.lat) / CoordinatePrecision
}

// GetLon returns the longitude of the vertex as a float64.
func (v *Vertex) GetLon() float64 {
	return float64(v.coord.lon) / CoordinatePrecision
}

// GetCoordinate returns the coordinate of the vertex.
func (v *Vertex) GetCoordinate() Coordinate {
	return NewFixedCoordinate(v.coord.lat, v.coord.lon)
}
