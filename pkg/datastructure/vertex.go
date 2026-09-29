package datastructure

type Vertex struct {
	lat   int32
	lon   int32
	pvPtr Index // pointer index to cellNumbers slice

	firstOut Index // index of the first outEdge of this vertex in the flattened graph.outEdges array (see CSR Graph nya C++ Boost Library: https://www.boost.org/doc/libs/latest/libs/graph/doc/compressed_sparse_row.html)
	firstIn  Index // index of the first inEdge of this vertex in the flattened graph.inEdges array
	id       Index
}

func NewVertex(lat, lon float64, id Index) Vertex {
	coordinate := NewCoordinate(lat, lon)
	return Vertex{
		lat: coordinate.GetFixedLat(),
		lon: coordinate.GetFixedLon(),
		id:  id,
	}
}

func NewEmptyVertex() Vertex {
	return Vertex{
		lat: invalidFixedCoordinate,
		lon: invalidFixedCoordinate,
		id:  INVALID_VERTEX_ID,
	}
}

func (v *Vertex) SetFirstOut(firstOut Index) {
	v.firstOut = firstOut
}

func (v *Vertex) SetFirstIn(firstIn Index) {
	v.firstIn = firstIn
}

func (v *Vertex) SetId(id Index) {
	v.id = id
}
func (v *Vertex) SetPvPtr(pvPtr Index) {
	v.pvPtr = pvPtr
}

func (v *Vertex) GetID() Index {
	return v.id
}

func (v *Vertex) GetLat() float64 {
	return float64(v.lat) / CoordinatePrecision
}

func (v *Vertex) GetLon() float64 {
	return float64(v.lon) / CoordinatePrecision
}

func (v *Vertex) GetCoordinate() Coordinate {
	return NewFixedCoordinate(v.lat, v.lon)
}

func (v *Vertex) GetFirstOut() Index {
	return v.firstOut
}

func (v *Vertex) GetFirstIn() Index {
	return v.firstIn
}

func (v *Vertex) GetPvPtr() Index {
	return v.pvPtr
}
