package datastructure

type Coordinate struct {
	lat float64 `json:"lat"`
	lon float64 `json:"lon"`
}

func (c Coordinate) Lat() float64 {
	return c.lat
}

func (c Coordinate) Lon() float64 {
	return c.lon
}

// 16 byte (128bit)

func NewCoordinate(lat, lon float64) Coordinate {
	return Coordinate{
		lat: lat,
		lon: lon,
	}
}

func NewCoordinates(lat, lon []float64) []Coordinate {
	coords := make([]Coordinate, len(lat))
	for i := range lat {
		coords[i] = NewCoordinate(lat[i], lon[i])
	}
	return coords
}
