package datastructure

type Coordinate struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

func (c Coordinate) GetLat() float64 {
	return c.Lat
}

func (c Coordinate) GetLon() float64 {
	return c.Lon
}

// 16 byte (128bit)

func NewCoordinate(lat, lon float64) Coordinate {
	return Coordinate{
		Lat: lat,
		Lon: lon,
	}
}

func NewCoordinates(lat, lon []float64) []Coordinate {
	coords := make([]Coordinate, len(lat))
	for i := range lat {
		coords[i] = NewCoordinate(lat[i], lon[i])
	}
	return coords
}
