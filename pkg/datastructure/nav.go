package datastructure

import "github.com/lintang-b-s/Navigatorx/pkg/geo"

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

func NewGeoCoordinates(coords []Coordinate) []geo.Coordinate {
	geoCoords := make([]geo.Coordinate, len(coords))
	for i, coord := range coords {
		geoCoords[i] = geo.NewCoordinate(coord.GetLat(), coord.GetLon())
	}
	return geoCoords
}

func (c Coordinate) ToGeoCoordinate() geo.Coordinate {

	return geo.NewCoordinate(c.GetLat(), c.GetLon())
}
