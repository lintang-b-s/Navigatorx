package datastructure

import (
	"fmt"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
)

var CoordinatePrecision = 1e7

const invalidFixedCoordinate = math.MinInt32

type Coordinate struct {
	lat int32 // latitude * 10^7 .terinspirasi dari how OSRM store osm node coordinates & save space
	lon int32 // longitude * 10^7
}

func (c Coordinate) GetLat() float64 {
	return float64(c.lat) / CoordinatePrecision
}

func (c Coordinate) GetLon() float64 {
	return float64(c.lon) / CoordinatePrecision
}

func (c Coordinate) GetFixedLat() int32 {
	return c.lat
}

func (c Coordinate) GetFixedLon() int32 {
	return c.lon
}

func (c Coordinate) IsValid() bool {
	return c.lat != invalidFixedCoordinate && c.lon != invalidFixedCoordinate
}

func IsSameCoordinate(a, b Coordinate) bool {
	return a == b
}

func NewCoordinate(lat, lon float64) Coordinate {
	fixedLat, err := coordinateToFixed(lat, "latitude")
	if err != nil {
		panic(err)
	}
	fixedLon, err := coordinateToFixed(lon, "longitude")
	if err != nil {
		panic(err)
	}
	return NewFixedCoordinate(fixedLat, fixedLon)
}

func NewFixedCoordinate(lat, lon int32) Coordinate {
	return Coordinate{lat: lat, lon: lon}
}

func NewUninitializedCoordinate() Coordinate {
	return NewFixedCoordinate(invalidFixedCoordinate, invalidFixedCoordinate)
}

func coordinateToFixed(value float64, name string) (int32, error) {
	fixed := math.Round(value * CoordinatePrecision)
	if fixed <= math.MinInt32 || fixed > math.MaxInt32 {
		return 0, fmt.Errorf("%s %.12g exceeds fixed-coordinate range", name, value)
	}
	return int32(fixed), nil
}

func NewInvalidCoordinate() Coordinate {
	return NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON)
}

type Coordinates []Coordinate

func NewCoordinatesWithCap(capacity int) *Coordinates {
	cs := make(Coordinates, 0, capacity)
	return &cs
}

func NewCoordinatesWithInitialValues(newCoords []Coordinate) *Coordinates {
	cs := make(Coordinates, 0, len(newCoords))
	cs = append(cs, newCoords...)
	return &cs
}

// https://go.dev/doc/effective_go#pointers_vs_values
func (cs *Coordinates) Append(newCoords []Coordinate) {
	slice := *cs
	slice = append(slice, newCoords...)
	*cs = slice
}

// https://go.dev/doc/effective_go#pointers_vs_values
func (cs *Coordinates) Prepend(newCoords []Coordinate) {
	slice := *cs
	slice = append(newCoords, slice...)
	*cs = slice
}

// https://go.dev/doc/effective_go#pointers_vs_values
func (cs *Coordinates) Reset() {
	slice := *cs
	slice = slice[:0]
	*cs = slice
}

// https://go.dev/doc/effective_go#pointers_vs_values
func (cs *Coordinates) Get(i int) Coordinate {
	slice := *cs
	return slice[i]
}

// https://go.dev/doc/effective_go#pointers_vs_values
func (cs *Coordinates) Length() int {
	slice := *cs
	return len(slice)
}
