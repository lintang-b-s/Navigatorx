package datastructure

import (
	"fmt"
	"log"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// CoordinatePrecision is the coordinate precision value.
var CoordinatePrecision = 1e7

// FloatCoordinate represents float coordinate.
type FloatCoordinate struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

// NewFloatCoordinate creates and returns float coordinate.
func NewFloatCoordinate(lat, lon float64) FloatCoordinate {
	return FloatCoordinate{Lat: lat, Lon: lon}
}

// Coordinate represents coordinate.
type Coordinate struct {
	lat int32 // latitude * 10^7 .terinspirasi dari how OSRM store osm node coordinates & save space (https://github.com/Project-OSRM/osrm-backend/blob/master/include/util/coordinate.hpp)
	lon int32 // longitude * 10^7
}

// GetLat returns lat.
func (c Coordinate) GetLat() float64 {
	return float64(c.lat) / CoordinatePrecision
}

// GetLon returns lon.
func (c Coordinate) GetLon() float64 {
	return float64(c.lon) / CoordinatePrecision
}

// GetFixedLat returns fixed lat.
func (c Coordinate) GetFixedLat() int32 {
	return c.lat
}

// GetFixedLon returns fixed lon.
func (c Coordinate) GetFixedLon() int32 {
	return c.lon
}

// ToFloatCoordinate handles to float coordinate.
func (c Coordinate) ToFloatCoordinate() FloatCoordinate {
	return NewFloatCoordinate(c.GetLat(), c.GetLon())
}

// IsSameCoordinate reports whether same coordinate.
func IsSameCoordinate(a, b Coordinate) bool {
	return util.Eq(a.lat, b.lat) && util.Eq(a.lon, b.lon)
}

// NewCoordinate creates and returns coordinate.
func NewCoordinate(lat, lon float64) Coordinate {
	fixedLat, err := coordinateToFixed(lat, "latitude")
	if err != nil {
		log.Fatal(err.Error())
	}
	fixedLon, err := coordinateToFixed(lon, "longitude")
	if err != nil {
		log.Fatal(err.Error())
	}
	return NewFixedCoordinate(fixedLat, fixedLon)
}

// NewCoordinateWithoutScaling creates and returns coordinate without scaling.
func NewCoordinateWithoutScaling(lat, lon float64) Coordinate {
	fixedLat := int32(math.Round(lat))
	fixedLon := int32(math.Round(lon))
	return NewFixedCoordinate(fixedLat, fixedLon)
}

// NewCoordinates build []Coordinate from coords array. coords[i][0] is lat of i-th coordinate, coords[i][1] is lon of i-th coordinate.
func NewCoordinates(coords [][]float64) []Coordinate {
	res := make([]Coordinate, len(coords))
	for i := 0; i < len(coords); i++ {
		res[i] = NewCoordinate(coords[i][0], coords[i][1])
	}
	return res
}

// NewFixedCoordinate creates and returns fixed coordinate.
func NewFixedCoordinate(lat, lon int32) Coordinate {
	return Coordinate{lat: lat, lon: lon}
}

func coordinateToFixed(value float64, name string) (int32, error) {
	fixed := math.Round(value * CoordinatePrecision)
	if fixed <= math.MinInt32 || fixed > math.MaxInt32 {
		return 0, fmt.Errorf("%s %.12g exceeds fixed-coordinate range", name, value)
	}
	return int32(fixed), nil
}

// NewInvalidCoordinate creates and returns invalid coordinate.
func NewInvalidCoordinate() Coordinate {
	return NewCoordinate(pkg.InvalidLat, pkg.InvalidLon)
}

// Coordinates represents coordinates.
type Coordinates []Coordinate

// NewCoordinatesWithCap creates and returns coordinates with cap.
func NewCoordinatesWithCap(capacity int) *Coordinates {
	cs := make(Coordinates, 0, capacity)
	return &cs
}

// NewCoordinatesWithInitialValues creates and returns coordinates with initial values.
func NewCoordinatesWithInitialValues(newCoords []Coordinate) *Coordinates {
	cs := make(Coordinates, 0, len(newCoords))
	cs = append(cs, newCoords...)
	return &cs
}

// Append appends .
// https://go.dev/doc/effective_go#pointers_vs_values
func (cs *Coordinates) Append(newCoords []Coordinate) {
	slice := *cs
	slice = append(slice, newCoords...)
	*cs = slice
}

// Prepend handles prepend.
func (cs *Coordinates) Prepend(newCoords []Coordinate) {
	slice := *cs
	newCoords = append(newCoords, slice...)
	*cs = newCoords
}

// AppendCoordinate appends one coordinate without creating a temporary slice.
func (cs *Coordinates) AppendCoordinate(coord Coordinate) {
	*cs = append(*cs, coord)
}

// Reset resets coordinates value.
// https://go.dev/doc/effective_go#pointers_vs_values
func (cs *Coordinates) Reset() {
	slice := *cs
	slice = slice[:0]
	*cs = slice
}

// Get returns i-th coordinate.
// https://go.dev/doc/effective_go#pointers_vs_values
func (cs *Coordinates) Get(i int) Coordinate {
	slice := *cs
	return slice[i]
}

// Length get length of coordinates.
// https://go.dev/doc/effective_go#pointers_vs_values
func (cs *Coordinates) Length() int {
	slice := *cs
	return len(slice)
}

// ToFloatCoordinates handles to float coordinates.
func (cs *Coordinates) ToFloatCoordinates() []FloatCoordinate {
	coords := make([]FloatCoordinate, cs.Length())
	for i := 0; i < cs.Length(); i++ {
		coords[i] = cs.Get(i).ToFloatCoordinate()
	}
	return coords
}
