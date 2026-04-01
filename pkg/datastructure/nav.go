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

