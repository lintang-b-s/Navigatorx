package datastructure

import "github.com/twpayne/go-polyline"

func PoylineFromCoords(path []Coordinate) string {

	pathCoords := make([][]float64, len(path))
	for i, p := range path {
		pathCoords[i] = []float64{p.GetLat(), p.GetLon()}
	}

	return string(polyline.EncodeCoords(pathCoords))
}
