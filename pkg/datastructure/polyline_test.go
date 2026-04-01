package datastructure

import (
	"math/rand"
	"testing"
	"time"
)

func TestGooglePolyline(t *testing.T) {
	testCases := []struct {
		name             string
		coords           *Coordinates
		expectedPolyline string
	}{
		{
			name: "contoh polyline (https://developers.google.com/maps/documentation/utilities/polylinealgorithm)",
			coords: NewCoordinatesWithInitialValues([]Coordinate{
				{
					Lat: 38.5,
					Lon: -120.2,
				},

				{
					Lat: 40.7,
					Lon: -120.95,
				},
				{
					Lat: 43.252,
					Lon: -126.453,
				},
			}),
			expectedPolyline: "_p~iF~ps|U_ulLnnqC_mqNvxq`@",
		},
	}

	for _, tc := range testCases {
		expectedCap := capacity(*tc.coords)
		got := GooglePoylineFromCoords(*tc.coords)
		if got != tc.expectedPolyline {
			t.Errorf("expected %v, got %v", tc.expectedPolyline, got)
		}
		gotBytes := []byte(got)
		if cap(gotBytes) != expectedCap {
			t.Errorf("expected capacity %v, got %v", expectedCap, cap(gotBytes))
		}
	}

	// test random coordinate, cuma assert expected capacity

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	bb := NewBoundingBox(-8.2618, 110.132, -6.888, 110.9221)
	for i := 0; i < 10000; i++ {
		coords := NewCoordinatesWithCap(1000)
		for j := 0; j < 1000; j++ {
			coords.Append([]Coordinate{RandomCoordinate(bb, rd)})
		}

		expectedCap := capacity(*coords)
		got := GooglePoylineFromCoords(*coords)

		gotBytes := []byte(got)
		gotCap := cap(gotBytes)
		if gotCap != expectedCap {
			t.Errorf("expected capacity %v, got %v", expectedCap, gotCap)
		}
	}
}

func RandomCoordinate(bb *BoundingBox, rd *rand.Rand) Coordinate {

	lat := bb.GetMinLat() + rd.Float64()*(bb.GetMaxLat()-bb.GetMinLat())
	lon := bb.GetMinLon() + rd.Float64()*(bb.GetMaxLon()-bb.GetMinLon())
	return NewCoordinate(lat, lon)
}
