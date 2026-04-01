package datastructure

import "testing"

func TestGooglePolyline(t *testing.T) {
	testCases := []struct {
		name             string
		coords           []Coordinate
		expectedPolyline string
	}{
		{
			name: "contoh polyline (https://developers.google.com/maps/documentation/utilities/polylinealgorithm)",
			coords: []Coordinate{
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
			},
			expectedPolyline: "_p~iF~ps|U_ulLnnqC_mqNvxq`@",
		},
	}

	for _, tc := range testCases {
		got := GooglePoylineFromCoords(tc.coords)
		if got != tc.expectedPolyline {
			t.Errorf("expected %v, got %v", tc.expectedPolyline, got)
		}
	}
}
