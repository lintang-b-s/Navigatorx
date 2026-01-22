package datastructure

import (
	"testing"
)

func TestMerge(t *testing.T) {

	testCases := []struct {
		name     string
		fPoints  []*Point
		gPoints  []*Point
		expected []*Point
	}{

		{
			name: "test merge(f,g) g is constant",
			fPoints: []*Point{NewPoint(0, 60), NewPoint(7200, 15),
				NewPoint(14500, 40)},
			gPoints:  []*Point{NewPoint(0, 80)},
			expected: []*Point{NewPoint(0, 60), NewPoint(7200, 15), NewPoint(14500, 40)},
		},
	}

	for _, tt := range testCases {
		t.Run(tt.name, func(t *testing.T) {
			pwlf := NewPWL(tt.fPoints)

			pwlg := NewPWL(tt.gPoints)
			pwlRes := Merge(pwlf, pwlg)
			for i, pp := range pwlRes.points {
				if !pEqual(pp, tt.expected[i]) {
					t.Error("pp and tt.expected[i] should equal")
				}
			}
		})
	}
}
