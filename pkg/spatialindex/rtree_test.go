package spatialindex

import (
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
)

func TestSearchWithinRadiusMapMatchingSortsAndLimitsCandidates(t *testing.T) {
	rt := NewRtree()
	insertPoint := func(eId da.Index, lat, lon float64) {
		x := geo.CalcLonToX(lon)
		y := geo.CalcLatToY(lat)
		rt.tr.Insert([2]float64{x, y}, [2]float64{x, y}, rt.BitPackOriginalEdgeId(eId, eId+100))
	}

	insertPoint(0, 0, 0.05)
	insertPoint(1, 0, 0.01)
	insertPoint(2, 0, 0.03)
	insertPoint(3, 0, 0.02)
	insertPoint(4, 0, 0.06)
	insertPoint(5, 0, 0.04)

	got := rt.SearchWithinRadius(0, 0, 10, 3)
	want := []da.Index{1, 3, 2, 5, 0}

	if len(got) != len(want) {
		t.Fatalf("candidate count mismatch: got %d, want %d (%v)", len(got), len(want), got)
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("candidate %d mismatch: got %d, want %d (all candidates %v)", i, got[i], want[i], got)
		}
	}
}
