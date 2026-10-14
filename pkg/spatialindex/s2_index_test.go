package spatialindex

import (
	"testing"

	"github.com/golang/geo/s2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/stretchr/testify/assert"
)

func TestGetCellSegments(t *testing.T) {
	testCases := []struct {
		name         string
		s2CellId     uint64 // level-15 s2 cell id
		segments     [][]da.Coordinate
		radius       float64
		radiusFail   float64
		wantSegments []da.Index
	}{{
		name:     "jalan solo",
		s2CellId: 3349011539087589376, // https://igorgatis.github.io/ws2/?cells=2e7a14404
		segments: [][]da.Coordinate{
			{
				// https://www.openstreetmap.org/way/1545342473
				da.NewCoordinate(-7.548392, 110.782761),
				da.NewCoordinate(-7.548772, 110.784030),
				da.NewCoordinate(-7.549190, 110.785363),
			},
			{
				// https://www.openstreetmap.org/way/469663497
				da.NewCoordinate(-7.559046, 110.785688),
				da.NewCoordinate(-7.558493, 110.783831),
				da.NewCoordinate(-7.557701, 110.781316),
			},
			{
				// https://www.openstreetmap.org/way/803197687
				da.NewCoordinate(-7.544172, 110.771247),
				da.NewCoordinate(-7.547836, 110.770200),
				da.NewCoordinate(-7.551277, 110.769149),
			},
			{
				// https://www.openstreetmap.org/way/103762976
				da.NewCoordinate(-7.583678, 110.825969),
				da.NewCoordinate(-7.584284, 110.827932),
				da.NewCoordinate(-7.584614, 110.829177),
			},
			{
				//https://www.openstreetmap.org/way/1352865870
				da.NewCoordinate(-7.559435, 110.843296),
				da.NewCoordinate(-7.558781, 110.844911),
				da.NewCoordinate(-7.558132, 110.846311),
			},
			{
				// https://www.openstreetmap.org/way/585010307
				da.NewCoordinate(-7.542263, 110.834702),
				da.NewCoordinate(-7.541923, 110.833308),
			},
		},
		radius:       1.5,
		radiusFail:   0.05,
		wantSegments: []da.Index{0, 1, 2},
	}}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			sidx := &S2RoadSegmentsIndex{}
			sidx.idx = make(map[s2.CellID][]da.Index, 1000)
			coverer := NewFlatCoverer(15)

			for id, seg := range tc.segments {
				sidx.AddPolyline(da.Index(id), seg, coverer)
			}

			got := sidx.GetCellSegments(s2.CellID(tc.s2CellId), tc.radius)
			assert.ElementsMatch(t, got, tc.wantSegments)

			gotFail := sidx.GetCellSegments(s2.CellID(tc.s2CellId), tc.radiusFail)
			assert.NotElementsMatch(t, gotFail, tc.wantSegments)
		})
	}

}
