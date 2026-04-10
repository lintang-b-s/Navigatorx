package geo

import (
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

func TestCurvedPolyline(t *testing.T) {
	testCases := []struct {
		name   string
		coords da.Coordinates
		want   bool
	}{
		{
			name: "Jalan Tentara Rakyat Mataram not curved https://www.openstreetmap.org/way/153857844",
			coords: []da.Coordinate{
				da.NewCoordinate(-7.7878482, 110.3589005),
				da.NewCoordinate(-7.7879126, 110.3587503),
				da.NewCoordinate(-7.7879722, 110.3585639),
				da.NewCoordinate(-7.7880526, 110.3582849),
			},
			want: false,
		},
		{
			name: "Jalan Tentara Rakyat Mataram curved https://www.openstreetmap.org/way/153857844",
			coords: []da.Coordinate{
				da.NewCoordinate(-7.7878482, 110.3589005),
				da.NewCoordinate(-7.7879126, 110.3587503),
				da.NewCoordinate(-7.7879722, 110.3585639),
				da.NewCoordinate(-7.7880526, 110.3582849),
				da.NewCoordinate(-7.7882019, 110.3577363),
				da.NewCoordinate(-7.7882019, 110.3577363),
				da.NewCoordinate(-7.7882260, 110.3576760),
				da.NewCoordinate(-7.7882724, 110.3576153),
				da.NewCoordinate(-7.7883012, 110.3575910),
				da.NewCoordinate(-7.7883752, 110.3575567),
				da.NewCoordinate(-7.7889260, 110.3575328),
				da.NewCoordinate(-7.7891283, 110.3575265),
				da.NewCoordinate(-7.7893485, 110.3575253),
			},
			want: true,
		},
		{
			name: "Jalan Kapten Pierre Tandean curved https://www.openstreetmap.org/way/103762969",
			coords: []da.Coordinate{
				da.NewCoordinate(-7.5461981, 110.8206405),
				da.NewCoordinate(-7.5458928, 110.8205551),
				da.NewCoordinate(-7.5455448, 110.8204328),
				da.NewCoordinate(-7.5455183, 110.8204234),
				da.NewCoordinate(-7.5453045, 110.8203532),
				da.NewCoordinate(-7.5450650, 110.8202974),
				da.NewCoordinate(-7.5447650, 110.8202506),
				da.NewCoordinate(-7.5442552, 110.8202069),
				da.NewCoordinate(-7.5432413, 110.8202486),
			},
			want: true,
		},
		{
			name: "Jalan Kapten Pierre Tandean curved 2 https://www.openstreetmap.org/way/780753520",
			coords: []da.Coordinate{
				da.NewCoordinate(-7.5504227, 110.8217724),
				da.NewCoordinate(-7.5504954, 110.8218209),
				da.NewCoordinate(-7.5505793, 110.8218489),
				da.NewCoordinate(-7.5506165, 110.8218591),
				da.NewCoordinate(-7.5507078, 110.8218763),
				da.NewCoordinate(-7.5507799, 110.8218900),
				da.NewCoordinate(-7.5508158, 110.8218967),
				da.NewCoordinate(-7.5508776, 110.8218987),
				da.NewCoordinate(-7.5508776, 110.8218987),
				da.NewCoordinate(-7.5509377, 110.8218879),
				da.NewCoordinate(-7.5510971, 110.8218455),
				da.NewCoordinate(-7.5513426, 110.8217529),
			},
			want: true,
		},

		{
			name: "Jalan Brigadir Jenderal Slamet Riyadi Sriwedari https://www.openstreetmap.org/way/170406373",
			coords: []da.Coordinate{
				da.NewCoordinate(-7.5653033, 110.8060187),
				da.NewCoordinate(-7.5654810, 110.8065900),
				da.NewCoordinate(-7.5656668, 110.8071998),
				da.NewCoordinate(-7.5657070, 110.8073357),
				da.NewCoordinate(-7.5658559, 110.8078212),
				da.NewCoordinate(-7.5658714, 110.8078698),
				da.NewCoordinate(-7.5658771, 110.8078876),
				da.NewCoordinate(-7.5662653, 110.8091301),
				da.NewCoordinate(-7.5664447, 110.8097294),
				da.NewCoordinate(-7.5664650, 110.8097972),
			},
			want: false,
		},
		{
			name: "Jalan Insinyur Soekarno https://www.openstreetmap.org/way/960747057",
			coords: []da.Coordinate{
				da.NewCoordinate(-7.5976223, 110.8150995),
				da.NewCoordinate(-7.5977351, 110.8150662),
				da.NewCoordinate(-7.5978314, 110.8150355),
				da.NewCoordinate(-7.5979802, 110.8149879),
				da.NewCoordinate(-7.5981805, 110.8149305),
				da.NewCoordinate(-7.5982272, 110.8149167),
				da.NewCoordinate(-7.5984987, 110.8148324),
				da.NewCoordinate(-7.5992616, 110.8146046),
			},
			want: false,
		},

		{
			name: "Jalan Menteri Supeno Manahan https://www.openstreetmap.org/way/1420694244",
			coords: []da.Coordinate{
				da.NewCoordinate(-7.5552297, 110.8083034),
				da.NewCoordinate(-7.5551912, 110.8082496),
				da.NewCoordinate(-7.5551337, 110.8081587),
				da.NewCoordinate(-7.5548185, 110.8076550),
				da.NewCoordinate(-7.5545622, 110.8072470),
				da.NewCoordinate(-7.5540497, 110.8064313),
			},
			want: false,
		},
		{
			name: "Jalan Fly Over Manahan https://www.openstreetmap.org/way/763555609",
			coords: []da.Coordinate{
				da.NewCoordinate(-7.5587959, 110.8077397),
				da.NewCoordinate(-7.5587272, 110.8077395),
				da.NewCoordinate(-7.5586540, 110.8077183),
				da.NewCoordinate(-7.5585832, 110.8076763),
				da.NewCoordinate(-7.5585239, 110.8076233),
				da.NewCoordinate(-7.5584718, 110.8075624),
				da.NewCoordinate(-7.5583975, 110.8074507),
				da.NewCoordinate(-7.5578310, 110.8065238),
			},
			want: true,
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			got := IsPolylineCurved(tc.coords)
			if got != tc.want {
				t.Errorf("want: %v, got: %v", tc.want, got)
			}
		})
	}
}
