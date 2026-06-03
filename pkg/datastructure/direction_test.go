package datastructure

import (
	"reflect"
	"testing"
)

func equalFloat64Slice(a, b []float64) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

func equalIndexSlice(a, b []Index) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

func equalCoordinates(a, b Coordinates) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

func TestIsContinue(t *testing.T) {
	testCases := []struct {
		name string
		tt   TurnType
		want bool
	}{
		{
			name: "continue_on_street_is_true",
			tt:   CONTINUE_ON_STREET,
			want: true,
		},
		{
			name: "turn_left_is_false",
			tt:   TURN_LEFT,
			want: false,
		},
		{
			name: "start_is_false",
			tt:   START,
			want: false,
		},
	}

	for _, tc := range testCases {
		tc := tc
		t.Run(tc.name, func(t *testing.T) {
			got := IsContinue(tc.tt)
			if got != tc.want {
				t.Fatalf("IsContinue(%v): want %v, got %v", tc.tt, tc.want, got)
			}
		})
	}
}

func TestIsTurnSlight(t *testing.T) {
	testCases := []struct {
		name string
		tt   TurnType
		want bool
	}{
		{
			name: "slight_left_is_true",
			tt:   TURN_SLIGHT_LEFT,
			want: true,
		},
		{
			name: "slight_right_is_true",
			tt:   TURN_SLIGHT_RIGHT,
			want: true,
		},
		{
			name: "continue_is_true",
			tt:   CONTINUE_ON_STREET,
			want: true,
		},
		{
			name: "turn_right_is_false",
			tt:   TURN_RIGHT,
			want: false,
		},
	}

	for _, tc := range testCases {
		tc := tc
		t.Run(tc.name, func(t *testing.T) {
			got := IsTurnSlight(tc.tt)
			if got != tc.want {
				t.Fatalf("IsTurnSlight(%v): want %v, got %v", tc.tt, tc.want, got)
			}
		})
	}
}

func TestNewAnnotationCopiesInputs(t *testing.T) {
	testCases := []struct {
		name           string
		duration       []float64
		distance       []float64
		geometry       Coordinates
		edgeGeomOffset []Index
	}{
		{
			name:           "populated_slices_are_copied",
			duration:       []float64{1.25, 3.5},
			distance:       []float64{10, 20},
			geometry:       Coordinates{{Lat: -7.9, Lon: 112.6}, {Lat: -7.8, Lon: 112.7}},
			edgeGeomOffset: []Index{0, 2},
		},
		{
			name:           "nil_inputs_produce_empty_slices",
			duration:       nil,
			distance:       nil,
			geometry:       nil,
			edgeGeomOffset: nil,
		},
	}

	for _, tc := range testCases {
		tc := tc
		t.Run(tc.name, func(t *testing.T) {
			ann := NewAnnotation(tc.duration, tc.distance, tc.geometry, tc.edgeGeomOffset)

			if !equalFloat64Slice(ann.GetDuration(), tc.duration) {
				t.Fatalf("duration mismatch: want %v, got %v", tc.duration, ann.GetDuration())
			}
			if !equalFloat64Slice(ann.GetDistance(), tc.distance) {
				t.Fatalf("distance mismatch: want %v, got %v", tc.distance, ann.GetDistance())
			}
			if !equalCoordinates(ann.GetGeometry(), tc.geometry) {
				t.Fatalf("geometry mismatch: want %v, got %v", tc.geometry, ann.GetGeometry())
			}
			if !equalIndexSlice(ann.GetEdgeGeomOffset(), tc.edgeGeomOffset) {
				t.Fatalf("edgeGeomOffset mismatch: want %v, got %v", tc.edgeGeomOffset, ann.GetEdgeGeomOffset())
			}

			if len(tc.duration) > 0 {
				tc.duration[0] = -999
				if ann.GetDuration()[0] == -999 {
					t.Fatalf("duration was not deep-copied")
				}
			}
			if len(tc.distance) > 0 {
				tc.distance[0] = -999
				if ann.GetDistance()[0] == -999 {
					t.Fatalf("distance was not deep-copied")
				}
			}
			if len(tc.geometry) > 0 {
				tc.geometry[0] = Coordinate{Lat: 999, Lon: 999}
				if ann.GetGeometry()[0] == (Coordinate{Lat: 999, Lon: 999}) {
					t.Fatalf("geometry was not deep-copied")
				}
			}
			if len(tc.edgeGeomOffset) > 0 {
				tc.edgeGeomOffset[0] = 999
				if ann.GetEdgeGeomOffset()[0] == 999 {
					t.Fatalf("edgeGeomOffset was not deep-copied")
				}
			}
		})
	}
}

func TestNewInstructionCopiesEdgeIDs(t *testing.T) {
	testCases := []struct {
		name        string
		constructor func(edgeIDs []Index) []Index
	}{
		{
			name: "new_instruction",
			constructor: func(edgeIDs []Index) []Index {
				ins := NewInstruction(
					TURN_LEFT,
					"Jalan Test",
					NewCoordinate(-7.8, 112.7),
					false,
					edgeIDs,
					120.0,
					3.5,
					[]Coordinate{{Lat: -7.8, Lon: 112.7}},
					90,
					NewAnnotation(nil, nil, nil, nil),
					true,
				)
				return ins.GetEdgeIds()
			},
		},
		{
			name: "new_instruction_with_roundabout",
			constructor: func(edgeIDs []Index) []Index {
				ins := NewInstructionWithRoundabout(
					USE_ROUNDABOUT,
					"Bundaran Test",
					NewCoordinate(-7.8, 112.7),
					true,
					NewRoundaboutInstruction(),
					120.0,
					3.5,
					edgeIDs,
					NewAnnotation(nil, nil, nil, nil),
					180,
				)
				return ins.GetEdgeIds()
			},
		},
	}

	for _, tc := range testCases {
		tc := tc
		t.Run(tc.name, func(t *testing.T) {
			original := []Index{1, 2, 3}
			gotEdgeIDs := tc.constructor(original)

			if !reflect.DeepEqual(gotEdgeIDs, original) {
				t.Fatalf("edge ids mismatch: want %v, got %v", original, gotEdgeIDs)
			}

			original[0] = 999
			if gotEdgeIDs[0] == 999 {
				t.Fatalf("edge ids were not copied")
			}
		})
	}
}

func TestBearingToCompass(t *testing.T) {
	testCases := []struct {
		name    string
		bearing float64
		want    string
	}{
		{name: "north_lower_bound", bearing: 0, want: "North"},
		{name: "north_east", bearing: 45, want: "North East"},
		{name: "east", bearing: 90, want: "East"},
		{name: "south_east", bearing: 135, want: "South East"},
		{name: "south", bearing: 180, want: "South"},
		{name: "south_west", bearing: 225, want: "South West"},
		{name: "west", bearing: 270, want: "West"},
		{name: "north_west", bearing: 315, want: "North West"},
		{name: "north_upper_bound", bearing: 359.9, want: "North"},
	}

	for _, tc := range testCases {
		tc := tc
		t.Run(tc.name, func(t *testing.T) {
			got := bearingToCompass(tc.bearing)
			if got != tc.want {
				t.Fatalf("bearingToCompass(%v): want %q, got %q", tc.bearing, tc.want, got)
			}
		})
	}
}
