package datastructure

import (
	"slices"
	"testing"

	"github.com/google/go-cmp/cmp"
)

func TestLookupTable_SegmentKV(t *testing.T) {
	t.Run("NewLookupTable", func(t *testing.T) {
		tests := []struct {
			name string
			data []*SegmentKV
			want []*SegmentKV
		}{
			{
				name: "nil_slice",
				data: nil,
				want: nil,
			},
			{
				name: "single_element",
				data: []*SegmentKV{
					NewSegmentKV(10, 2, 0),
				},
				want: []*SegmentKV{
					NewSegmentKV(10, 2, 0),
				},
			},
			{
				name: "already_sorted",
				data: []*SegmentKV{
					NewSegmentKV(10, 2, 0),
					NewSegmentKV(10, 5, 1),
					NewSegmentKV(20, 2, 2),
					NewSegmentKV(30, 1, 3),
				},
				want: []*SegmentKV{
					NewSegmentKV(10, 2, 0),
					NewSegmentKV(10, 5, 1),
					NewSegmentKV(20, 2, 2),
					NewSegmentKV(30, 1, 3),
				},
			},
			{
				name: "unsorted_data",
				data: []*SegmentKV{
					NewSegmentKV(30, 1, 3),
					NewSegmentKV(10, 5, 1),
					NewSegmentKV(20, 2, 2),
					NewSegmentKV(10, 2, 0),
				},
				want: []*SegmentKV{
					NewSegmentKV(10, 2, 0),
					NewSegmentKV(10, 5, 1),
					NewSegmentKV(20, 2, 2),
					NewSegmentKV(30, 1, 3),
				},
			},
			{
				name: "reverse_sorted",
				data: []*SegmentKV{
					NewSegmentKV(30, 1, 3),
					NewSegmentKV(20, 2, 2),
					NewSegmentKV(10, 5, 1),
					NewSegmentKV(10, 2, 0),
				},
				want: []*SegmentKV{
					NewSegmentKV(10, 2, 0),
					NewSegmentKV(10, 5, 1),
					NewSegmentKV(20, 2, 2),
					NewSegmentKV(30, 1, 3),
				},
			},
		}

		for _, tc := range tests {
			t.Run(tc.name, func(t *testing.T) {
				lt := NewLookupTable(slices.Clone(tc.data))
				if diff := cmp.Diff(tc.want, lt.data, cmp.AllowUnexported(SegmentKV{}, KeyVal{})); diff != "" {
					t.Errorf("NewLookupTable(%+v) returned diff (-want +got):\n%s", tc.data, diff)
				}
			})
		}
	})

	t.Run("Get", func(t *testing.T) {
		singleData := []*SegmentKV{
			NewSegmentKV(10, 2, 42),
		}
		multiData := []*SegmentKV{
			NewSegmentKV(10, 2, 0),
			NewSegmentKV(10, 5, 1),
			NewSegmentKV(20, 2, 2),
			NewSegmentKV(30, 1, 3),
		}

		tests := []struct {
			name  string
			data  []*SegmentKV
			query *SegmentKV
			want  Index
		}{
			{
				name:  "empty_table",
				data:  nil,
				query: NewSegmentKV(10, 2, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "single_element_hit",
				data:  singleData,
				query: NewSegmentKV(10, 2, 0),
				want:  42,
			},
			{
				name:  "single_element_miss_smaller",
				data:  singleData,
				query: NewSegmentKV(5, 2, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "single_element_miss_larger",
				data:  singleData,
				query: NewSegmentKV(15, 2, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "multi_first_element",
				data:  multiData,
				query: NewSegmentKV(10, 2, 0),
				want:  0,
			},
			{
				name:  "multi_middle_element_one",
				data:  multiData,
				query: NewSegmentKV(10, 5, 0),
				want:  1,
			},
			{
				name:  "multi_middle_element_two",
				data:  multiData,
				query: NewSegmentKV(20, 2, 0),
				want:  2,
			},
			{
				name:  "multi_last_element",
				data:  multiData,
				query: NewSegmentKV(30, 1, 0),
				want:  3,
			},
			{
				name:  "multi_miss_smaller_than_min",
				data:  multiData,
				query: NewSegmentKV(5, 1, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "multi_miss_larger_than_max",
				data:  multiData,
				query: NewSegmentKV(40, 1, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "multi_miss_between_keys",
				data:  multiData,
				query: NewSegmentKV(15, 1, 0),
				want:  INVALID_LK_TABLE_ID,
			},
		}

		for _, tc := range tests {
			t.Run(tc.name, func(t *testing.T) {
				lt := NewLookupTable(slices.Clone(tc.data))
				got := lt.Get(tc.query)
				if got != tc.want {
					t.Errorf("lt.Get(%+v) = %v, want %v (table data: %+v)", tc.query, got, tc.want, tc.data)
				}
			})
		}
	})
}

func TestLookupTable_TurnKV(t *testing.T) {
	t.Run("NewLookupTable", func(t *testing.T) {
		tests := []struct {
			name string
			data []*TurnKV
			want []*TurnKV
		}{
			{
				name: "nil_slice",
				data: nil,
				want: nil,
			},
			{
				name: "single_element",
				data: []*TurnKV{
					NewTurnKV(1, 2, 1, 0),
				},
				want: []*TurnKV{
					NewTurnKV(1, 2, 1, 0),
				},
			},
			{
				name: "already_sorted",
				data: []*TurnKV{
					NewTurnKV(1, 2, 1, 0),
					NewTurnKV(1, 2, 2, 1),
					NewTurnKV(1, 3, 1, 2),
					NewTurnKV(2, 1, 1, 3),
				},
				want: []*TurnKV{
					NewTurnKV(1, 2, 1, 0),
					NewTurnKV(1, 2, 2, 1),
					NewTurnKV(1, 3, 1, 2),
					NewTurnKV(2, 1, 1, 3),
				},
			},
			{
				name: "unsorted_data",
				data: []*TurnKV{
					NewTurnKV(2, 1, 1, 3),
					NewTurnKV(1, 3, 1, 2),
					NewTurnKV(1, 2, 2, 1),
					NewTurnKV(1, 2, 1, 0),
				},
				want: []*TurnKV{
					NewTurnKV(1, 2, 1, 0),
					NewTurnKV(1, 2, 2, 1),
					NewTurnKV(1, 3, 1, 2),
					NewTurnKV(2, 1, 1, 3),
				},
			},
			{
				name: "reverse_sorted",
				data: []*TurnKV{
					NewTurnKV(2, 1, 1, 3),
					NewTurnKV(1, 3, 1, 2),
					NewTurnKV(1, 2, 2, 1),
					NewTurnKV(1, 2, 1, 0),
				},
				want: []*TurnKV{
					NewTurnKV(1, 2, 1, 0),
					NewTurnKV(1, 2, 2, 1),
					NewTurnKV(1, 3, 1, 2),
					NewTurnKV(2, 1, 1, 3),
				},
			},
		}

		for _, tc := range tests {
			t.Run(tc.name, func(t *testing.T) {
				lt := NewLookupTable(slices.Clone(tc.data))
				if diff := cmp.Diff(tc.want, lt.data, cmp.AllowUnexported(TurnKV{}, KeyVal{})); diff != "" {
					t.Errorf("NewLookupTable(%+v) returned diff (-want +got):\n%s", tc.data, diff)
				}
			})
		}
	})

	t.Run("Get", func(t *testing.T) {
		singleData := []*TurnKV{
			NewTurnKV(1, 2, 1, 42),
		}
		multiData := []*TurnKV{
			NewTurnKV(1, 2, 1, 0),
			NewTurnKV(1, 2, 2, 1),
			NewTurnKV(1, 3, 1, 2),
			NewTurnKV(2, 1, 1, 3),
		}

		tests := []struct {
			name  string
			data  []*TurnKV
			query *TurnKV
			want  Index
		}{
			{
				name:  "empty_table",
				data:  nil,
				query: NewTurnKV(1, 2, 1, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "single_element_hit",
				data:  singleData,
				query: NewTurnKV(1, 2, 1, 0),
				want:  42,
			},
			{
				name:  "single_element_miss_smaller",
				data:  singleData,
				query: NewTurnKV(0, 2, 1, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "single_element_miss_larger",
				data:  singleData,
				query: NewTurnKV(2, 2, 1, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "multi_first_element",
				data:  multiData,
				query: NewTurnKV(1, 2, 1, 0),
				want:  0,
			},
			{
				name:  "multi_middle_element_one",
				data:  multiData,
				query: NewTurnKV(1, 2, 2, 0),
				want:  1,
			},
			{
				name:  "multi_middle_element_two",
				data:  multiData,
				query: NewTurnKV(1, 3, 1, 0),
				want:  2,
			},
			{
				name:  "multi_last_element",
				data:  multiData,
				query: NewTurnKV(2, 1, 1, 0),
				want:  3,
			},
			{
				name:  "multi_miss_u_smaller_than_min",
				data:  multiData,
				query: NewTurnKV(0, 2, 1, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "multi_miss_u_larger_than_max",
				data:  multiData,
				query: NewTurnKV(3, 1, 1, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "multi_miss_matching_u_smaller_v",
				data:  multiData,
				query: NewTurnKV(1, 1, 1, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "multi_miss_matching_u_larger_v",
				data:  multiData,
				query: NewTurnKV(1, 4, 1, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "multi_miss_matching_uv_smaller_w",
				data:  multiData,
				query: NewTurnKV(1, 2, 0, 0),
				want:  INVALID_LK_TABLE_ID,
			},
			{
				name:  "multi_miss_matching_uv_larger_w",
				data:  multiData,
				query: NewTurnKV(1, 2, 3, 0),
				want:  INVALID_LK_TABLE_ID,
			},
		}

		for _, tc := range tests {
			t.Run(tc.name, func(t *testing.T) {
				lt := NewLookupTable(slices.Clone(tc.data))
				got := lt.Get(tc.query)
				if got != tc.want {
					t.Errorf("lt.Get(%+v) = %v, want %v (table data: %+v)", tc.query, got, tc.want, tc.data)
				}
			})
		}
	})
}
