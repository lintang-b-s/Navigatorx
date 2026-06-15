package routing

import (
	"reflect"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

func TestRemoveDuplicates(t *testing.T) {
	cases := []struct {
		name string
		in   []da.Index
		want []da.Index
	}{
		{name: "empty", in: nil, want: nil},
		{name: "single", in: []da.Index{1}, want: []da.Index{1}},
		{name: "all equal", in: []da.Index{7, 7, 7, 7}, want: []da.Index{7}},
		{name: "already unique", in: []da.Index{1, 2, 3, 4, 5}, want: []da.Index{1, 2, 3, 4, 5}},
		{
			name: "preserves first occurrence order",
			in:   []da.Index{3, 1, 2, 1, 3, 2, 4},
			want: []da.Index{3, 1, 2, 4},
		},
		{
			name: "duplicates at start",
			in:   []da.Index{5, 5, 1, 1, 2, 3, 3},
			want: []da.Index{5, 1, 2, 3},
		},
		{
			name: "alternating duplicates",
			in:   []da.Index{1, 2, 1, 2, 1, 2},
			want: []da.Index{1, 2},
		},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := removeDuplicates(tc.in)
			if !reflect.DeepEqual(got, tc.want) {
				t.Fatalf("removeDuplicates(%v) = %v, want %v", tc.in, got, tc.want)
			}
		})
	}
}

func TestRemoveDuplicatesInPlace(t *testing.T) {
	in := []da.Index{3, 1, 2, 1, 3, 2, 4}
	got := removeDuplicates(in)
	if len(got) != 4 {
		t.Fatalf("expected length 4, got %d", len(got))
	}
	if &got[0] != &in[0] {
		t.Fatalf("removeDuplicates must reuse the input backing array")
	}
}
