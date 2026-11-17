package util

import "testing"

func TestDivRound(t *testing.T) {
	testCases := []struct {
		name string
		a    int32
		b    int32
		want int32
	}{
		{
			name: "32/5",
			a:    32,
			b:    5,
			want: 6,
		},

		{
			name: "33/5",
			a:    33,
			b:    5,
			want: 7,
		},
		{
			name: "-56/5",
			a:    -56,
			b:    5,
			want: -11,
		},

		{
			name: "-58/5",
			a:    -58,
			b:    5,
			want: -12,
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			got := DivRound(tc.a, tc.b)
			if tc.want != got {
				t.Errorf("got: %v, want: %v", got, tc.want)
			}
		})
	}
}
