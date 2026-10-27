package partitioner

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestRandomizedSelect(t *testing.T) {
	testCases := []struct {
		name  string
		input []int
		rank  int
		want  int
	}{{
		name:  "[10, 8, 3, 100, 6, 25, 4]. rank=3, expected=6",
		input: []int{10, 8, 3, 100, 6, 25, 4},
		rank:  3, // 1-indexed
		want:  6,
	},
		{
			name:  "[99, 101, 234, -1220, -200, -67, 67, 81, 98]. rank=5, expected=81",
			input: []int{99, 101, 234, -1220, -200, -67, 67, 81, 98},
			rank:  5,
			want:  81,
		},

		{
			name:  "[99, 101, 234, -1220, -200, -67, 67, 81, 98]. rank=5, expected=81",
			input: []int{99, 101, 234, -1220, -200, -67, 67, 81, 98},
			rank:  8,
			want:  101,
		},
	}

	for _, tc := range testCases {
		n := len(tc.input)
		nums := make([]int, n)
		for i := 0; i < n; i++ {
			v := tc.input[i]
			nums[i] = v
		}
		q := randomizedSelect(nums, 0, n-1, tc.rank, func(left, right int) bool {
			return nums[left] < nums[right]
		})
		got := nums[q]
		assert.Equal(t, tc.want, got)
	}
}
