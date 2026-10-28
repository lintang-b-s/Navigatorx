package datastructure

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestQueue(t *testing.T) {
	testCases := []struct {
		name     string
		pushVals []int
		popCount int
		want     []int
	}{
		{
			name:     "push [1,2,3] pop 2 times. want [3]",
			pushVals: []int{1, 2, 3},
			popCount: 2,
			want:     []int{3},
		},
		{
			name:     "push [1,2,3,4] pop 2 times. want [3,4]",
			pushVals: []int{1, 2, 3, 4},
			popCount: 2,
			want:     []int{3, 4},
		},

		{
			name:     "push [1,2,3,4] pop 1 times. want [2,3,4]",
			pushVals: []int{1, 2, 3, 4},
			popCount: 1,
			want:     []int{2, 3, 4},
		},

		{
			name:     "push [1,2,3,4] pop 3 times. want [4]",
			pushVals: []int{1, 2, 3, 4},
			popCount: 3,
			want:     []int{4},
		},

		{
			name:     "push [1,2,3,4] pop 4 times. want []",
			pushVals: []int{1, 2, 3, 4},
			popCount: 4,
			want:     []int{},
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			n := len(tc.pushVals)
			qq := NewQueue[int](n)
			for _, v := range tc.pushVals {
				qq.Push(v)
			}
			for range tc.popCount {
				qq.Pop()
			}
			assert.Equal(t, tc.want, qq.Values())
		})
	}
}

func TestQueuePushsPopsPushs(t *testing.T) {
	testCases := []struct {
		name      string
		pushVals  []int
		popCount  int
		pushVals2 []int
		want      []int
	}{
		{
			name:      "push [1,2,3]. pop 2 times. push [4,5]. want [3,4,5]",
			pushVals:  []int{1, 2, 3},
			popCount:  2,
			pushVals2: []int{4, 5},
			want:      []int{3, 4, 5},
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			n := len(tc.pushVals)
			qq := NewQueue[int](n)
			for _, v := range tc.pushVals {
				qq.Push(v)
			}
			for range tc.popCount {
				qq.Pop()
			}
			for _, v := range tc.pushVals2 {
				qq.Push(v)
			}
			assert.Equal(t, tc.want, qq.Values())
		})
	}
}

func TestQueueFull(t *testing.T) {
	testCases := []struct {
		name     string
		cap      int
		pushVals []int
		wantFull bool
	}{
		{
			name:     "cap=3. push [1,2,3, 4]. wantFull true",
			pushVals: []int{1, 2, 3, 4},
			cap:      3,
			wantFull: true,
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {

			qq := NewQueue[int](tc.cap)
			gotFull := false
			for _, v := range tc.pushVals {
				gotFull = !qq.Push(v)
			}
			assert.Equal(t, tc.wantFull, gotFull)
		})
	}
}

func TestQueueEmpty(t *testing.T) {
	testCases := []struct {
		name      string
		pushVals  []int
		popCount  int
		wantEmpty bool
	}{
		{
			name:      " push [1,2,3,4]. pop 4 times. wantEmpty true",
			pushVals:  []int{1, 2, 3, 4},
			popCount:  4,
			wantEmpty: true,
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			n := len(tc.pushVals)
			qq := NewQueue[int](n)
			for _, v := range tc.pushVals {
				qq.Push(v)
			}
			gotEmpty := false
			for range tc.popCount + 1 {
				_, f := qq.Pop()
				gotEmpty = !f
			}

			assert.Equal(t, tc.wantEmpty, gotEmpty)
			gotEmpty = qq.IsEmpty()
			assert.Equal(t, tc.wantEmpty, gotEmpty)
		})
	}
}

func TestQueueClearReuse(t *testing.T) {
	testCases := []struct {
		name      string
		pushVals  []int
		pushVals2 []int
		want      []int
	}{
		{
			name:      "push [1,2,3,4]. reset(). push again [100, 120, 67, 666].",
			pushVals:  []int{1, 2, 3, 4},
			pushVals2: []int{100, 120, 67, 666},
			want:      []int{100, 120, 67, 666},
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			n := len(tc.pushVals)
			qq := NewQueue[int](n)
			for _, v := range tc.pushVals {
				qq.Push(v)
			}
			qq.Reset() // reset
			for _, v := range tc.pushVals2 {
				qq.Push(v)
			}
			assert.Equal(t, tc.want, qq.Values())
		})
	}
}
