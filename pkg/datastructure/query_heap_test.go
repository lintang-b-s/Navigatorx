package datastructure

import (
	"math/rand"
	"sort"
	"testing"
	"time"
)

func TestQueryHeap(t *testing.T) {
	t.Run("random array query heap test ARRAY_STORAGE", func(t *testing.T) {
		rd := rand.New(rand.NewSource((time.Now().UnixNano())))

		numInputs := int(1e6)
		pq := NewQueryHeap[int](uint32(numInputs), uint32(numInputs), ARRAY_STORAGE, true)

		sortedArr := make([]int, numInputs)
		for i := 0; i < numInputs; i++ {
			rdNum := rd.Intn(numInputs)
			sortedArr[i] = rdNum
			pq.Insert(Index(rdNum), float64(rdNum), NewVertexInfo(0, NewVertexEdgePair(0, 0, false)), rdNum)
		}

		sort.Ints(sortedArr)

		for i := 0; i < numInputs; i++ {
			numNode := pq.ExtractMin()
			got := numNode.GetItem()
			want := sortedArr[i]
			if got != want {
				t.Errorf("want: %v, got: %v", want, got)
			}
		}
	})
}
