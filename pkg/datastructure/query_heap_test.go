package datastructure

import (
	"math/rand"
	"slices"
	"testing"
	"time"
)

func TestQueryHeap(t *testing.T) {
	t.Run("random array query min-heap test ARRAY_STORAGE", func(t *testing.T) {
		rd := rand.New(rand.NewSource((time.Now().UnixNano())))

		numInputs := int32(1e5)
		pq := NewQueryHeap[int32, int32](Index(numInputs), ArrayStorageMode)
		// insert
		sortedArr := make([]int32, numInputs)
		for i := 0; i < int(numInputs); i++ {
			rdNum := rd.Int31n(numInputs)
			rdRank := rdNum
			sortedArr[i] = rdRank
			pq.Insert(Index(rdNum), int32(rdRank), NewVData(rdRank, NewParentVertex(0, false)), rdNum)
		}

		slices.Sort(sortedArr)
		// extract-min
		for i := 0; i < int(numInputs); i++ {
			node := pq.ExtractMin()
			got := node.GetRank()
			want := sortedArr[i]
			if got != want {
				t.Errorf("want: %v, got: %v", want, got)
			}
		}

		// insert again
		smallest := int32(10)
		ranks := make([]int32, numInputs)
		for i := int32(0); i < numInputs; i++ {
			rdRank := rd.Int31n(numInputs) + smallest
			ranks[i] = rdRank
			pq.Insert(Index(i), rdRank, NewVData(rdRank, NewParentVertex(0, false)), i)
		}

		// decrease-key
		sortedArr = make([]int32, numInputs)
		for i := int32(0); i < numInputs; i++ {
			oRank := ranks[i]
			nRank := rd.Int31n(oRank)
			pq.DecreaseKey(Index(i), nRank, nRank, NewParentVertex(0, false))
			sortedArr[i] = nRank
		}

		slices.Sort(sortedArr)

		// extract-min for check decreased-key
		for i := 0; i < int(numInputs); i++ {
			node := pq.ExtractMin()
			got := node.GetRank()
			want := sortedArr[i]
			if got != want {
				t.Fatalf("step 2 - index %d: want %v, got %v", i, want, got)
			}
		}
	})
}
