package datastructure

import (
	"math/rand"
	"slices"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/stretchr/testify/assert"
)

func TestQueryHeap(t *testing.T) {
	util.ActivateMode[int32]()

	t.Run("random array query min-heap test ARRAY_STORAGE", func(t *testing.T) {
		rd := rand.New(rand.NewSource((time.Now().UnixNano())))

		numInputs := int32(1e5)
		pq := NewQueryHeap[Index, int32](Index(numInputs), Index(numInputs), ArrayStorageMode)
		// insert
		sortedArr := make([]int32, numInputs)
		for i := 0; i < int(numInputs); i++ {
			rdRank := rd.Int31n(numInputs)
			sortedArr[i] = rdRank
			pq.Insert(Index(i), int32(rdRank), NewVData(rdRank, Index(i), 0))
		}

		slices.Sort(sortedArr)
		// extract-min
		for i := 0; i < int(numInputs); i++ {
			_, got := pq.ExtractMin()
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
			pq.Insert(Index(i), rdRank, NewVData(rdRank, Index(i), 0))
		}

		// decrease-key
		sortedArr = make([]int32, numInputs)
		for i := int32(0); i < numInputs; i++ {
			oRank := ranks[i]
			nRank := rd.Int31n(oRank)
			pq.DecreaseKey(Index(i), nRank, nRank, 0)
			sortedArr[i] = nRank
		}

		slices.Sort(sortedArr)

		// extract-min for check decreased-key
		for i := 0; i < int(numInputs); i++ {
			_, got := pq.ExtractMin()
			want := sortedArr[i]
			if got != want {
				t.Fatalf("step 2 - index %d: want %v, got %v", i, want, got)
			}
		}
	})
}

func TestQueryHeapSimple(t *testing.T) {
	util.ActivateMode[int32]()

	testCases := []struct {
		name        string
		arr         []int32
		expectedArr []int32
	}{
		{
			name:        "insert arr=[50, 10, 90, 100, 5, 25, 99] and extract min len(arr) times",
			arr:         []int32{50, 10, 90, 100, 5, 25, 99},
			expectedArr: []int32{5, 10, 25, 50, 90, 99, 100},
		},

		{
			name:        "insert arr=[67, 76, 100, 2, 8] and extract min len(arr) times",
			arr:         []int32{67, 76, 100, 2, 8},
			expectedArr: []int32{2, 8, 67, 76, 100},
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			pq := NewQueryHeap[Index, int32](Index(len(tc.arr)), Index(len(tc.arr)), ArrayStorageMode)
			for i, v := range tc.arr {
				pq.Insert(Index(i), v, NewVData(v, Index(i), 0))
			}

			got := make([]int32, len(tc.arr))
			for i := 0; i < len(tc.arr); i++ {
				_, minCost := pq.ExtractMin()
				got[i] = minCost
			}

			assert.Equal(t, tc.expectedArr, got)
		})
	}
}

/*
BenchmarkMinHeap
go test -run=^$ -bench=^BenchmarkMinHeap$  -benchtime=15s  -benchmem -cpuprofile minheap.cpu -memprofile minheap.mem
---- optimized optHeapifyUp() and optHeapifyDown(): -----
Running tool: /usr/bin/go test -test.fullpath=true -benchmem -run=^$ -bench ^BenchmarkMinHeap$ github.com/lintang-b-s/Navigatorx/pkg/datastructure

goos: linux
goarch: amd64
pkg: github.com/lintang-b-s/Navigatorx/pkg/datastructure
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
BenchmarkMinHeap-12     26680800               649.2 ns/op            96 B/op          0 allocs/op
PASS
ok      github.com/lintang-b-s/Navigatorx/pkg/datastructure     18.230s

--- old & slow heapifyUp() and heapifyDown(): ----

running tool: /usr/bin/go test -test.fullpath=true -benchmem -run=^$ -bench ^BenchmarkMinHeap$ github.com/lintang-b-s/Navigatorx/pkg/datastructure

goos: linux
goarch: amd64
pkg: github.com/lintang-b-s/Navigatorx/pkg/datastructure
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
=== RUN   BenchmarkMinHeap
BenchmarkMinHeap
BenchmarkMinHeap-12      1057399              1065 ns/op             993 B/op          0 allocs/op
PASS
ok      github.com/lintang-b-s/Navigatorx/pkg/datastructure     3.991s
*/
func BenchmarkMinHeap(b *testing.B) {
	rd := rand.New(rand.NewSource((time.Now().UnixNano())))
	util.ActivateMode[int32]()
	numInputs := int32(30_000_000)
	pq := NewQueryHeap[Index, int32](Index(numInputs), Index(numInputs), ArrayStorageMode)
	// insert
	sortedArr := make([]int32, numInputs)
	for i := 0; i < int(numInputs); i++ {
		rdRank := rd.Int31n(numInputs)
		sortedArr[i] = rdRank
		pq.Insert(Index(i), rdRank, NewVData(rdRank, Index(i), 0))
	}

	for b.Loop() {
		rdRank := rd.Int31n(numInputs)
		pq.Insert(Index(rdRank), rdRank, NewVData(rdRank, Index(rdRank), 0))
		nrank := max(rdRank-1, 0)
		pq.DecreaseKey(Index(rdRank), nrank, nrank, 0)
		pq.ExtractMin()
	}
}
