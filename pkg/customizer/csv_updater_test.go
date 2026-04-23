package customizer

import (
	"math/rand"
	"sort"
	"testing"
	"time"
)

func TestLookupTable(t *testing.T) {
	t.Run("test lookup table", func(t *testing.T) {
		rd := rand.New(rand.NewSource(time.Now().UnixNano()))

		const maxValUint34 = (int(1) << 34) - 1
		const numInputs = int(2e6)

		set := make(map[int]struct{})

		randomInput := make([]int, numInputs)
		for i := 0; i < numInputs; {
			randVal := int(rd.Int63n(int64(maxValUint34)))
			_, ok := set[randVal]
			if !ok {
				randomInput[i] = randVal

				set[randVal] = struct{}{}
				i++
			}
		}

		vOsmNodeIdLookupTable := NewLookupTable[int](randomInput, func(a, b int) bool {
			return a < b
		})

		if !sort.IntsAreSorted(vOsmNodeIdLookupTable.data) {
			t.Errorf("expected lookupTable.data sorted, got not sorted")
		}

		for expectedVId := 0; expectedVId < numInputs; expectedVId++ {
			ithOsmNodeId := randomInput[expectedVId]
			gotVId := vOsmNodeIdLookupTable.Get(ithOsmNodeId)
			if gotVId != expectedVId {
				t.Errorf("want: %v, got: %v", expectedVId, gotVId)
			}
		}
	})
}
