package customizer

import (
	"math/rand"
	"testing"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

func TestLookupTable(t *testing.T) {
	t.Run("test lookup table", func(t *testing.T) {
		rd := rand.New(rand.NewSource(time.Now().UnixNano()))

		const maxValUint34 = (uint64(1) << 34) - 1
		const numInputs = int(2e6)

		ps := da.NewPackedSlice(34, uint64(numInputs))

		set := make(map[uint64]struct{})

		randomInput := make([]uint64, numInputs)
		for i := 0; i < numInputs; {
			randVal := uint64(rd.Int63n(int64(maxValUint34)))
			_, ok := set[randVal]
			if !ok {
				randomInput[i] = randVal
				ps.Append(randVal)
				
				set[randVal] = struct{}{}
				i++
			}
		}

		vOsmNodeIdLookupTable := NewLookupTable[uint64](randomInput, func(a, b uint64) bool {
			return a < b
		})

		for expectedVId := 0; expectedVId < numInputs; expectedVId++ {
			ithOsmNodeId := randomInput[expectedVId]
			gotVId := vOsmNodeIdLookupTable.Get(ithOsmNodeId)
			if gotVId != expectedVId {
				t.Errorf("want: %v, got: %v", expectedVId, gotVId)
			}
		}
	})
}
