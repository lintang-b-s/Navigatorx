package datastructure

import (
	"math/rand"
	"testing"
	"time"
)

// please run the test using command: "cd ./pkg/datastructure  && go test -run TestPackedSlice  -v -timeout=0  -count=1"
func TestPackedSlice(t *testing.T) {
	testCases := []struct {
		name         string
		input        []uint64
		expectedData []uint64
	}{{
		name:         "packed slice append, get test dari example https://github.com/Telenav/open-source-spec/blob/master/osrm/doc/packed_vector.md",
		input:        []uint64{1597322404, 1432114613, 1939964443, 2112255763},
		expectedData: []uint64{12301770855514715300, 17236323169611417708, 3},
	}}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			ps := NewPackedSlice(33, 4)
			for _, val := range tc.input {
				ps.Append(val)
			}

			for i, exp := range tc.input {
				got := ps.Get(uint64(i))
				if got != exp {
					t.Errorf("expected: %v, got: %v", exp, got)
				}
			}

			for i, exp := range tc.expectedData {
				if ps.data[i] != exp {
					t.Errorf("expected: %v, got: %v", exp, ps.data[i])
				}
			}
		})
	}

	// generate random test cases
	t.Run("random input packed slice test", func(t *testing.T) {

		rd := rand.New(rand.NewSource(time.Now().UnixNano()))

		maxValUint34 := 2 ^ 34 - 1
		const numInputs = int(1e8)

		ps := NewPackedSlice(BIT_SIZE_OSM_NODE_ID, uint64(numInputs))

		randomInput := make([]uint64, numInputs)
		for i := 0; i < numInputs; i++ {
			randVal := uint64(rd.Int63n(int64(maxValUint34)))
			randomInput[i] = randVal
			ps.Append(randVal)
		}
		for i := 0; i < numInputs; i++ {
			got := ps.Get(uint64(i))
			expected := randomInput[i]
			if got != expected {
				t.Errorf("expected: %v, got: %v", expected, got)
			}
		}
	})
}
