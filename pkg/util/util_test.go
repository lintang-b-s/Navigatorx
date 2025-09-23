package util

import (
	"encoding/binary"
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestBitPacking(t *testing.T) {

	var buf [8]byte
	bitpackedEdgeInfoField := int32(125)
	bitpackedEdgeInfoField = BitPackInt(bitpackedEdgeInfoField, int32(4), 8)
	bitpackedEdgeInfoField = BitPackInt(bitpackedEdgeInfoField, int32(8), 14)
	bitpackedEdgeInfoField = BitPackIntBool(bitpackedEdgeInfoField, false, 20)
	bitpackedEdgeInfoField = BitPackIntBool(bitpackedEdgeInfoField, false, 21)
	bitpackedEdgeInfoField = BitPackIntBool(bitpackedEdgeInfoField, true, 31)

	binary.LittleEndian.PutUint32(buf[4:8], uint32(bitpackedEdgeInfoField))

	bitpack, isShortcut := BitUnpackIntBool(int32(binary.LittleEndian.Uint32(buf[4:8])), 21)
	_ = bitpack
	if isShortcut {
		t.Errorf("Bitpack isShortcut: %t\n", isShortcut)
	}

	bitpack, isRoundabout := BitUnpackIntBool(int32(binary.LittleEndian.Uint32(buf[4:8])), 20)
	_ = bitpack
	if isRoundabout {
		t.Errorf("Bitpack isRoundabout: %t\n", isRoundabout)
	}

	_, trafficLight := BitUnpackIntBool(int32(binary.LittleEndian.Uint32(buf[4:8])), 31)
	if !trafficLight {
		t.Errorf("Bitpack trafficLight: %t\n", trafficLight)
	}
}

func TestSort(t *testing.T) {
	cellNumbers := []int{10, 3, 8, 2, 1}
	newToOldPosition := []int{0, 1, 2, 3, 4}

	sortedCellNumbers := QuickSortG(newToOldPosition, func(jVal, pivotVal int) bool {
		cellNumberA := cellNumbers[jVal]
		cellNumberB := cellNumbers[pivotVal]
		return cellNumberA < cellNumberB
	})

	assert.Equal(t, []int{4, 3, 1, 2, 0}, sortedCellNumbers)

	QuickSortGIdx(newToOldPosition, func(j, pivotIdx int) bool {
		cellNumberA := cellNumbers[newToOldPosition[j]]
		cellNumberB := cellNumbers[newToOldPosition[pivotIdx]]
		return cellNumberA < cellNumberB
	})

	assert.Equal(t, []int{4, 3, 1, 2, 0}, newToOldPosition)

	for i := 0; i < 100; i++ {
		arr := make([]int, 10000)
		for j := 0; j < 10000; j++ {
			n := generateRandomInt(1, 1000000)
			arr[j] = n
		}
		QuickSortIdx(arr, 0, len(arr)-1, func(j, pivotIdx int) bool {
			return arr[j] < arr[pivotIdx]
		})

		for k := 1; k < len(arr); k++ {
			if arr[k] < arr[k-1] {
				t.Errorf("array not sorted at index %d: %v\n", k, arr)
			}
		}
	}
}
