package datastructure

import (
	"github.com/bits-and-blooms/bitset"
	"github.com/lintang-b-s/Navigatorx/pkg"
)

/*
TurnLanesData simpan values dari tiap turn lanes.
https://wiki.openstreetmap.org/wiki/Key:turn#Values


TurnLanesData disimpan di setiap road segment (edge).
		    e_b
		 |      |
		 | 	    |
  e_a    |      |
----------      ---------------

----------

----------       --------------
								  e_c
----------

----------      ---------------
		  |     |
		  |     |
		  |     |
		  |     |



e_a punya 4 lanes: left|left;through|through;right|right

lane pertama cuma boleh buat turn left.
lane kedua boleh turn left atau lurus terus.
lane ketiga  boleh turn right atau lurus terus.
lane keempat cuma boleh turn right.


tipeMask[i] menyimpan semua allowed turns di lane-i.
assume max allowed turns in one lane max 5, masing masing turns di lane ini harus beda.
tipe semua allowed lane turns ada di constant.go TurnLaneType.
worst case nya 11+10+9+8+7=45 bits untuk setiap tipeMask[i]..
kita harus pakai bitvector/bitset.. buat simpen allowed turns di lane-i (tipeMask[i]).
library bitset dynamic size bitset nya
*/ // nolint: gofmt
type TurnLanesData struct {
	tipeMask []*bitset.BitSet
}

// NewTurnLanesData. create new turn lanes data from turnLaneTypes.
// turnLaneTypes[i] berisi allowed turns di lane-i
// harus bikin bitset nya
func NewTurnLanesData(turnLaneTypes [][]pkg.TurnLaneType) TurnLanesData {
	n := len(turnLaneTypes)
	tipeMask := make([]*bitset.BitSet, n)
	for i := 0; i < n; i++ {
		tipeMask[i] = bitset.New(25)
	}

	for i := 0; i < n; i++ {
		for _, turnt := range turnLaneTypes[i] {
			tipeMask[i].Set(uint(turnt))
		}
	}

	return TurnLanesData{tipeMask: tipeMask}
}

// Valid. return valid if i-th lane has allowed turn type turnt
// can return false if the routing engine profile vehicle not allowed to go to this i-th lane.
func (tld TurnLanesData) Valid(i int, turnt pkg.TurnLaneType) bool {
	if tld.tipeMask[i].Test(uint(pkg.LANE_NO_ENTRY)) {
		// ini kalo vehicle dari router gak boleh pakai i-th lane.
		return false
	}
	return tld.tipeMask[i].Test(uint(turnt))
}

func NewEmptyTurnLanesData() TurnLanesData {
	tipeMask := make([]*bitset.BitSet, 0)

	return TurnLanesData{tipeMask: tipeMask}
}
