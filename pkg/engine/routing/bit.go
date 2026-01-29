package routing

import "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

func onOverlayBit(v datastructure.Index) datastructure.Index {
	return onBit(v, OVERLAY_OFFSET)
}

func offOverlayBit(v datastructure.Index) datastructure.Index {
	return offBit(v, OVERLAY_OFFSET)
}

func isOverlay(u datastructure.Index) bool {
	return isBitOn(u, OVERLAY_OFFSET)
}

func offsetForward(uEntryOffset datastructure.Index, uCellNumber, sCellNumber datastructure.Pv) datastructure.Index {
	if uCellNumber == sCellNumber {

		return onBit(uEntryOffset, FORWARD_S_OFFSET_BIT)
	} else {

		return onBit(uEntryOffset, FORWARD_T_OFFSET_BIT)
	}
}

func offsetBackward(uExitOffset datastructure.Index, uCellNumber, sCellNumber datastructure.Pv) datastructure.Index {
	if uCellNumber == sCellNumber {
		return onBit(uExitOffset, BACKWARD_S_OFFSET_BIT)
	} else {
		return onBit(uExitOffset, BACKWARD_T_OFFSET_BIT)
	}
}
func offsetForwardOverlay(u *datastructure.OverlayVertex, uEntryOffset datastructure.Index,
	sCellNumber datastructure.Pv) datastructure.Index {
	if u.GetCellNumber() == sCellNumber {
		return onBit(uEntryOffset, FORWARD_S_OFFSET_BIT)
	} else {
		return onBit(uEntryOffset, FORWARD_T_OFFSET_BIT)
	}
}

func offsetBackwardOverlay(u *datastructure.OverlayVertex, uExitOffset datastructure.Index,
	sCellNumber datastructure.Pv) datastructure.Index {
	if u.GetCellNumber() == sCellNumber {
		return onBit(uExitOffset, BACKWARD_S_OFFSET_BIT)
	} else {
		return onBit(uExitOffset, BACKWARD_T_OFFSET_BIT)

	}
}

func adjustForwardOffBit(uEntryOffset datastructure.Index) datastructure.Index {
	if isBitOn(uEntryOffset, FORWARD_S_OFFSET_BIT) {
		return offBit(uEntryOffset, FORWARD_S_OFFSET_BIT)
	} else {
		return offBit(uEntryOffset, FORWARD_T_OFFSET_BIT)
	}
}

func adjustBackwardOffbit(uExitOffset datastructure.Index) datastructure.Index {
	if isBitOn(uExitOffset, BACKWARD_S_OFFSET_BIT) {
		return offBit(uExitOffset, BACKWARD_S_OFFSET_BIT)
	} else {
		return offBit(uExitOffset, BACKWARD_T_OFFSET_BIT)
	}
}

func adjustForward(uEntryOffset datastructure.Index, uStartEntryOffset datastructure.Index) datastructure.Index {
	if isBitOn(uEntryOffset, FORWARD_S_OFFSET_BIT) {
		return offBit(uEntryOffset, FORWARD_S_OFFSET_BIT) - uStartEntryOffset
	} else {
		return offBit(uEntryOffset, FORWARD_T_OFFSET_BIT) - uStartEntryOffset

	}
}

func adjustBackward(uExitOffset datastructure.Index, uStartExitOffset datastructure.Index) datastructure.Index {
	if isBitOn(uExitOffset, BACKWARD_S_OFFSET_BIT) {
		return offBit(uExitOffset, BACKWARD_S_OFFSET_BIT) - uStartExitOffset
	} else {
		return offBit(uExitOffset, BACKWARD_T_OFFSET_BIT) - uStartExitOffset
	}
}

func adjustOverlay(e datastructure.Index) datastructure.Index {
	return onBit(offBit(e, OVERLAY_OFFSET), UNPACK_OVERLAY_OFFSET)
}

func isBitOn(u datastructure.Index, i int) bool {
	if u&datastructure.Index(uint32(1)<<i) != 0 {
		return true
	}

	return false
}

func offBit(u datastructure.Index, i int) datastructure.Index {
	return u & ^datastructure.Index(uint32(1)<<i)
}

func onBit(u datastructure.Index, i int) datastructure.Index {
	return u | datastructure.Index(uint32(1)<<i)
}
