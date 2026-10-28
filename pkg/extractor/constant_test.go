package extractor

import (
	"testing"
)

func TestParseTurnRestriction(t *testing.T) {
	testCases := []struct {
		input string
		want  TurnRestriction
	}{
		{"no_left_turn", NoLeftTurn},
		{"no_right_turn", NoRightTurn},
		{"no_straight_on", NoStraightOn},
		{"no_u_turn", NoUTurn},
		{"only_left_turn", OnlyLeftTurn},
		{"only_right_turn", OnlyRightTurn},
		{"only_straight_on", OnlyStraightOn},
		{"no_entry", NoEntry},
		{"no_exit", NoExit},
		{"invalid", INVALID},
		{"unknown", NONE},
		{"", NONE},
	}

	for _, tc := range testCases {
		got := parseTurnRestriction(tc.input)
		if got != tc.want {
			t.Errorf("parseTurnRestriction(%q) = %v, want %v", tc.input, got, tc.want)
		}
	}
}

func TestGetBarrierType(t *testing.T) {
	testCases := []struct {
		input string
		want  BarrierTypeT
	}{
		{"bollard", BOLLARD},
		{"swing_gate", SwingGate},
		{"jersey_barrier", JerseyBarrier},
		{"lift_gate", LiftGate},
		{"block", BLOCK},
		{strGate, GATE},
		{"removable", REMOVABLE},
		{"", NoBarrier},
		{"unknown", InvalidBarrier},
	}

	for _, tc := range testCases {
		got := getBarrierType(tc.input)
		if got != tc.want {
			t.Errorf("getBarrierType(%q) = %v, want %v", tc.input, got, tc.want)
		}
	}
}
