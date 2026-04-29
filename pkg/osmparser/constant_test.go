package osmparser

import (
	"testing"
)

func TestParseTurnRestriction(t *testing.T) {
	testCases := []struct {
		input string
		want  TurnRestriction
	}{
		{"no_left_turn", NO_LEFT_TURN},
		{"no_right_turn", NO_RIGHT_TURN},
		{"no_straight_on", NO_STRAIGHT_ON},
		{"no_u_turn", NO_U_TURN},
		{"only_left_turn", ONLY_LEFT_TURN},
		{"only_right_turn", ONLY_RIGHT_TURN},
		{"only_straight_on", ONLY_STRAIGHT_ON},
		{"no_entry", NO_ENTRY},
		{"no_exit", NO_EXIT},
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
		{"swing_gate", SWING_GATE},
		{"jersey_barrier", JERSEY_BARRIER},
		{"lift_gate", LIFT_GATE},
		{"block", BLOCK},
		{"gate", GATE},
		{"removable", REMOVABLE},
		{"", NO_BARRIER},
		{"unknown", INVALID_BARRIER},
	}

	for _, tc := range testCases {
		got := getBarrierType(tc.input)
		if got != tc.want {
			t.Errorf("getBarrierType(%q) = %v, want %v", tc.input, got, tc.want)
		}
	}
}
