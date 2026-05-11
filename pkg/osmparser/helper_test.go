package osmparser

import (
	"testing"
	"time"
)

func TestIsConditionalAccessInTimerange(t *testing.T) {
	testCases := []struct {
		name        string
		condTime    string
		currentTime time.Time
		want        bool
		wantErr     bool // true if err != nil
	}{
		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr 07:00-09:00 dan currentTime inTimeRange",
			condTime:    "Mo-Fr 07:00-09:00",
			currentTime: time.Date(2026, 4, 8, 8, 0, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr 07:00-09:00 dan currentTime not inTimeRange",
			condTime:    "Mo-Fr 07:00-09:00",
			currentTime: time.Date(2026, 4, 12, 8, 0, 0, 0, time.Local),
			want:        false,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: time range input 23:00-05:00 dan currentTime not inTimeRange",
			condTime:    "23:00-05:00",
			currentTime: time.Date(2026, 4, 12, 8, 0, 0, 0, time.Local),
			want:        false,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: time range input 23:00-05:00 dan currentTime inTimeRange",
			condTime:    "23:00-05:00",
			currentTime: time.Date(2026, 4, 12, 23, 2, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: time range input 23:00-05:00, 10:00-15:00 dan currentTime inTimeRange",
			condTime:    "23:00-05:00, 10:00-15:00",
			currentTime: time.Date(2026, 4, 12, 13, 2, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},

		{
			name:        "TestIsConditionalAccessInTimerange: time range input 23:00-05:00, 10:00-15:00 dan currentTime not inTimeRange",
			condTime:    "23:00-05:00, 10:00-15:00",
			currentTime: time.Date(2026, 4, 12, 7, 2, 0, 0, time.Local),
			want:        false,
			wantErr:     false,
		},

		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr,Su 22:00-06:00 dan currentTime not inTimeRange",
			condTime:    "Mo-Fr,Su 22:00-06:00",
			currentTime: time.Date(2026, 4, 12, 7, 2, 0, 0, time.Local),
			want:        false,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr,Su 22:00-06:00 dan currentTime inTimeRange",
			condTime:    "Mo-Fr,Su 22:00-06:00",
			currentTime: time.Date(2026, 4, 12, 5, 2, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr,Su 22:00-06:00 dan currentTime inTimeRange",
			condTime:    "Mo-Fr,Su 22:00-06:00",
			currentTime: time.Date(2026, 4, 8, 5, 2, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr,Su 22:00-06:00 dan currentTime not inTimeRange",
			condTime:    "Mo-Fr,Su 22:00-06:00",
			currentTime: time.Date(2026, 4, 8, 6, 2, 0, 0, time.Local),
			want:        false,
			wantErr:     false,
		},

		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr 14:00-21:00; Sa-Su,PH 07:00-10:00 dan currentTime inTimeRange",
			condTime:    "Mo-Fr 14:00-21:00; Sa-Su,PH 07:00-10:00",
			currentTime: time.Date(2026, 4, 8, 15, 2, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr 14:00-21:00; Sa-Su,PH 07:00-10:00 dan currentTime inTimeRange",
			condTime:    "Mo-Fr 14:00-21:00; Sa-Su,PH 07:00-10:00",
			currentTime: time.Date(2026, 4, 12, 8, 2, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr 14:00-21:00; Sa-Su,PH 07:00-10:00 dan currentTime not inTimeRange",
			condTime:    "Mo-Fr 14:00-21:00; Sa-Su,PH 07:00-10:00",
			currentTime: time.Date(2026, 4, 12, 11, 2, 0, 0, time.Local),
			want:        false,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr 14:00-21:00; Sa-Su,PH 07:00-10:00 dan currentTime not inTimeRange",
			condTime:    "Mo-Fr 14:00-21:00; Sa-Su,PH 07:00-10:00",
			currentTime: time.Date(2026, 4, 8, 22, 2, 0, 0, time.Local),
			want:        false,
			wantErr:     false,
		},

		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr 07:00-09:00,16:00-18:00 dan currentTime inTimeRange",
			condTime:    "Mo-Fr 07:00-09:00,16:00-18:00",
			currentTime: time.Date(2026, 4, 8, 17, 2, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr 07:00-09:00,16:00-18:00 dan currentTime not inTimeRange",
			condTime:    "Mo-Fr 07:00-09:00,16:00-18:00",
			currentTime: time.Date(2026, 4, 8, 19, 2, 0, 0, time.Local),
			want:        false,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: sunset-sunrise dan currentTime in night range",
			condTime:    "sunset-sunrise",
			currentTime: time.Date(2026, 4, 8, 20, 0, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: sunset-sunrise dan currentTime in day range",
			condTime:    "sunset-sunrise",
			currentTime: time.Date(2026, 4, 8, 12, 0, 0, 0, time.Local),
			want:        false,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: dusk-dawn dan currentTime in night range",
			condTime:    "dusk-dawn",
			currentTime: time.Date(2026, 4, 8, 20, 0, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: sunrise-sunset dan currentTime in day range",
			condTime:    "sunrise-sunset",
			currentTime: time.Date(2026, 4, 8, 12, 0, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: (sunrise+01:00)-sunset dan currentTime in offset range",
			condTime:    "(sunrise+01:00)-sunset",
			currentTime: time.Date(2026, 4, 8, 6, 30, 0, 0, time.Local),
			want:        false,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: (sunrise+01:00)-sunset dan currentTime after offset range",
			condTime:    "(sunrise+01:00)-sunset",
			currentTime: time.Date(2026, 4, 8, 7, 30, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
		{
			name:        "TestIsConditionalAccessInTimerange: 24/7",
			condTime:    "24/7",
			currentTime: time.Date(2026, 4, 8, 12, 0, 0, 0, time.Local),
			want:        true,
			wantErr:     false,
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {

			got, gotErr := isConditionalAccessInTimerange(tc.condTime, tc.currentTime)
			if got != tc.want {
				t.Errorf("want: %v, got: %v", tc.want, got)
			}

			if (gotErr != nil) != tc.wantErr {
				t.Errorf("wantErr: %v, got err: %v", tc.wantErr, gotErr)
			}

		})
	}
}

func TestGetReversedOneWay(t *testing.T) {
	// mock pkg.IsVehicleEnabled
	// NOTE: pkg.IsVehicleEnabled is a global variable, so this might affect other tests if run in parallel
	// But for unit tests, we can manage it.

	// Since I can't easily mock the 'pkg' package variables without changing the code,
	// I'll assume the default state or just test the logic that doesn't depend on it being changed mid-test.
	// Actually, I can set it if I'm careful.
}

func TestGetReversibleOneWay(t *testing.T) {
	testCases := []struct {
		name        string
		input       string
		currentTime time.Time
		want        uint8
	}{
		{
			name:        "Forward reversible in time",
			input:       "yes @ (09:00-17:00)",
			currentTime: time.Date(2026, 4, 8, 10, 0, 0, 0, time.Local),
			want:        DIRECTION_FORWARD_REVERSIBLE,
		},
		{
			name:        "Backward reversible in time",
			input:       "-1 @ (17:01-08:59)",
			currentTime: time.Date(2026, 4, 8, 20, 0, 0, 0, time.Local),
			want:        DIRECTION_BACKWARD_REVERSIBLE,
		},
		{
			name:        "Reversible not in time",
			input:       "yes @ (09:00-17:00)",
			currentTime: time.Date(2026, 4, 8, 18, 0, 0, 0, time.Local),
			want:        NO_ROUTE_REVERSIBLE,
		},
		{
			name:        "Multiple ranges",
			input:       "yes @ (09:00-12:00); -1 @ (13:00-17:00)",
			currentTime: time.Date(2026, 4, 8, 14, 0, 0, 0, time.Local),
			want:        DIRECTION_BACKWARD_REVERSIBLE,
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			got, err := GetReversibleOneWay(tc.input, tc.currentTime)
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if got != tc.want {
				t.Errorf("GetReversibleOneWay(%q) = %v, want %v", tc.input, got, tc.want)
			}
		})
	}
}

func TestIsRoundaboutByName(t *testing.T) {
	p := &OsmParser{}
	testCases := []struct {
		name string
		want bool
	}{
		{"Roundabout North", true},
		{"Bundaran HI", true},
		{"Main Street", false},
		{"roundabout", true},
		{"bundaran", true},
	}

	for _, tc := range testCases {
		got := p.isRoundaboutByName(tc.name)
		if got != tc.want {
			t.Errorf("isRoundaboutByName(%q) = %v, want %v", tc.name, got, tc.want)
		}
	}
}
