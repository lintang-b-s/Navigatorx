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
			name:        "TestIsConditionalAccessInTimerange: time range input Mo-Fr 07:00-09:00,16:00-18:00 dan currentTime inTimeRange",
			condTime:    "Mo-Fr 07:00-09:00,16:00-18:00",
			currentTime: time.Date(2026, 4, 8, 19, 2, 0, 0, time.Local),
			want:        false,
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
