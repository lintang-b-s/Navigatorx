package osmparser

// NOTE: kebanyakan unit tests pada package osmparser digenerate oleh gemini 3 flash

import (
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/paulmach/osm"
)

func TestNewOSMParserV2(t *testing.T) {
	p := NewOSMParserV2()
	if p == nil {
		t.Fatal("NewOSMParserV2 returned nil")
	}
	if p.wayNodeMap == nil {
		t.Error("wayNodeMap not initialized")
	}

	if p.highwayWhitelist == nil {
		t.Error("highwayWhitelist not initialized")
	}
}

func TestAcceptOsmWay(t *testing.T) {
	p := NewOSMParserV2()
	p.highwayWhitelist = map[string]struct{}{
		"motorway":     {},
		"trunk":        {},
		"primary":      {},
		"secondary":    {},
		"tertiary":     {},
		"residential":  {},
		"unclassified": {},
	}

	testCases := []struct {
		name string
		tags osm.Tags
		want bool
	}{
		{
			name: "Accepted highway",
			tags: osm.Tags{{Key: "highway", Value: "primary"}},
			want: true,
		},
		{
			name: "Unaccepted highway",
			tags: osm.Tags{{Key: "highway", Value: "footway"}},
			want: false,
		},
		{
			name: "Access no",
			tags: osm.Tags{{Key: "highway", Value: "primary"}, {Key: "access", Value: "no"}},
			want: false,
		},
		{
			name: "Access agricultural",
			tags: osm.Tags{{Key: "highway", Value: "primary"}, {Key: "access", Value: "agricultural"}},
			want: false,
		},
		{
			name: "Junction roundabout",
			tags: osm.Tags{{Key: "junction", Value: "roundabout"}},
			want: true,
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			way := &osm.Way{Tags: tc.tags}
			got := p.acceptOsmWay(way)
			if got != tc.want {
				t.Errorf("acceptOsmWay(%v) = %v, want %v", tc.tags, got, tc.want)
			}
		})
	}
}

func TestIsBarrierNodeAccessible(t *testing.T) {
	p := NewOSMParserV2()
	p.currentTime = time.Date(2026, 4, 8, 12, 0, 0, 0, time.Local)

	// Mock pkg.VehicleType
	originalVehicleType := pkg.VehicleType
	defer func() { pkg.VehicleType = originalVehicleType }()
	pkg.VehicleType = pkg.MOTORCAR

	testCases := []struct {
		name string
		tags osm.Tags
		want bool
	}{
		{
			name: "No barrier",
			tags: osm.Tags{},
			want: true,
		},
		{
			name: "Bollard for motorcar",
			tags: osm.Tags{{Key: "barrier", Value: "bollard"}},
			want: false,
		},
		{
			name: "Gate access yes",
			tags: osm.Tags{{Key: "barrier", Value: "gate"}, {Key: "access", Value: "yes"}},
			want: true,
		},
		{
			name: "Gate access no",
			tags: osm.Tags{{Key: "barrier", Value: "gate"}, {Key: "access", Value: "no"}},
			want: false,
		},
		{
			name: "Conditional access yes",
			tags: osm.Tags{
				{Key: "barrier", Value: "gate"},
				{Key: "access:conditional", Value: "yes @ (09:00-17:00)"},
			},
			want: true,
		},
		{
			name: "Conditional access no",
			// access:conditional is stored for runtime evaluation, not evaluated at parse time.
			// Without an explicit access=no tag, the node is accessible during parsing.
			tags: osm.Tags{
				{Key: "barrier", Value: "gate"},
				{Key: "access:conditional", Value: "no @ (09:00-17:00)"},
			},
			want: true,
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			node := &osm.Node{Tags: tc.tags}
			got, err := p.isBarrierNodeAccessible(node)
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if got != tc.want {
				t.Errorf("isBarrierNodeAccessible(%v) = %v, want %v", tc.tags, got, tc.want)
			}
		})
	}
}
