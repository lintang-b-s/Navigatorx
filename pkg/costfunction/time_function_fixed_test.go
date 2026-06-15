package costfunction

import (
	"math"
	"path/filepath"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func TestInt32PartialEdgeScaling(t *testing.T) {
	tf := NewTimeCostFunction(
		true,
		[]int32{125},
		[]uint32{1_000},
		[]uint32{8},
		nil,
	)

	if got := tf.GetWeightFromLength(0, 333); got != 42 {
		t.Fatalf("partial edge weight = %d, want 42 centiseconds", got)
	}
}

func TestInt32PublicUnitConversions(t *testing.T) {
	tf := NewTimeCostFunction(true, []int32{}, []uint32{}, []uint32{}, nil)
	if got := tf.WeightToSeconds(123); got != 1.23 {
		t.Fatalf("weight conversion = %v, want 1.23 seconds", got)
	}
	if got := tf.DistanceToMeters(12_345); got != 123.45 {
		t.Fatalf("distance conversion = %v, want 123.45 meters", got)
	}
}

func TestSpeedFromKilometerPerHour(t *testing.T) {
	tf := NewTimeCostFunction(true, []int32{}, []uint32{}, []uint32{}, nil)
	tests := []struct {
		name  string
		kmh   float64
		want  uint32
		error bool
	}{
		{name: "exact", kmh: 36, want: 10},
		{name: "nearest", kmh: 60, want: 17},
		{name: "below storage resolution", kmh: 0.1, want: 0},
		{name: "negative", kmh: -1, error: true},
		{name: "infinite", kmh: math.Inf(1), error: true},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			got := tf.SpeedFromKilometerPerHour(test.kmh)

			if got != test.want {
				t.Fatalf("SpeedFromKilometerPerHour(%v) = %d, want %d", test.kmh, got, test.want)
			}
		})
	}
}

func TestInt32TimeFunctionArtifactRoundTrip(t *testing.T) {
	filename := filepath.Join(t.TempDir(), "timefunction.ntf")
	tf := NewTimeCostFunction(
		true,
		[]int32{100, 200},
		[]uint32{1_000, 2_000},
		[]uint32{10, 10},
		[]uint16{25, util.TurnCostForbidden},
	)
	if err := tf.WriteToFile(filename); err != nil {
		t.Fatal(err)
	}

	got, err := ReadFromFile[int32](filename, nil)
	if err != nil {
		t.Fatal(err)
	}
	if got.GetWeight(1) != 200 || got.GetSegmentLength(1) != 2_000 {
		t.Fatalf("unexpected artifact values: weight=%d length=%d", got.GetWeight(1), got.GetSegmentLength(1))
	}
	if got.GetTurnCost(1) != util.Infinity[int32]() {
		t.Fatalf("forbidden turn did not decode to infinity")
	}
}

func TestTimeFunctionRejectsWrongNumericRepresentation(t *testing.T) {
	filename := filepath.Join(t.TempDir(), "timefunction.ntf")
	tf := NewTimeCostFunction(
		false,
		[]float64{1},
		[]uint32{200},
		[]uint32{0},
		nil,
	)
	if err := tf.WriteToFile(filename); err != nil {
		t.Fatal(err)
	}
	if _, err := ReadFromFile[int32](filename, nil); err == nil {
		t.Fatal("int32 reader accepted float64 artifact")
	}
}
