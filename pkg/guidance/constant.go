package guidance

import (
	"sync"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

const (
	// RelativeBearingUTurn is the relative bearing uturn value.
	RelativeBearingUTurn float64 = 100.0
	// MergeRelativeBearing is the merge relative bearing value.
	MergeRelativeBearing float64 = 40.0

	// KeepLeftRightALTTurnRelativeBearing is the keep left right altturn relative bearing value.
	KeepLeftRightALTTurnRelativeBearing float64 = 20.0

	// ContinueALTTurnRelativeBearing is the continue altturn relative bearing value.
	ContinueALTTurnRelativeBearing float64 = 8.6

	// ContinueALTCurrentRelativeBearing is the continue  altcurrent relative bearing value.
	ContinueALTCurrentRelativeBearing float64 = 7

	// UTurnRelativeBearingMin is the uturn relative bearing min value.
	UTurnRelativeBearingMin float64 = 160

	// UTurnRelativeBearingMax is the uturn relative bearing max value.
	UTurnRelativeBearingMax float64 = 185

	ignoreTurnSignCacheVal = uint64(da.IGNORE) | uint64(da.InvalidStreetNameId)<<8
)

const (
	instructionsPoolClassesLen = 12
	routeStepsPoolClassesLen   = 12
)

var (
	// inspired by http/2 sync.pool example in: https://github.com/golang/go/blob/7e394a2/src/net/http/h2_bundle.go#L998-L1043
	instructionsCapClasses = [instructionsPoolClassesLen]int{
		1 << 4,
		1 << 5,
		1 << 6,
		1 << 7,
		1 << 8,
		1 << 9,
		1 << 10,
		1 << 11,
		1 << 12,
		1 << 13,
		1 << 14,
		1 << 15,
	}

	instructionsPools [instructionsPoolClassesLen]sync.Pool = [instructionsPoolClassesLen]sync.Pool{ // nolint:dupl
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<4)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<5)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<6)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<7)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<8)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<9)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<10)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<11)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<12)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<13)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<14)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.Instruction, 0, 1<<15)
			return s
		}},
	}
)

func getInstructions(capacity int) []da.Instruction {
	for i := range instructionsPoolClassesLen {
		if capacity <= instructionsCapClasses[i] {
			s := instructionsPools[i].Get().([]da.Instruction)
			s = s[:0]
			return s
		}
	}
	s := make([]da.Instruction, 0, capacity)
	return s
}

func putInstructions(s []da.Instruction) {
	c := cap(s)
	for i := range instructionsPoolClassesLen {
		if c == instructionsCapClasses[i] {
			instructionsPools[i].Put(s) // nolint:staticcheck
			return
		}
	}
}

var (
	// https://github.com/golang/go/blob/7e394a2/src/net/http/h2_bundle.go#L998-L1043
	routeStepsPools [routeStepsPoolClassesLen]sync.Pool = [routeStepsPoolClassesLen]sync.Pool{ // nolint:dupl
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<4)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<5)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<6)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<7)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<8)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<9)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<10)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<11)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<12)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<13)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<14)
			return s
		}},
		{New: func() interface{} {
			s := make([]da.RouteStep, 0, 1<<15)
			return s
		}},
	}
)

func getRouteSteps(capacity int) []da.RouteStep {
	for i := range routeStepsPoolClassesLen {
		if capacity <= instructionsCapClasses[i] {
			s := routeStepsPools[i].Get().([]da.RouteStep)
			s = s[:0]
			return s
		}
	}
	s := make([]da.RouteStep, 0, capacity)
	return s
}

// PutRouteSteps put the routesteps back to sync.Pool to be able to reused.
func PutRouteSteps(s []da.RouteStep) {
	c := cap(s)
	for i := range routeStepsPoolClassesLen {
		if c == instructionsCapClasses[i] {
			routeStepsPools[i].Put(s) // nolint:staticcheck
			return
		}
	}
}
