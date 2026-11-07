// Package datastructure provides core graph data structures and navigation-related types.
package datastructure

import (
	"fmt"
	"strings"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// TurnType represents turn type.
type TurnType uint8

const (
	// UNKNOWN is the unknown value.
	UNKNOWN TurnType = iota
	// UTurnUnknown is the uturn unknown value.
	UTurnUnknown
	// UTurnLeft is the uturn left value.
	UTurnLeft

	// KeepLeft is the keep left value.
	KeepLeft

	// LeaveRoundabout is the leave roundabout value.
	LeaveRoundabout

	// TurnSharpLeft is the turn sharp left value.
	TurnSharpLeft

	// TurnLeft is the turn left value.
	TurnLeft

	// TurnSlightLeft is the turn slight left value.
	TurnSlightLeft

	// ContinueOnStreet is the continue on street value.
	ContinueOnStreet

	// MergeOnto is the merge onto value.
	MergeOnto

	// TurnSlightRight is the turn slight right value.
	TurnSlightRight

	// TurnRight is the turn right value.
	TurnRight

	// TurnSharpRight is the turn sharp right value.
	TurnSharpRight

	// FINISH is the finish value.
	FINISH

	// UseRoundabout is the use roundabout value.
	UseRoundabout

	// IGNORE is the ignore value.
	IGNORE

	// KeepRight is the keep right value.
	KeepRight

	// UTurnRight is the uturn right value.
	UTurnRight

	// START is the start value.
	START
)

// IsContinue reports whether continue.
// IsContinue. return true if tt is CONTINUE_ON_STREET
func IsContinue(tt TurnType) bool {
	return tt == ContinueOnStreet
}

// IsTurnSlight reports whether turn slight.
// IsTurnSlight. return true if tt is TURN_SLIGHT_* or CONTINUE_ON_STREET
func IsTurnSlight(tt TurnType) bool {
	return tt == TurnSlightLeft || tt == TurnSlightRight || tt == ContinueOnStreet
}

// Annotation represents annotation.
type Annotation struct {
	duration              []float64
	distance              []float64
	geometry              string   // geometry of a all road segments inside a single route step
	segmentGeometryOffset []uint32 // offset (starting point) of each road segment geometry in this single route step
}

// NewAnnotation creates and returns annotation.
func NewAnnotation(duration, distance []float64, geometry string, segmentGeometryOffset []uint32) Annotation {
	return Annotation{
		duration:              duration,
		distance:              distance,
		geometry:              geometry,
		segmentGeometryOffset: segmentGeometryOffset,
	}
}

// Duration returns duration.
func (ann *Annotation) Duration() []float64 {
	return ann.duration
}

// Distance returns distance.
func (ann *Annotation) Distance() []float64 {
	return ann.distance
}

// Geometry returns geometry.
func (ann *Annotation) Geometry() string {
	return ann.geometry
}

// SegmentGeometryOffset returns segment geometry offset.
func (ann *Annotation) SegmentGeometryOffset() []uint32 {
	return ann.segmentGeometryOffset
}

// Instruction represents instruction.
type Instruction struct {
	annotation          Annotation
	segmentIds          []Index
	roundabout          RoundaboutInstruction
	heading             float64
	hasHeading          bool
	point               Coordinate
	streetname          string
	cumulativeDistance  float64
	cumulativeCost      float64
	turnBearing         float64
	turnType            string
	turnSign            TurnType
	isRoundabout        bool
	suggestAlternatives bool
}

// NewInstruction creates and returns instruction.
func NewInstruction(sign TurnType, name string, p Coordinate, isRoundAbout bool, segmentIds []Index,
	cumulativeDist, cumulativeCost float64, turnBearing float64, ann Annotation,
	clockwise bool) Instruction {
	ins := Instruction{
		annotation:         ann,
		turnSign:           sign,
		streetname:         name,
		point:              p,
		isRoundabout:       isRoundAbout,
		cumulativeCost:     cumulativeCost,
		cumulativeDistance: cumulativeDist,
		segmentIds:         segmentIds,
		turnBearing:        turnBearing,
	}

	_, ins.turnType = getDirectionDescription(sign, ins.roundabout.exited, ins.roundabout.exitNumber, clockwise)

	return ins
}

// NewInstructionWithRoundabout creates and returns instruction with roundabout.
func NewInstructionWithRoundabout(sign TurnType, name string, p Coordinate, isRoundAbout bool, roundabout RoundaboutInstruction,
	cumulativeDistance, cumulativeCost float64, segmentIds []Index, ann Annotation, turnBearing float64) Instruction {

	ins := Instruction{
		annotation:         ann,
		turnSign:           sign,
		streetname:         name,
		point:              p,
		roundabout:         roundabout,
		isRoundabout:       isRoundAbout,
		cumulativeDistance: cumulativeDistance,
		cumulativeCost:     cumulativeCost,
		segmentIds:         segmentIds,
		turnBearing:        turnBearing,
	}
	ins.turnType = "ROUNDABOUT"
	return ins
}

// StreetName returns street name.
func (ins *Instruction) StreetName() string {
	return ins.streetname
}

// SetSuggestAlternatives sets suggest alternatives.
func (ins *Instruction) SetSuggestAlternatives(suggestAlternatives bool) {
	ins.suggestAlternatives = suggestAlternatives
}

// SuggestAlternatives returns suggest alternatives.
func (ins *Instruction) SuggestAlternatives() bool {
	return ins.suggestAlternatives
}

// SetHeading sets heading.
func (ins *Instruction) SetHeading(heading float64) {
	ins.heading = heading
	ins.hasHeading = true
}

// SetSign sets sign.
func (ins *Instruction) SetSign(sign TurnType) {
	ins.turnSign = sign
}

// SetStreetName sets street name.
func (ins *Instruction) SetStreetName(streetName string) {
	ins.streetname = streetName
}

// TurnSign returns turn sign.
func (ins *Instruction) TurnSign() TurnType {
	return ins.turnSign
}

// SegmentIds returns segment ids.
func (ins *Instruction) SegmentIds() []Index {
	return ins.segmentIds
}

// Point returns point.
func (ins *Instruction) Point() Coordinate {
	return ins.point
}

// CumulativeDistance returns cumulative distance.
func (ins *Instruction) CumulativeDistance() float64 {
	return ins.cumulativeDistance
}

// CumulativeCost returns cumulative cost.
func (ins *Instruction) CumulativeCost() float64 {
	return ins.cumulativeCost
}

// Annotation returns annotation.
func (ins *Instruction) Annotation() Annotation {
	return ins.annotation
}

func bearingToCompass(bearing float64) string {
	switch {
	case bearing < 22.5:
		return "North"
	case bearing < 67.5:
		return "North East"
	case bearing < 112.5:
		return "East"
	case bearing < 157.5:
		return "South East"
	case bearing < 202.5:
		return "South"
	case bearing < 247.5:
		return "South West"
	case bearing < 292.5:
		return "West"
	case bearing < 337.5:
		return "North West"
	default:
		return "North"
	}
}

// GetTurnBearing returns turn bearing.
func (ins *Instruction) GetTurnBearing() float64 {
	return ins.turnBearing
}

var headCompassLookup = [8]string{
	"Head north", "Head northeast", "Head east", "Head southeast",
	"Head south", "Head southwest", "Head west", "Head northwest",
}

func compassDirection(bearing float64) string {

	idx := int((bearing+22.5)/45) % 8
	return headCompassLookup[idx]
}

// GetTurnDescription returns turn description.
func (ins *Instruction) GetTurnDescription(clockwise bool) string {

	streetName := ins.streetname
	streetName = strings.ReplaceAll(streetName, "\x00", "")
	sign := ins.turnSign
	switch sign {
	case ContinueOnStreet:
		if isEmpty(streetName) {
			return "Continue"
		}
		return "Continue onto " + streetName
	case START:
		if ins.hasHeading {
			headingAngle := ins.heading
			headingAngle = util.RadiansToDegree(headingAngle)
			if headingAngle < 0.0 {
				headingAngle += 360
			}
			compassDir := bearingToCompass(headingAngle)
			if streetName != "" {
				return "Head " + compassDir + " toward " + streetName
			}
			return compassDirection(headingAngle)
		}
		return "Head toward " + streetName
	case FINISH:
		return "you have arrived at your destination"

	}
	dir, _ := getDirectionDescription(sign, ins.roundabout.exited, ins.roundabout.exitNumber, clockwise)
	if dir == "" {
		return fmt.Sprintf("unknown  %d", sign)
	}
	if isEmpty(streetName) {
		return dir
	}
	switch dir {
	case "Keep left":
		return "Keep left to continue on " + streetName
	case "Keep right":
		return "Keep right continue on " + streetName
	}
	return dir + " onto " + streetName
}

func isEmpty(str string) bool {
	return strings.TrimSpace(str) == ""
}

func getDirectionDescription(sign TurnType, roundaboutExited bool, roundaboutExitNum int, clockwise bool) (string, string) {
	switch sign {
	case UTurnUnknown:
		return "Make U-turn", "U_TURN_RIGHT"
	case UTurnRight:
		return "Make U-turn right", "U_TURN_RIGHT"
	case UTurnLeft:
		return "Make U-turn left", "U_TURN_LEFT"
	case KeepLeft:
		return "Keep left", "KEEP_LEFT"
	case TurnSharpLeft:
		return "Turn sharp left", "TURN_SHARP_LEFT"
	case TurnLeft:
		return "Turn left", "TURN_LEFT"
	case TurnSlightLeft:
		return "Turn slight left", "TURN_SLIGHT_LEFT"
	case TurnSlightRight:
		return "Turn slight right", "TURN_SLIGHT_RIGHT"
	case TurnRight:
		return "Turn right", "TURN_RIGHT"
	case TurnSharpRight:
		return "Turn sharp right", "TURN_SHARP_RIGHT"
	case KeepRight:
		return "Keep right", "KEEP_RIGHT"
	case MergeOnto:
		return "Merge onto", "MERGE_ONTO"
	case ContinueOnStreet:
		return "Continue onto", "CONTINUE_ONTO"
	case UseRoundabout:
		if !roundaboutExited {
			return "Enter the roundabout", "USE_ROUNDABOUT"
		}
		roundaboutDir := "clockwise"
		if !clockwise {
			roundaboutDir = "counter-clockwise"
		}

		return fmt.Sprintf("At Roundabout, take the exit point %d %s",
			roundaboutExitNum, roundaboutDir), ""
	default:
		return "", ""
	}
}

// RoundaboutInstruction represents roundabout instruction.
type RoundaboutInstruction struct {
	exitNumber int
	exited     bool
}

// NewRoundaboutInstruction creates and returns roundabout instruction.
func NewRoundaboutInstruction() RoundaboutInstruction {
	return RoundaboutInstruction{}
}

// IncrementExitNumber handles increment exit number.
func (ins *Instruction) IncrementExitNumber() {
	ins.roundabout.exitNumber++
}

// SetExited sets exited.
func (ins *Instruction) SetExited() {
	ins.roundabout.exited = true
}

// RouteStep represents a single driving direction route step .
type RouteStep struct {
	instruction         string
	segmentIds          []Index
	annotation          Annotation
	streetName          string
	point               Coordinate
	travelTime          float64
	distance            float64
	turnBearing         float64
	turnType            string
	suggestAlternatives bool
}

// NewRouteStep creates and returns driving direction route step.
func NewRouteStep(ins Instruction, description string, prevCost, prevDist float64,
	segmentIds []Index, turnBearing float64, ann Annotation) RouteStep {
	return RouteStep{
		instruction:         description,
		point:               ins.point,
		streetName:          ins.streetname,
		travelTime:          util.RoundFloat(prevCost, 2),
		distance:            util.RoundFloat(prevDist, 2),
		segmentIds:          segmentIds,
		turnBearing:         util.RoundFloat(turnBearing, 2),
		turnType:            ins.turnType,
		suggestAlternatives: ins.suggestAlternatives,
		annotation:          ann,
	}
}

// GetInstruction returns instruction.
func (d *RouteStep) GetInstruction() string {

	return d.instruction
}

// GetPoint returns point.
func (d *RouteStep) GetPoint() Coordinate {
	return d.point
}

// GetStreetName returns street name.
func (d *RouteStep) GetStreetName() string {
	return d.streetName
}

// GetCost returns cost.
func (d *RouteStep) GetCost() float64 {
	return d.travelTime
}

// GetDistance returns distance.
func (d *RouteStep) GetDistance() float64 {
	return d.distance
}

// GetSegmentsIds returns segments ids.
func (d *RouteStep) GetSegmentsIds() []Index {
	return d.segmentIds
}

// GetTurnBearing returns turn bearing.
func (d *RouteStep) GetTurnBearing() float64 {
	return d.turnBearing
}

// GetTurnTableId returns turn table id.
func (d *RouteStep) GetTurnTableId() string {
	return d.turnType
}

// GetSuggestAlternatives returns suggest alternatives.
func (d *RouteStep) GetSuggestAlternatives() bool {
	return d.suggestAlternatives
}

// Annotation returns annotation.
func (d *RouteStep) Annotation() Annotation {
	return d.annotation
}
