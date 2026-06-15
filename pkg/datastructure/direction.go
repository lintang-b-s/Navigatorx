// Package datastructure provides core graph data structures and navigation-related types.
package datastructure

import (
	"fmt"
	"strings"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type TurnType uint8

const (
	UNKNOWN TurnType = iota
	U_TURN_UNKNOWN
	U_TURN_LEFT
	KEEP_LEFT
	LEAVE_ROUNDABOUT
	TURN_SHARP_LEFT
	TURN_LEFT
	TURN_SLIGHT_LEFT
	CONTINUE_ON_STREET
	MERGE_ONTO
	TURN_SLIGHT_RIGHT
	TURN_RIGHT
	TURN_SHARP_RIGHT
	FINISH
	USE_ROUNDABOUT
	IGNORE
	KEEP_RIGHT
	U_TURN_RIGHT
	START
)

// IsContinue. return true if tt is CONTINUE_ON_STREET
func IsContinue(tt TurnType) bool {
	return tt == CONTINUE_ON_STREET
}

// IsTurnSlight. return true if tt is TURN_SLIGHT_* or CONTINUE_ON_STREET
func IsTurnSlight(tt TurnType) bool {
	return tt == TURN_SLIGHT_LEFT || tt == TURN_SLIGHT_RIGHT || tt == CONTINUE_ON_STREET
}

type Annotation struct {
	duration       []float64
	distance       []float64
	geometry       Coordinates
	edgeGeomOffset []Index
}

func NewAnnotation(duration, distance []float64, geometry Coordinates, edgeGeomOffset []Index) Annotation {
	durationCopy := make([]float64, len(duration))
	copy(durationCopy, duration)

	distanceCopy := make([]float64, len(distance))
	copy(distanceCopy, distance)

	geometryCopy := make(Coordinates, len(geometry))
	copy(geometryCopy, geometry)

	edgeGeomOffsetCopy := make([]Index, len(edgeGeomOffset))
	copy(edgeGeomOffsetCopy, edgeGeomOffset)

	return Annotation{
		duration:       durationCopy,
		distance:       distanceCopy,
		geometry:       geometryCopy,
		edgeGeomOffset: edgeGeomOffsetCopy,
	}
}

func (ann *Annotation) GetDuration() []float64 {
	return ann.duration
}

func (ann *Annotation) GetDistance() []float64 {
	return ann.distance
}

func (ann *Annotation) GetGeometry() Coordinates {
	return ann.geometry
}

func (ann *Annotation) GetEdgeGeomOffset() []Index {
	return ann.edgeGeomOffset
}

type Instruction struct {
	annotation Annotation
	edgeIds    []Index

	roundabout           RoundaboutInstruction
	heading              float64
	hasHeading           bool
	point                Coordinate
	streetname           string
	cumulativeDistance   float64
	cumulativeTravelTime float64
	turnBearing          float64
	turnType             string
	turnSign             TurnType
	isRoundabout         bool
	suggestAlternatives  bool
}

func NewInstruction(sign TurnType, name string, p Coordinate, isRoundAbout bool, edgeIds []Index,
	cumulativeDist, cumulativeTravelTime float64, turnBearing float64, ann Annotation,
	clockwise bool) Instruction {
	ins := Instruction{
		annotation:           ann,
		turnSign:             sign,
		streetname:           name,
		point:                p,
		isRoundabout:         isRoundAbout,
		cumulativeTravelTime: cumulativeTravelTime,
		cumulativeDistance:   cumulativeDist,
		edgeIds:              edgeIds,
		turnBearing:          turnBearing,
	}

	_, ins.turnType = getDirectionDescription(sign, &ins, clockwise)

	return ins
}

func NewInstructionWithRoundabout(sign TurnType, name string, p Coordinate, isRoundAbout bool, roundabout RoundaboutInstruction,
	cumulativeDistance, cumulativeTravelTime float64, edgeIds []Index, ann Annotation, turnBearing float64) Instruction {

	ins := Instruction{
		annotation:           ann,
		turnSign:             sign,
		streetname:           name,
		point:                p,
		roundabout:           roundabout,
		isRoundabout:         isRoundAbout,
		cumulativeDistance:   cumulativeDistance,
		cumulativeTravelTime: cumulativeTravelTime,
		edgeIds:              edgeIds,
		turnBearing:          turnBearing,
	}
	ins.turnType = "ROUNDABOUT"
	return ins
}

func (ins *Instruction) GetStreetName() string {
	return ins.streetname
}

func (ins *Instruction) SetSuggestAlternatives(suggestAlternatives bool) {
	ins.suggestAlternatives = suggestAlternatives
}

func (ins *Instruction) GetSuggestAlternatives() bool {
	return ins.suggestAlternatives
}

func (ins *Instruction) SetHeading(heading float64) {
	ins.heading = heading
	ins.hasHeading = true
}

func (ins *Instruction) SetSign(sign TurnType) {
	ins.turnSign = sign
}

func (ins *Instruction) SetStreetName(streetName string) {
	ins.streetname = streetName
}

func (ins *Instruction) GetTurnSign() TurnType {
	return ins.turnSign
}

func (ins *Instruction) GetEdgeIds() []Index {
	return ins.edgeIds
}

func (ins *Instruction) GetPoint() Coordinate {
	return ins.point
}

func (ins *Instruction) GetCumulativeDistance() float64 {
	return ins.cumulativeDistance
}

func (ins *Instruction) GetCumulativeTravelTime() float64 {
	return ins.cumulativeTravelTime
}

func (ins *Instruction) GetAnnotation() Annotation {
	return ins.annotation
}

func bearingToCompass(bearing float64) string {
	if bearing < 22.5 {
		return "North"
	} else if bearing < 67.5 {
		return "North East"
	} else if bearing < 112.5 {
		return "East"
	} else if bearing < 157.5 {
		return "South East"
	} else if bearing < 202.5 {
		return "South"
	} else if bearing < 247.5 {
		return "South West"
	} else if bearing < 292.5 {
		return "West"
	} else if bearing < 337.5 {
		return "North West"
	} else {
		return "North"
	}
}
func (ins *Instruction) GetTurnBearing() float64 {
	return ins.turnBearing
}
func (ins *Instruction) GetTurnDescription(clockwise bool) string {

	streetName := ins.GetStreetName()
	streetName = strings.ReplaceAll(streetName, "\x00", "")

	sign := ins.GetTurnSign()
	var description string

	switch sign {
	case CONTINUE_ON_STREET:
		if isEmpty(streetName) {
			description = "Continue"
		} else {
			description = fmt.Sprintf("Continue onto %s", streetName)
		}
	case START:
		if ins.hasHeading {
			headingAngle := ins.heading
			headingAngle = util.RadiansToDegree(headingAngle)
			if headingAngle < 0.0 {
				headingAngle += 360
			}
			compassDir := bearingToCompass(headingAngle)
			if streetName != "" {
				description = fmt.Sprintf("Head %s toward %s", compassDir, streetName)
			} else {
				description = fmt.Sprintf("Head %s", compassDir)
			}
		} else {
			description = fmt.Sprintf("Head toward %s", streetName)
		}
	case FINISH:
		description = "you have arrived at your destination"
	default:
		dir, _ := getDirectionDescription(sign, ins, clockwise)
		if dir == "" {
			description = fmt.Sprintf("unknown  %d", sign)
		} else {
			if isEmpty(streetName) {
				description = dir
			} else {
				switch dir {
				case "Keep left":
					description = fmt.Sprintf("%s to continue on %s", dir, streetName)
				case "Keep right":
					description = fmt.Sprintf("%s continue on %s", dir, streetName)

				default:

					description = fmt.Sprintf("%s onto %s", dir, streetName)
				}
			}
		}
	}

	return description
}

func isEmpty(str string) bool {
	return strings.TrimSpace(str) == ""
}

func getDirectionDescription(sign TurnType, ins *Instruction, clockwise bool) (string, string) {
	switch sign {
	case U_TURN_UNKNOWN:
		return "Make U-turn", "U_TURN_RIGHT"
	case U_TURN_RIGHT:
		return "Make U-turn right", "U_TURN_RIGHT"
	case U_TURN_LEFT:
		return "Make U-turn left", "U_TURN_LEFT"
	case KEEP_LEFT:
		return "Keep left", "KEEP_LEFT"
	case TURN_SHARP_LEFT:
		return "Turn sharp left", "TURN_SHARP_LEFT"
	case TURN_LEFT:
		return "Turn left", "TURN_LEFT"
	case TURN_SLIGHT_LEFT:
		return "Turn slight left", "TURN_SLIGHT_LEFT"
	case TURN_SLIGHT_RIGHT:
		return "Turn slight right", "TURN_SLIGHT_RIGHT"
	case TURN_RIGHT:
		return "Turn right", "TURN_RIGHT"
	case TURN_SHARP_RIGHT:
		return "Turn sharp right", "TURN_SHARP_RIGHT"
	case KEEP_RIGHT:
		return "Keep right", "KEEP_RIGHT"
	case MERGE_ONTO:
		return "Merge onto", "MERGE_ONTO"
	case CONTINUE_ON_STREET:
		return "Continue onto", "CONTINUE_ONTO"
	case USE_ROUNDABOUT:
		if !ins.roundabout.exited {
			return "Enter the roundabout", "USE_ROUNDABOUT"
		}
		roundaboutDir := "clockwise"
		if !clockwise {
			roundaboutDir = "counter-clockwise"
		}

		return fmt.Sprintf("At Roundabout, take the exit point %d %s", ins.roundabout.exitNumber,
			roundaboutDir), ""

	default:
		return "", ""
	}
}

type RoundaboutInstruction struct {
	exitNumber int
	exited     bool
}

func NewRoundaboutInstruction() RoundaboutInstruction {
	return RoundaboutInstruction{}
}

func (ins *Instruction) IncrementExitNumber() {
	ins.roundabout.exitNumber++
}

func (ins *Instruction) SetExited() {
	ins.roundabout.exited = true
}

type DrivingDirection struct {
	instruction         string
	edgeIds             []Index
	annotation          Annotation
	streetName          string
	point               Coordinate
	travelTime          float64
	distance            float64
	turnBearing         float64
	turnType            string
	suggestAlternatives bool
}

func NewDrivingDirection(ins Instruction, description string, prevTravelTime, prevDist float64,
	edgeIds []Index, turnBearing float64, ann Annotation) DrivingDirection {
	return DrivingDirection{
		instruction:         description,
		point:               ins.point,
		streetName:          ins.streetname,
		travelTime:          util.RoundFloat(prevTravelTime, 2),
		distance:            util.RoundFloat(prevDist, 2),
		edgeIds:             edgeIds,
		turnBearing:         util.RoundFloat(turnBearing, 2),
		turnType:            ins.turnType,
		suggestAlternatives: ins.suggestAlternatives,
		annotation:          ann,
	}
}

func (d *DrivingDirection) GetInstruction() string {
	return d.instruction
}

func (d *DrivingDirection) GetPoint() Coordinate {
	return d.point
}

func (d *DrivingDirection) GetStreetName() string {
	return d.streetName
}

func (d *DrivingDirection) GetTravelTime() float64 {
	return d.travelTime
}

func (d *DrivingDirection) GetDistance() float64 {
	return d.distance
}

func (d *DrivingDirection) GetEdgesIds() []Index {
	return d.edgeIds
}

func (d *DrivingDirection) GetTurnBearing() float64 {
	return d.turnBearing
}

func (d *DrivingDirection) GetTurnTableId() string {
	return d.turnType
}

func (d *DrivingDirection) GetSuggestAlternatives() bool {
	return d.suggestAlternatives
}

func (d *DrivingDirection) GetAnnotation() Annotation {
	return d.annotation
}
