package datastructure

import (
	"fmt"
	"strings"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	UNKNOWN            = -9999
	U_TURN_UNKNOWN     = -999
	U_TURN_LEFT        = -8
	KEEP_LEFT          = -7
	LEAVE_ROUNDABOUT   = -6
	TURN_SHARP_LEFT    = -3
	TURN_LEFT          = -2
	TURN_SLIGHT_LEFT   = -1
	CONTINUE_ON_STREET = 0
	TURN_SLIGHT_RIGHT  = 1
	TURN_RIGHT         = 2
	TURN_SHARP_RIGHT   = 3
	FINISH             = 4
	USE_ROUNDABOUT     = 6
	IGNORE             = 9999999
	KEEP_RIGHT         = 7
	U_TURN_RIGHT       = 8
	START              = 101
)

type Instruction struct {
	point                Coordinate
	turnSign             int
	streetname           string
	cumulativeDistance   float64
	cumulativeTravelTime float64
	extrainfo            map[string]interface{}
	isRoundabout         bool
	roundabout           *RoundaboutInstruction
	edgeIds              []Index
	edgeSpeed            float64
	points               []Coordinate
	turnBearing          float64
	turnType             string
}

func NewInstruction(sign int, name string, p Coordinate, isRoundAbout bool, edgeIds []Index,
	cumulativeDist, cumulativeTravelTime float64, points []Coordinate, turnBearing float64, clockwise bool) *Instruction {
	var roundabout *RoundaboutInstruction
	var ins *Instruction
	roundabout = NewRoundaboutInstruction()
	ins = &Instruction{
		turnSign:             sign,
		streetname:           name,
		point:                p,
		extrainfo:            make(map[string]interface{}, 3),
		roundabout:           roundabout,
		isRoundabout:         isRoundAbout,
		cumulativeTravelTime: cumulativeTravelTime,
		cumulativeDistance:   cumulativeDist,
		edgeIds:              edgeIds,
		points:               points,
		turnBearing:          turnBearing,
	}

	_, ins.turnType = getDirectionDescription(sign, ins, clockwise)

	return ins
}

func NewInstructionWithRoundabout(sign int, name string, p Coordinate, isRoundAbout bool, roundabout *RoundaboutInstruction,
	cumulativeDistance, cumulativeTravelTime float64, edgeIDs []Index, turnBearing float64) Instruction {
	ins := Instruction{
		turnSign:             sign,
		streetname:           name,
		point:                p,
		extrainfo:            make(map[string]interface{}, 3),
		roundabout:           roundabout,
		isRoundabout:         isRoundAbout,
		cumulativeDistance:   cumulativeDistance,
		cumulativeTravelTime: cumulativeTravelTime,
		edgeIds:              edgeIDs,
		turnBearing:          turnBearing,
	}
	ins.turnType = "ROUNDABOUT"
	return ins
}

func (ins *Instruction) GetStreetName() string {
	return ins.streetname
}

func (ins *Instruction) SetExtraInfo(key string, val interface{}) {
	ins.extrainfo[key] = val
}

func (ins *Instruction) SetSign(sign int) {
	ins.turnSign = sign
}

func (ins *Instruction) SetStreetName(streetName string) {
	ins.streetname = streetName
}

func (ins *Instruction) GetTurnSign() int {
	return ins.turnSign
}

func (ins *Instruction) GetPoints() []Coordinate {
	return ins.points
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
func (instr *Instruction) GetTurnBearing() float64 {
	return instr.turnBearing
}
func (instr *Instruction) GetTurnDescription(clockwise bool) string {

	streetName := instr.GetStreetName()
	sign := instr.GetTurnSign()
	var description string

	switch sign {
	case CONTINUE_ON_STREET:
		if isEmpty(streetName) {
			description = "Continue"
		} else {
			description = fmt.Sprintf("Continue onto %s", streetName)
		}
	case START:
		if heading, ok := instr.extrainfo["heading"]; ok {
			headingAngle := heading.(float64)
			if headingAngle < 0.0 {
				headingAngle += 360
			}
			compassDir := bearingToCompass(headingAngle)
			description = fmt.Sprintf("Head %s toward %s", compassDir, streetName)
		} else {
			description = fmt.Sprintf("Head toward %s", streetName)
		}
	case FINISH:
		description = fmt.Sprint("you have arrived at your destination")
	default:
		dir, _ := getDirectionDescription(sign, instr, clockwise)
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

func getDirectionDescription(sign int, instruction *Instruction, clockwise bool) (string, string) {
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
	case USE_ROUNDABOUT:
		if !instruction.roundabout.exited {
			return "Enter the roundabout", "USE_ROUNDABOUT"
		}
		roundaboutDir := "clockwise"
		if !clockwise {
			roundaboutDir = "counter-clockwise"
		}

		return fmt.Sprintf("At Roundabout, take the exit point %d %s", instruction.roundabout.exitNumber,
			roundaboutDir), ""

	default:
		return "", ""
	}
}

type RoundaboutInstruction struct {
	exitNumber int
	exited     bool
}

func NewRoundaboutInstruction() *RoundaboutInstruction {
	roundabout := &RoundaboutInstruction{
		exitNumber: 0,
		exited:     false,
	}

	return roundabout
}

func (i *Instruction) IncrementExitNumber() {
	i.roundabout.exitNumber++
}

func (i *Instruction) SetExited() {
	i.roundabout.exited = true
}

type DrivingDirection struct {
	instruction string
	point       Coordinate
	streetName  string
	travelTime  float64
	distance    float64
	edgeIds     []Index
	polyline    string
	turnBearing float64
	turnType    string
}

func NewDrivingDirection(ins Instruction, description string, prevETA, prevDist float64,
	edgeIds []Index, polyline string, turnBearing float64) DrivingDirection {
	return DrivingDirection{
		instruction: description,
		point:       ins.point,
		streetName:  ins.streetname,
		travelTime:  util.RoundFloat(prevETA, 2),
		distance:    util.RoundFloat(prevDist, 2),
		edgeIds:     edgeIds,
		polyline:    polyline,
		turnBearing: util.RoundFloat(turnBearing, 2),
		turnType:    ins.turnType,
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

func (d *DrivingDirection) GetPolyline() string {
	return d.polyline
}

func (d *DrivingDirection) GetTurnBearing() float64 {
	return d.turnBearing
}

func (d *DrivingDirection) GetTurnType() string {
	return d.turnType
}
