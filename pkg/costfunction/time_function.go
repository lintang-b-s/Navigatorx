package costfunction

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"

	"github.com/cockroachdb/errors"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
)

type TimeFunction struct {
	turnCosts     []float64 // map from pkg.TurnType to turn cost in seconds
	edgeMaxSpeeds []float64 // map from outEdge id to its maxspeed in meters per minute (m/s)
	isRoadNetwork bool
}

func NewTimeCostFunction(roadNetwork bool, edgeMaxSpeeds []float64) *TimeFunction {
	if !roadNetwork {
		return &TimeFunction{isRoadNetwork: false}
	}

	if pkg.WITH_TURN_COSTS {
		mapTurnCosts := viper.GetStringMap("turncosts")
		turnCosts := make([]float64, 6)
		for turnTypeStr, cost := range mapTurnCosts {

			turnType := getTurnType(turnTypeStr)
			switch v := cost.(type) {
			case int:
				turnCosts[turnType] = float64(v)
			case float64:
				turnCosts[turnType] = float64(v)
			default:
				panic("unsupported type")
			}
		}
		turnCosts[pkg.NONE] = 0
		turnCosts[pkg.NO_ENTRY] = pkg.INF_WEIGHT
		return &TimeFunction{turnCosts: turnCosts, isRoadNetwork: true, edgeMaxSpeeds: edgeMaxSpeeds}
	}

	return &TimeFunction{isRoadNetwork: true, edgeMaxSpeeds: edgeMaxSpeeds}
}

func NewTimeCostFunctionEmpty() *TimeFunction {
	return &TimeFunction{}
}

// GetWeight. Get travel time dari outEdge (in minutes).
// eId adalah id dari outEdge yang ingin didapat weightnya
func (tf *TimeFunction) GetWeight(eId da.Index, eDefaultWeight, eLength float64) float64 {
	if tf.isRoadNetwork {
		if util.Eq(eDefaultWeight, pkg.INF_WEIGHT) {
			return eDefaultWeight
		}
		maxspeed := tf.edgeMaxSpeeds[eId] // m/s
		if eDefaultWeight == 0 || math.IsNaN(maxspeed) {
			return 0
		}
		return eLength / maxspeed
	}
	// ini buat correctness test
	return eDefaultWeight
}

func (tf *TimeFunction) GetTurnCost(turnType pkg.TurnType) float64 {
	if tf.turnCosts == nil {
		return 0
	}

	return tf.turnCosts[turnType]
}

func getTurnType(turnTypeStr string) pkg.TurnType {
	var turnType pkg.TurnType
	switch turnTypeStr {
	case "left_turn":
		turnType = pkg.LEFT_TURN
	case "right_turn":
		turnType = pkg.RIGHT_TURN
	case "straight_on":
		turnType = pkg.STRAIGHT_ON
	case "u_turn":
		turnType = pkg.U_TURN
	case "no_entry":
		turnType = pkg.NO_ENTRY
	case "none":
		turnType = pkg.NONE
	default:
		panic("unsupported turn type")
	}
	return turnType
}

func (tf *TimeFunction) WriteToFile(filename string) error {
	dir := filepath.Dir(filename)
	if _, err := os.Stat(dir); os.IsNotExist(err) {
		if err := os.MkdirAll(dir, 0755); err != nil {
			return err
		}
	}

	f, err := os.Create(filename)
	if err != nil {
		return errors.Wrapf(err, "timefunction.WriteToFile: failed to create file %v", filename)
	}
	defer f.Close()

	w := bufio.NewWriter(f)

	numOfEdges := len(tf.edgeMaxSpeeds)
	_, err = fmt.Fprintf(w, "%d\n", numOfEdges)
	if err != nil {
		return errors.Wrapf(err, "timefunction.WriteToFile: failed to write numOfEdges: %v", numOfEdges)
	}
	for eId := 0; eId < numOfEdges; eId++ {
		_, err = fmt.Fprintf(w, "%s", strconv.FormatFloat(tf.edgeMaxSpeeds[eId], 'f', -1, 64))
		if err != nil {
			return errors.Wrapf(err, "timefunction.WriteToFile: failed to write tf.edgeMaxSpeeds[%d]: %v", eId, tf.edgeMaxSpeeds[eId])
		}
		if eId < numOfEdges-1 {
			_, err = fmt.Fprint(w, " ")
			if err != nil {
				return errors.Wrapf(err, "timefunction.WriteToFile: failed to write space")
			}
		}
	}

	_, err = fmt.Fprintf(w, "\n")
	if err != nil {
		return errors.Wrapf(err, "timefunction.WriteToFile: failed to write new line")
	}
	if err = w.Flush(); err != nil {
		return errors.Wrapf(err, "timefunction.WriteToFile: failed to flush bufio writer")
	}

	return nil
}

func ReadFromFile(filename string) (*TimeFunction, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, errors.Wrapf(err, "timefunction.ReadFromFile: failed to open file %v", filename)
	}

	defer f.Close()

	r := bufio.NewReader(f)
	line, err := util.ReadLine(r)
	if err != nil {
		return nil, errors.Wrapf(err, "timefunction.ReadFromFile: failed to util.ReadLine(r)")
	}

	numOfEdges, err := util.ParseInt(line)
	if err != nil {
		return nil, errors.Wrapf(err, "timefunction.ReadFromFile: failed read numOfEdges")
	}

	line, err = util.ReadLine(r)
	if err != nil {
		return nil, errors.Wrapf(err, "timefunction.ReadFromFile: failed to util.ReadLine(r) untuk edge max speeds")
	}

	edgeMaxSpeeds := make([]float64, numOfEdges)
	ff := util.Fields(line)
	for eId := 0; eId < numOfEdges; eId++ {
		eMaxSpeed, err := strconv.ParseFloat(ff[eId], 64)
		if err != nil {
			return nil, errors.Wrapf(err, "timefunction.ReadFromFile: failed to read maxspeed of eId: %v", eId)
		}
		edgeMaxSpeeds[eId] = eMaxSpeed
	}
	return NewTimeCostFunction(true, edgeMaxSpeeds), nil
}
