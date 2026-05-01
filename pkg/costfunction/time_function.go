// Package costfunction provides interfaces and implementations for edge and turn cost calculations.
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
)

type TimeFunction struct {
	turnTable []float64 // map from turnTableId  to turn cost in seconds
	// turnMatrices[v][i][j] -> turnCosts = flattened 1-D indexed array index of i-th incoming edge dari v and j-th outgoing edge dari v  = i*outDegree + j

	edgeMaxSpeeds []float64 // map from outEdge id to its maxspeed in meters per minute (m/s)
	isRoadNetwork bool
}

func NewTimeCostFunction(roadNetwork bool, edgeMaxSpeeds []float64, turnTable []float64) *TimeFunction {
	if !roadNetwork {
		return &TimeFunction{isRoadNetwork: false}
	}

	if pkg.WITH_TURN_COSTS {

		return &TimeFunction{turnTable: turnTable, isRoadNetwork: true, edgeMaxSpeeds: edgeMaxSpeeds}
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
		maxspeed := tf.edgeMaxSpeeds[eId] // m/s
		if util.Eq(eDefaultWeight, pkg.INF_WEIGHT) || util.Eq(maxspeed, 0) {
			return eDefaultWeight
		}
		if eDefaultWeight == 0 || math.IsNaN(maxspeed) {
			return 0
		}
		return eLength / maxspeed
	}
	// ini buat correctness test
	return eDefaultWeight
}

func (tf *TimeFunction) GetTurnCost(turnTableId da.Index) float64 {
	if tf.turnTable == nil {
		return 0
	}

	return tf.turnTable[turnTableId]
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

	numOfTurns := len(tf.turnTable)
	_, err = fmt.Fprintf(w, "%d\n", numOfTurns)
	if err != nil {
		return errors.Wrapf(err, "timefunction.WriteToFile: failed to write numOfTurns: %v", numOfTurns)
	}
	if numOfTurns > 0 {
		for tId := 0; tId < numOfTurns; tId++ {
			_, err = fmt.Fprintf(w, "%s", strconv.FormatFloat(tf.turnTable[tId], 'f', -1, 64))
			if err != nil {
				return errors.Wrapf(err, "timefunction.WriteToFile: failed to write tf.turnTable[%d]: %v", tId, tf.turnTable[tId])
			}
			if tId < numOfTurns-1 {
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

	line, err = util.ReadLine(r)
	if err != nil {
		if err.Error() == "EOF" {
			return NewTimeCostFunction(true, edgeMaxSpeeds, nil), nil
		}
		return nil, errors.Wrapf(err, "timefunction.ReadFromFile: failed to read numOfTurns")
	}

	numOfTurns, err := util.ParseInt(line)
	if err != nil {
		return nil, errors.Wrapf(err, "timefunction.ReadFromFile: failed read numOfTurns")
	}

	var turnTable []float64
	if numOfTurns > 0 {
		line, err = util.ReadLine(r)
		if err != nil {
			return nil, errors.Wrapf(err, "timefunction.ReadFromFile: failed to util.ReadLine(r) untuk turn table")
		}

		turnTable = make([]float64, numOfTurns)
		ff = util.Fields(line)
		for tId := 0; tId < numOfTurns; tId++ {
			tCost, err := strconv.ParseFloat(ff[tId], 64)
			if err != nil {
				return nil, errors.Wrapf(err, "timefunction.ReadFromFile: failed to read turn cost of tId: %v", tId)
			}
			turnTable[tId] = tCost
		}
	}

	return NewTimeCostFunction(true, edgeMaxSpeeds, turnTable), nil
}
