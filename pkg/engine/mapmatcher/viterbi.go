package mapmatcher

import (
	"maps"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// https://web.stanford.edu/~jurafsky/slp3/A.pdf

type Transition struct {
	from, to int
}

func NewTransition(from, to int) Transition {
	return Transition{from, to}
}

type State struct {
	stateId       int
	backPointer   *State
	observationId int
}

func newState(stateId int, backPointer *State, observationId int) *State {
	return &State{stateId: stateId, backPointer: backPointer, observationId: observationId}
}

func (st *State) GetStateId() int {
	return st.stateId
}

func (st *State) GetObservationId() int {
	return st.observationId
}

type Viterbi struct {
	lastState      map[int]*State
	prevCandidates []int
	forwardProb    map[int]float64
	isBroken       bool

	candidatesHistory  [][]int
	lastStateHistory   []map[int]*State
	forwardProbHistory []map[int]float64
}

func NewViterbi(numberOfObservations int) *Viterbi {
	lastStateHistory := make([]map[int]*State, numberOfObservations)
	forwardProbHistory := make([]map[int]float64, numberOfObservations)
	for i := 0; i < numberOfObservations; i++ {
		lastStateHistory[i] = make(map[int]*State)
		forwardProbHistory[i] = make(map[int]float64)
	}
	return &Viterbi{
		candidatesHistory:  make([][]int, numberOfObservations),
		lastStateHistory:   lastStateHistory,
		forwardProbHistory: forwardProbHistory,
	}
}

func (vi *Viterbi) IsHMMBroken() bool {
	return vi.isBroken
}

func (vi *Viterbi) GetForwardProb() map[int]float64 {
	return vi.forwardProb
}

func (vi *Viterbi) InitForwardProb(observationId int,
	candidates []int,
	initialLogProbabilities map[int]float64) {

	initForwardProb := make(map[int]float64)
	for _, candidate := range candidates {
		logProbability := initialLogProbabilities[candidate]
		initForwardProb[candidate] = logProbability
	}

	vi.isBroken = vi.hmmBreak(initForwardProb)
	if vi.isBroken {
		return
	}

	vi.forwardProb = initForwardProb

	vi.lastState = make(map[int]*State)
	for _, candidate := range candidates {
		vi.lastState[candidate] = &State{
			stateId:       candidate,
			backPointer:   nil,
			observationId: observationId,
		}
	}

	vi.prevCandidates = make([]int, len(candidates))
	copy(vi.prevCandidates, candidates)
	vi.saveHistory(observationId, candidates, vi.lastState, vi.forwardProb)
}

func (vi *Viterbi) transitionLogProbability(
	prevState int,
	curState int,
	transitionLogProbabilities map[Transition]float64) float64 {

	transition := NewTransition(prevState, curState)
	logProb, ok := transitionLogProbabilities[transition]

	if !ok {
		return InvalidLogProb // transition has zero probability (log-based)
	}

	return logProb
}

func (vi *Viterbi) ForwardStep(observationId int,
	candidates []int,
	emissionLogProbabilities map[int]float64,
	transitionLogProbabilities map[Transition]float64) {
	candidatesForwardProb := make(map[int]float64, len(candidates))
	candidateStates := make(map[int]*State, len(candidates))

	for _, curState := range candidates {
		maxLogProbability := InvalidLogProb
		var maxPrevState = -1

		for _, prevState := range vi.prevCandidates {
			logProbability := vi.forwardProb[prevState] + vi.transitionLogProbability(
				prevState, curState, transitionLogProbabilities)

			if util.Gt(logProbability, maxLogProbability) {
				maxLogProbability = logProbability
				maxPrevState = prevState
			}
		}

		emissionProb, ok := emissionLogProbabilities[curState]
		if !ok {
			emissionProb = InvalidLogProb
		}

		candidatesForwardProb[curState] = maxLogProbability + emissionProb

		// maxPrevState will be nil if there is no transition with non -inf probability
		if maxPrevState != -1 {
			candidateStates[curState] = &State{
				stateId:       curState,
				backPointer:   vi.lastState[maxPrevState],
				observationId: observationId,
			}
		}
	}

	vi.isBroken = vi.hmmBreak(candidatesForwardProb)
	if vi.isBroken {
		return
	}

	// update current states
	vi.forwardProb = make(map[int]float64)
	maps.Copy(vi.forwardProb, candidatesForwardProb)
	vi.lastState = make(map[int]*State)
	maps.Copy(vi.lastState, candidateStates)
	vi.prevCandidates = make([]int, len(candidates))
	copy(vi.prevCandidates, candidates)

	vi.saveHistory(observationId, candidates, candidateStates, candidatesForwardProb)
}

func (vi *Viterbi) saveHistory(
	observationId int,
	candidates []int,
	lastState map[int]*State,
	forwardProb map[int]float64,
) {

	vi.candidatesHistory[observationId] = make([]int, len(candidates))
	copy(vi.candidatesHistory[observationId], candidates)
	vi.lastStateHistory[observationId] = make(map[int]*State)
	maps.Copy(vi.lastStateHistory[observationId], lastState)
	vi.forwardProbHistory[observationId] = make(map[int]float64)
	maps.Copy(vi.forwardProbHistory[observationId], forwardProb)
}

func (vi *Viterbi) StepBack(observationId int) {

	vi.forwardProb = make(map[int]float64)
	maps.Copy(vi.forwardProb, vi.forwardProbHistory[observationId])
	vi.lastState = make(map[int]*State)
	maps.Copy(vi.lastState, vi.lastStateHistory[observationId])
	vi.prevCandidates = make([]int, len(vi.candidatesHistory[observationId]))
	copy(vi.prevCandidates, vi.candidatesHistory[observationId])
	vi.isBroken = false
}

func (vi *Viterbi) hmmBreak(forwardProb map[int]float64) bool {
	for _, logProbability := range forwardProb {
		if util.Gt(logProbability, InvalidLogProb) {
			return false
		}
	}

	return true
}

func (vi *Viterbi) RetrieveMostLikelyStateSequence() []*State {
	if len(vi.forwardProb) == 0 || len(vi.lastState) == 0 {
		return []*State{}
	}

	var lastStateId int
	maxLogProbability := InvalidLogProb

	for stateId, logProb := range vi.forwardProb {
		if logProb > maxLogProbability {
			lastStateId = stateId
			maxLogProbability = logProb
		}
	}

	result := make([]*State, 0, 10)
	es, ok := vi.lastState[lastStateId]
	if !ok {
		return result
	}

	for es != nil {
		ss := newState(es.stateId, es, es.observationId)
		result = append(result, ss)
		es = es.backPointer
	}

	util.ReverseG(result)

	return result
}
