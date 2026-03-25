package offline

import (
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// https://web.stanford.edu/~jurafsky/slp3/A.pdf

type transition struct {
	from, to int
}

func newTransition(from, to int) transition {
	return transition{from, to}
}

func (tt *transition) getFrom() int {
	return tt.from
}

func (tt *transition) getTo() int {
	return tt.to
}

type state struct {
	stateId       int
	backPointer   *state
	observationId int
}

func newState(stateId int, backPointer *state, observationId int) *state {
	return &state{stateId: stateId, backPointer: backPointer, observationId: observationId}
}

func (st *state) getStateId() int {
	return st.stateId
}

func (st *state) getObservationId() int {
	return st.observationId
}

type Viterbi struct {
	lastState      map[int]*state
	prevCandidates []int
	forwardProb    map[int]float64
	isBroken       bool
}

func newViterbi() *Viterbi {
	return &Viterbi{}
}

func (vi *Viterbi) isHMMBroken() bool {
	return vi.isBroken
}

func (vi *Viterbi) initForwardProb(observationId int,
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

	vi.lastState = make(map[int]*state)
	for _, candidate := range candidates {
		vi.lastState[candidate] = &state{
			stateId:       candidate,
			backPointer:   nil,
			observationId: observationId,
		}
	}

	vi.prevCandidates = make([]int, len(candidates))
	copy(vi.prevCandidates, candidates)
}

func (v *Viterbi) transitionLogProbability(
	prevState int,
	curState int,
	transitionLogProbabilities map[transition]float64) float64 {

	transition := newTransition(prevState, curState)
	logProb, ok := transitionLogProbabilities[transition]

	if !ok {
		return INVALIDLOGPROB // transition has zero probability (log-based)
	}

	return logProb
}

func (vi *Viterbi) forwardStep(observationId int,
	candidates []int,
	emissionLogProbabilities map[int]float64,
	transitionLogProbabilities map[transition]float64) {
	candidatesForwardProb := make(map[int]float64, len(candidates))
	candidateStates := make(map[int]*state, len(candidates))

	for _, curState := range candidates {
		maxLogProbability := INVALIDLOGPROB
		var maxPrevState int = -1

		for _, prevState := range vi.prevCandidates {
			logProbability := vi.forwardProb[prevState] + vi.transitionLogProbability(
				prevState, curState, transitionLogProbabilities)

			if logProbability > maxLogProbability {
				maxLogProbability = logProbability
				maxPrevState = prevState
			}
		}

		emissionProb, ok := emissionLogProbabilities[curState]
		if !ok {
			emissionProb = INVALIDLOGPROB
		}
		candidatesForwardProb[curState] = maxLogProbability + emissionProb

		// maxPrevState will be nil if there is no transition with non -inf probability
		if maxPrevState != -1 {
			candidateStates[curState] = &state{
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

	vi.forwardProb = candidatesForwardProb
	vi.lastState = candidateStates
	vi.prevCandidates = make([]int, len(candidates))
	copy(vi.prevCandidates, candidates)
}

func (vi *Viterbi) hmmBreak(forwardProb map[int]float64) bool {
	for _, logProbability := range forwardProb {
		if !util.Lt(logProbability, INVALIDLOGPROB) {
			return false
		}
	}
	return true
}

func (v *Viterbi) retrieveMostLikelyStateSequence() []*state {
	var lastStateId int
	maxLogProbability := INVALIDLOGPROB

	for stateId, logProb := range v.forwardProb {
		if logProb > maxLogProbability {
			lastStateId = stateId
			maxLogProbability = logProb
		}
	}

	result := make([]*state, 0, 10)
	es := v.lastState[lastStateId]

	for es != nil {
		ss := newState(es.stateId, es, es.observationId)
		result = append(result, ss)
		es = es.backPointer
	}

	util.ReverseG(result)

	return result
}
