package datastructure

import "slices"

// adapted from circular queue datastructure implementation in: https://docs.google.com/presentation/d/e/2PACX-1vRqnhc-Ad0PtI53U5gCGA8cXvmV_cNT4gRFZtMHu5pB2y5gi4SWH3XhqcpwDWWgzYxFXATDd6mHXEsa/pub?slide=id.g3f98749a2c7_1_7
// or https://github.com/gophercon/2026-talks/blob/main/SimonLaw-ThreeSimpleDataStructuresBuiltOnSlices/README.md

// Queue represents preallocated circular queue data structure
// the maximum size of the queue must be known in advance
type Queue[T comparable] struct {
	s    []T
	beg  int
	end  int
	zero T
}

// NewQueue creates and returns new circular queue
func NewQueue[T comparable](size int) *Queue[T] {
	var zero T
	return &Queue[T]{s: make([]T, size), zero: zero}
}

// at get i-th (0-indexed) element from the front element of the circular queue. handling circular wrap-around atomatically.
func (q *Queue[T]) at(i int) int {
	return (q.beg + i) % len(q.s)
}

// Push push item to the queue. return false if the circular queue is full.
func (q *Queue[T]) Push(x T) bool {
	if q.end == len(q.s) {
		return false
	}
	q.s[q.at(q.end)] = x
	q.end++
	return true
}

// Pop pop item from the queue. follows First in, First Out. return false is the circular queue is empty.
func (q *Queue[T]) Pop() (x T, ok bool) {
	if q.end == 0 {
		return q.zero, false
	}
	x, q.s[q.beg] = q.s[q.beg], q.zero
	q.beg = q.at(1)
	q.end--
	return x, true
}

// Values return remaining items in queue.
func (q *Queue[T]) Values() (xs []T) {
	if q.end == 0 {
		return make([]T, 0)
	}
	b, e := q.beg, q.at(q.end)
	if b < e {
		return q.s[b:e]
	}
	xs = append(slices.Clip(q.s[b:]), q.s[:e]...)
	return xs
}

// IsEmpty returns true if queue is empty.
func (q *Queue[T]) IsEmpty() bool {
	return q.end == 0
}

// Reset reset the circular queue.
func (q *Queue[T]) Reset() {
	if q.end == 0 {
		return
	}
	clear(q.s)
	q.beg = 0
	q.end = 0
}
