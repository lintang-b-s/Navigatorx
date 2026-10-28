// Package concurrent provides utilities for concurrent computing.
package concurrent

import (
	"context"
	"fmt"
	"sync"
	"time"
)

var (
	// ErrScheduleTimeout is returned when scheduling a task exceeds the specified timeout.
	ErrScheduleTimeout = fmt.Errorf("schedule timed out")
)

// JobFunc represents a function that processes a job of type T and returns a result of type G.
type JobFunc[T any, G any] func(job T) G

// JobFuncPlain represents a parameterless task function.
type JobFuncPlain func()

// WorkerPool manages a pool of workers processing concurrent jobs.
type WorkerPool[T any, G any] struct {
	numWorkers int
	jobQueue   chan T
	results    chan G
	workQueue  chan JobFuncPlain
	sem        chan struct{}
	wg         sync.WaitGroup
}

// NewWorkerPool creates a new WorkerPool with the specified number of workers and queue size.
func NewWorkerPool[T any, G any](numWorkers, jobQueueSize int) *WorkerPool[T, G] {
	return &WorkerPool[T, G]{
		numWorkers: numWorkers,
		jobQueue:   make(chan T, jobQueueSize),
		results:    make(chan G, jobQueueSize),
		workQueue:  make(chan JobFuncPlain, jobQueueSize),
		sem:        make(chan struct{}, numWorkers),
	}
}

// Spawn creates and starts the specified number of initial workers.
func (wp *WorkerPool[T, G]) Spawn(initialWorkers int) {
	for i := 0; i < initialWorkers; i++ {
		wp.sem <- struct{}{}
		go wp.worker2(func() {
		})
	}
}

// Schedule schedules a task for execution by the worker pool.
func (wp *WorkerPool[T, G]) Schedule(task JobFuncPlain) error {
	return wp.schedule(task, nil)
}

// ScheduleTimeout schedules a task for execution, returning ErrScheduleTimeout if the timeout expires.
func (wp *WorkerPool[T, G]) ScheduleTimeout(timeout time.Duration, task JobFuncPlain) error {
	return wp.schedule(task, time.After(timeout))
}

func (wp *WorkerPool[T, G]) schedule(task JobFuncPlain, timeout <-chan time.Time) error {
	select {
	case <-timeout:
		return ErrScheduleTimeout
	case wp.workQueue <- task:
		return nil
	case wp.sem <- struct{}{}: // coba acquire semaphore. kalau gak full, spawn new worker
		go wp.worker2(task)
		return nil
	}
}

func (wp *WorkerPool[T, G]) worker2(task JobFuncPlain) {
	defer func() {
		// release the semaphore
		<-wp.sem
	}()

	task()

	for task := range wp.workQueue {
		task()
	}
}

func (wp *WorkerPool[T, G]) workerWithContext(ctx context.Context, jobFunc JobFunc[T, G]) {
	defer wp.wg.Done()

	for {
		select {
		case <-ctx.Done():
			return
		case job, ok := <-wp.jobQueue:
			if !ok {
				return
			}

			result := jobFunc(job)

			select {
			case wp.results <- result:
			case <-ctx.Done():
				return
			}
		}
	}
}

// StartWithContext starts worker goroutines with the provided context and job function.
func (wp *WorkerPool[T, G]) StartWithContext(ctx context.Context, jobFunc JobFunc[T, G]) {
	for i := 1; i <= wp.numWorkers; i++ {
		wp.wg.Add(1)
		go wp.workerWithContext(ctx, jobFunc)
	}
}

// Wait waits for all workers to finish and closes the results channel.
func (wp *WorkerPool[T, G]) Wait() {
	wp.wg.Wait()
	close(wp.results)
}

// AddJob enqueues a job into the job queue.
func (wp *WorkerPool[T, G]) AddJob(job T) {
	wp.jobQueue <- job
}

// CollectResults returns the channel for collecting processed job results.
func (wp *WorkerPool[T, G]) CollectResults() chan G {
	return wp.results
}

// Close closes the jobQueue and workQueue channels.
func (wp *WorkerPool[T, G]) Close() {
	close(wp.jobQueue)
	if wp.workQueue != nil {
		close(wp.workQueue)
	}
}
