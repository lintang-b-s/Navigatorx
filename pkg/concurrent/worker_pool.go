package concurrent

import (
	"context"
	"fmt"
	"sync"
	"time"
)

var (
	ErrScheduleTimeout = fmt.Errorf("schedule timed out")
)

type JobFunc[T any, G any] func(job T) G

type JobFuncPlain func()

type WorkerPool[T any, G any] struct {
	numWorkers int
	jobQueue   chan T
	results    chan G
	workQueue  chan JobFuncPlain
	sem        chan struct{}
	wg         sync.WaitGroup
}

func NewWorkerPool[T any, G any](numWorkers, jobQueueSize int) *WorkerPool[T, G] {
	return &WorkerPool[T, G]{
		numWorkers: numWorkers,
		jobQueue:   make(chan T, jobQueueSize),
		results:    make(chan G, jobQueueSize),
		workQueue:  make(chan JobFuncPlain, jobQueueSize),
		sem:        make(chan struct{}, numWorkers),
	}
}

func (wp *WorkerPool[T, G]) Spawn(queueSize int) {
	for i := 0; i < wp.numWorkers; i++ {
		wp.sem <- struct{}{}
		go wp.worker2(func() {
		})
	}
}

func (wp *WorkerPool[T, G]) Schedule(task JobFuncPlain) error {
	return wp.schedule(task, nil)
}

func (wp *WorkerPool[T, G]) ScheduleTimeout(timeout time.Duration, task JobFuncPlain) error {
	return wp.schedule(task, time.After(timeout))
}

func (wp *WorkerPool[T, G]) schedule(task JobFuncPlain, timeout <-chan time.Time) error {
	select {
	case <-timeout:
		return ErrScheduleTimeout
	case wp.workQueue <- task:
		return nil
	case wp.sem <- struct{}{}:
		go wp.worker2(task)
		return nil
	}
}

func (wp *WorkerPool[T, G]) worker2(task JobFuncPlain) {
	defer func() { <-wp.sem }()

	task()

	for task := range wp.workQueue {
		task()
	}
}

func (wp *WorkerPool[T, G]) worker(jobFunc JobFunc[T, G]) {
	defer wp.wg.Done()
	for job := range wp.jobQueue {
		res := jobFunc(job)
		wp.results <- res
	}
}

func (wp *WorkerPool[T, G]) Start(jobFunc JobFunc[T, G]) {
	for i := 1; i <= wp.numWorkers; i++ {
		wp.wg.Add(1)
		go wp.worker(jobFunc)
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

func (wp *WorkerPool[T, G]) StartWithContext(ctx context.Context, jobFunc JobFunc[T, G]) {
	for i := 1; i <= wp.numWorkers; i++ {
		wp.wg.Add(1)
		go wp.workerWithContext(ctx, jobFunc)
	}
}

func (wp *WorkerPool[T, G]) Wait() {
	go func() {
		wp.wg.Wait()
		close(wp.results)
	}()
}

func (wp *WorkerPool[T, G]) WaitDirect() {
	wp.wg.Wait()
	close(wp.results)
}

func (wp *WorkerPool[T, G]) AddJob(job T) {
	wp.jobQueue <- job
}

func (wp *WorkerPool[T, G]) CollectResults() chan G {
	return wp.results
}

func (wp *WorkerPool[T, G]) Close() {
	close(wp.jobQueue)
	if wp.workQueue != nil {
		close(wp.workQueue)
	}
}
