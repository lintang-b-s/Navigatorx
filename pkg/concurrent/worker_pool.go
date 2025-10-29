package concurrent

import (
	"sync"
)

type JobFunc[T any, G any] func(job T) G

type WorkerPool[T any, G any] struct {
	numWorkers int
	jobQueue   chan T
	results    chan G
	wg         sync.WaitGroup
}

func NewWorkerPool[T any, G any](numWorkers, jobQueueSize int) *WorkerPool[T, G] {
	return &WorkerPool[T, G]{
		numWorkers: numWorkers,
		jobQueue:   make(chan T, jobQueueSize),
		results:    make(chan G, jobQueueSize),
	}
}

func (wp *WorkerPool[T, G]) worker(id int, jobFunc JobFunc[T, G]) {
	defer wp.wg.Done()
	for job := range wp.jobQueue {
		res := jobFunc(job)
		wp.results <- res
	}
}

func (wp *WorkerPool[T, G]) Start(jobFunc JobFunc[T, G]) {
	for i := 1; i <= wp.numWorkers; i++ {
		wp.wg.Add(1)
		go wp.worker(i, jobFunc)
	}
}

func (wp *WorkerPool[T, G]) Wait() {
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
}
