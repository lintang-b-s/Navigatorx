package util

import (
	"fmt"
	"sync"
)

// Progress tracks and prints percentage-based completion progress.
type Progress struct {
	mu        sync.Mutex
	total     int
	completed int
	percent   int
}

// NewProgress creates a new Progress tracker with the given total count.
func NewProgress(total int) *Progress {
	return &Progress{total: total, percent: 4}
}

// Add records completed work items and prints progress milestones.
func (p *Progress) Add(vertices int) {
	p.mu.Lock()
	defer p.mu.Unlock()

	p.completed += vertices
	for p.percent <= 100 && p.total > 0 && p.completed*100 >= p.percent*p.total {
		// while completed/total >= percent/100
		fmt.Printf("%d%%...", p.percent)
		p.percent += 2
	}
}

// Finish completes the progress output by printing a trailing newline.
func (p *Progress) Finish() {
	fmt.Println()
}
