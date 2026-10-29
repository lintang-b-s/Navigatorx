package util

import (
	"fmt"
	"sync"
)

// Progress tracks and prints percentage-based completion progress.
type Progress struct {
	mu          sync.Mutex
	total       int
	completed   int
	percent     int
	percentShow int
}

// NewProgress creates a new Progress tracker with the given total count.
func NewProgress(total int, percentShow int) *Progress {
	return &Progress{total: total, percentShow: percentShow, percent: 0}
}

// Add records completed work items and prints progress milestones.
func (p *Progress) Add(vertices int) {
	p.mu.Lock()
	defer p.mu.Unlock()

	p.completed += vertices
	for p.percent <= 100 && p.completed*100 >= p.percent*p.total {
		// while completed/total >= percent/100
		fmt.Printf("%d%%...", p.percent)
		p.percent += p.percentShow
	}
}

// Finish completes the progress output by printing a trailing newline.
func (p *Progress) Finish() {
	fmt.Println()
}
