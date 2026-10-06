package util

import (
	"fmt"
	"sync"
)

type Progress struct {
	mu        sync.Mutex
	total     int
	completed int
	percent   int
}

func NewProgress(total int) *Progress {
	return &Progress{total: total, percent: 4}
}

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

func (p *Progress) Finish() {
	fmt.Println()
}
