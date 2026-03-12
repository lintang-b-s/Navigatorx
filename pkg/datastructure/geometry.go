package datastructure

import (
	"math"
)

const (
	EPS = 1e-9
)

type Point struct {
	x, y float64
	id   int
}

func NewPoint(x, y float64) *Point {
	return &Point{x, y, 0}
}

func (p *Point) GetX() float64 {
	return p.x
}

func (p *Point) GetY() float64 {
	return p.y
}

// equal operator
func Eq(a, b float64) bool {
	return math.Abs(a-b) <= EPS
}

// equal operator
func EqEps(a, b, eps float64) bool {
	return math.Abs(a-b) <= eps
}

// less than operator
func Lt(a, b float64) bool {
	return a+EPS < b
}

// greater than or equal to operator
func Ge(a, b float64) bool {
	return Le(b, a)
}

func Gt(a, b float64) bool {
	return Lt(b, a)
}

// less than or equal operator
func Le(a, b float64) bool {
	return a <= b+EPS
}

// equal operator
func pEqual(a, b *Point) bool {
	return Eq(a.x, b.x) && Eq(a.y, b.y)
}

// less than operator
func pLt(a, b *Point) bool {
	if math.Abs(a.x-b.x) > EPS {
		return a.x < b.x
	}
	return a.y < b.y
}

func (p *Point) add(p2 *Point) {
	p.x += p2.x
	p.y += p2.y
}

func (p *Point) sub(p2 *Point) {
	p.x -= p2.x
	p.y -= p2.y
}

func (p *Point) mult(p2 *Point) {
	p.x *= p2.x
	p.y *= p2.y
}

func (p *Point) multConst(q float64) {
	p.x *= q
	p.y *= q
}

type Vector struct {
	x, y float64
}

func NewVector(x, y float64) *Vector {
	return &Vector{x, y}
}

func toVec(a, b *Point) *Vector {
	return NewVector(b.x-a.x, b.y-a.y)
}
