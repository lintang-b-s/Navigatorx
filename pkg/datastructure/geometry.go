package datastructure

import (
	"math"
)

const (
	EPS = 1e-9
	
)

type Point struct {
	x, y float64
}

func NewPoint(x, y float64) *Point {
	return &Point{x, y}
}

func (p *Point) GetX() float64 {
	return p.x
}

func (p *Point) GetY() float64 {
	return p.y
}

// equal operator
func eq(a, b float64) bool {
	return math.Abs(a-b) <= EPS
}

// less than operator
func Lt(a, b float64) bool {
	return a+EPS < b
}

// greater than or equal than operator
func Ge(a, b float64) bool {
	return Le(b, a)
}

// less than or equal operator
func Le(a, b float64) bool {
	return a <= b+EPS
}

// equal operator
func pEqual(a, b *Point) bool {
	return eq(a.x, b.x) && eq(a.y, b.y)
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

// ax + by + c = 0
type Line struct {
	a, b, c float64
}

func NewLine(a, b, c float64) *Line {
	return &Line{a, b, c}
}

// compute line equation given 2 points on that line
func pointsToLine(p1, p2 *Point) *Line {
	l := &Line{}
	if math.Abs(p1.x-p2.x) < EPS {
		// vertical line
		l = &Line{1.0, 0.0, -p1.x}
	} else {
		a := -(p1.y - p2.y) / (p1.x - p2.x)
		l = &Line{a, 1.0, -(a * p1.x) - p1.y}
	}
	return l
}

func areParallel(l1, l2 *Line) bool {
	return math.Abs(l1.a-l2.a) < EPS && math.Abs(l1.b-l2.b) < EPS
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

// cross product of two vectors a and b
func cross(a, b *Vector) float64 {
	return a.x*b.y - a.y*b.x
}

// counterclockwise test
// returns true if point r is on the left side of line pq
func ccw(p, q, r *Point) bool {
	return cross(toVec(p, q), toVec(p, r)) > -EPS
}

// returns true if point r is on the same line as the line pq
func collinear(p, q, r *Point) bool {
	return math.Abs(cross(toVec(p, q), toVec(p, r))) < EPS
}

// returns true (+ intersection point p) if two lines are intersect
func areIntersect(l1, l2 *Line) (bool, *Point) {
	if areParallel(l1, l2) {
		return false, nil
	}

	p := NewPoint(0, 0)
	p.x = (l2.b*l1.c - l1.b*l2.c) / (l2.a*l1.b - l1.a*l2.b)

	if math.Abs(l1.b) > EPS {
		p.y = -(l1.a*p.x + l1.c)
	} else {
		p.y = -(l2.a*p.x + l2.c)
	}
	return true, p
}

// check wether line segments (ab) and (cd) intersect in exactly one point (+ intersection point)
func intersectionPoint(a, b, c, d *Point) *Point {
	l1 := pointsToLine(a, b)
	l2 := pointsToLine(c, d)

	_, intersectPoint := areIntersect(l1, l2)
	return intersectPoint
}

// check wether line segments (ab) and (cd) intersect in exactly one point (+ intersection point)
func intersect(a, b, c, d *Point) bool {
	l1 := pointsToLine(a, b)
	l2 := pointsToLine(c, d)

	isIntersect, _ := areIntersect(l1, l2)
	return isIntersect
}

// check wether line segment (ab) and y=c intersect
func intersectionPointHorizontalLine(a, b *Point, c float64) *Point {
	p := NewPoint(a.GetX(), c)
	q := NewPoint(b.GetX(), c)

	if math.Abs(p.GetX()-q.GetX()) <= EPS {
		return NewPoint((p.GetX()+q.GetX())/2.0, c)
	}

	return intersectionPoint(a, b, p, q)
}
