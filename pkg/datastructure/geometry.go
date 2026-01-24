package datastructure

import (
	"math"
)

const (
	EPS = 1e-6
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

func Gt(a, b float64) bool {
	return Lt(b, a)
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

func dir(p, q, r *Point) int {

	if math.Abs(p.GetX()-q.GetX()) < EPS && math.Abs(p.GetY()-q.GetY()) < EPS {
		return 0
	}

	if math.Abs(p.GetX()-r.GetX()) < EPS && math.Abs(p.GetY()-r.GetY()) < EPS {
		return 0
	}

	if math.Abs(q.GetX()-r.GetX()) < EPS && math.Abs(q.GetY()-r.GetY()) < EPS {
		return 0
	}

	x := cross(toVec(p, r), toVec(p, q))
	if math.Abs(x) < EPS {
		return 0
	}

	if x > 0 {
		return 1
	}
	return -1
}

// counterclockwise test
// returns true if point r is on the left side of line pq
func ccw(p, q, r *Point) bool {
	return dir(p, q, r) == -1
}

func cw(p, q, r *Point) bool {
	return dir(p, q, r) == 1
}

// returns true if point r is on the same line as the line pq
func collinear(p, q, r *Point) bool {
	return dir(p, q, r) == 0
}

// return dot product of two vectors a and b
func dot(a, b *Vector) float64 {
	return a.x*b.x + a.y*b.y
}

func normSq(v *Vector) float64 {
	return v.x*v.x + v.y*v.y
}

// return ccw angle <aob
func angle(a, o, b *Point) float64 {
	oa := toVec(o, a)
	ob := toVec(o, b)
	ang := math.Acos(dot(oa, ob) / math.Sqrt(normSq(oa)*normSq(ob)))

	return ang
}

// check wether line segments (ab) and (pq) intersect (+ intersection point)
func intersectionPoint(a, b, p, q *Point) *Point {

	denom := cross(toVec(q, p), toVec(a, b))

	bb := *b
	cp := cross(toVec(q, p), toVec(a, p)) / denom
	bb.sub(a)
	bb.multConst(cp)
	aa := *a
	aa.add(&bb)
	return &aa
}

// check wether line segments (ab) and (cd) intersect in exactly zone point
func intersect(a, b, p, q *Point) bool {

	if dir(a, b, p) == 0 {
		return false
	}
	if dir(a, b, q) == 0 {
		return false
	}

	if dir(p, q, a) == 0 {
		return false
	}
	if dir(p, q, b) == 0 {
		return false
	}

	if dir(a, b, p) == dir(a, b, q) {
		return false
	}
	if dir(p, q, a) == dir(p, q, b) {
		return false
	}

	return true
}

// check wether line segment (ab) and y=c intersect
func intersectionPointHorizontalLine(a, b *Point, c float64) *Point {
	p := NewPoint(a.GetX(), c)
	q := NewPoint(b.GetX(), c)

	if math.Abs(p.GetX()-q.GetX()) <= EPS {
		return NewPoint((p.GetX()+q.GetX())/2.0, c)
	}

	itPoint := intersectionPoint(a, b, p, q)

	return itPoint
}
