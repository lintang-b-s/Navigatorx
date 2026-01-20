package datastructure

import (
	"encoding/csv"
	"log"
	"math"
	"os"
	"strconv"

	"github.com/lintang-b-s/Navigatorx/pkg"
)

// periodic piecewise linear function
// for implementing td-crp: https://link-springer-com.ezproxy.ugm.ac.id/chapter/10.1007/978-3-319-38851-9_3
type PWL struct {
	points   []*Point // x=departure time , y=travel time
	min, max float64
}

func NewPWL(points []*Point) *PWL {

	pwl := &PWL{points, 0, 0}
	pwl.updateMinMax()

	return pwl
}

func (pwl *PWL) updateMinMax() {
	max := -1.0
	min := math.MaxFloat64

	for _, p := range pwl.points {
		if Lt(p.GetY(), min) {
			min = p.GetY()
		}

		if Lt(max, p.GetY()) {
			max = p.GetY()
		}
	}
	pwl.min = min
	pwl.max = max
}

func (pwl *PWL) GetPoints() []*Point {
	return pwl.points
}

func (pwl *PWL) SetPoints(ps []*Point) {
	pwl.points = ps
	pwl.updateMinMax()
}

func (pwl *PWL) isConst() bool {
	return len(pwl.points) == 1
}

func (pwl *PWL) GetMin() float64 {
	return pwl.min
}

func (pwl *PWL) GetMax() float64 {
	return pwl.max
}

func (pwl *PWL) get(i int) *Point {
	n := len(pwl.points)
	if i >= 0 && i < n {
		return pwl.points[i]
	}

	if i >= -n && i < 0 {
		pp := *pwl.points[i+n]
		pp.add(NewPoint(-PERIOD, 0))
		return &pp
	}

	if i >= n && i < n+n {
		pp := *pwl.points[i-n]
		pp.add(NewPoint(PERIOD, 0))
		return &pp
	}

	// (i >= n && i >= n+n ) ||  i < -n

	imod := i % n
	if imod < 0 {
		imod += n
	}

	offset := PERIOD * float64(i/n)
	pp := *pwl.points[imod]
	pp.add(NewPoint(offset, 0))
	return &pp
}

func (pwl *PWL) set(i int, p *Point) {
	pwl.points[i] = p
}

// get travel time value from the travel time function Evaluated at x=time
func (pwl *PWL) Eval(time float64) float64 {
	if pwl.isConst() {
		return pwl.get(0).GetY()
	}

	x := math.Mod(time, PERIOD)

	for i := 0; i < len(pwl.points); i++ {
		if Le(x, pwl.get(i).GetX()) {

			if eq(x, pwl.get(i).GetX()) {
				return pwl.get(i).GetY()
			}

			var p, q *Point
			if i > 0 {
				p = pwl.get(i - 1)
			} else {
				p = NewPoint(pwl.get(len(pwl.points)-1).GetX()-PERIOD, pwl.get(len(pwl.points)-1).GetY())
			}

			q = pwl.get(i)

			// p.x < x < q.x
			// linear interpolation
			m := (q.y - p.y) / (q.x - p.x)
			res := m*(x-p.x) + p.y
			return res
		}
	}

	var q, p *Point
	q = NewPoint(pwl.get(0).GetX()+PERIOD, pwl.get(0).GetY())
	p = pwl.get(len(pwl.points) - 1)

	// linear interpolation

	m := (q.GetY() - p.GetY()) / (q.GetX() - p.GetX())
	res := m*(x-p.GetX()) + p.GetY()

	return res
}

/*
given two edge (u,v) with travel time function f and (v,w) with travel time function g
u -> f -> v -> g -> w
travel time function f(\tau) adalah waktu tempuh dari edge saat berangkat dari node u pada waktu \tau

this function compute:
link(f,g): \tau -> g(\tau + f(\tau)) + f(\tau), for all \tau \in [0, \Pi]

*/
func Link(f, g *PWL) *PWL {
	if g.isConst() && f.isConst() {
		p := *g.get(0)
		p.add(f.get(0))
		resPWL := NewPWL([]*Point{&p})
		resPWL.updateMinMax()
		return resPWL
	}

	if f.isConst() {
		return LinkConstTwo(f.get(0).GetY(), g)
	}

	if g.isConst() {
		return LinkConstOne(f, g.get(0).GetY())
	}

	ps := make([]*Point, 0, f.Size()+g.Size())
	resPWL := NewPWL(ps)

	i := g.lowerBound(f.Eval(0))
	j := 0

	for true {
		var uwX, uwY float64

		if Ge(g.get(i).GetY(), pkg.INF_WEIGHT) || Ge(f.get(j).GetY(), pkg.INF_WEIGHT) {
			if resPWL.Size() == 0 {
				resPWL.appendPoint(NewPoint(0, pkg.INF_WEIGHT))
			}
			break
		}

		if eq(g.get(i).GetX(), f.get(j).GetX()+f.get(j).GetY()) {
			uwX = f.get(j).GetX()
			uwY = g.get(i).GetY() + f.get(j).GetY()

			i++
			j++
		} else if Lt(g.get(i).GetX(), f.get(j).GetX()+f.get(j).GetY()) {

			mArrivalFInverse := (f.get(j).GetX() - f.get(j-1).GetX()) / (f.get(j).GetX() + f.get(j).GetY() -
				f.get(j-1).GetX() - f.get(j-1).GetY())
			uwX = mArrivalFInverse*(g.get(i).GetX()-f.get(j-1).GetX()-f.get(j-1).GetY()) + f.get(j-1).GetX()
			uwY = g.get(i).GetX() + g.get(i).GetY() - uwX

			i++
		} else {
			// g[i-1].x < f.get(j).x + f.get(j).y < g.get(i).x

			// linear interpolation
			uwX = f.get(j).GetX()
			mG := (g.get(i).GetY() - g.get(i-1).GetY()) / (g.get(i).GetX() - g.get(i-1).GetX())
			uwY = g.get(i-1).GetY() + mG*(f.get(j).GetX()+f.get(j).GetY()-g.get(i-1).GetX()) + f.get(j).GetY()

			j++
		}

		if Lt(PERIOD, uwX) {

			break
		}

		uwX = math.Min(uwX, PERIOD)
		uwX = math.Max(uwX, 0.0)

		resPWL.appendPoint(NewPoint(uwX, uwY))
	}

	if resPWL.Size() > 1 {
		last := resPWL.Size() - 1
		firstP := *resPWL.get(0)
		firstP.add(NewPoint(PERIOD, 0))
		if collinear(resPWL.get(last-1), resPWL.get(last), &firstP) {
			resPWL.points = resPWL.points[:last]
		}
	}

	resPWL.updateMinMax()

	return resPWL
}

func LinkConstOne(f *PWL, c float64) *PWL {
	ps := make([]*Point, 0, f.Size())
	resPWL := NewPWL(ps)

	for i := 0; i < f.Size(); i++ {
		fp := *f.get(i)
		fp.add(NewPoint(0, c))
		resPWL.appendPoint(&fp)
	}

	resPWL.updateMinMax()

	return resPWL
}

func LinkConstTwo(c float64, f *PWL) *PWL {
	ps := make([]*Point, 0, f.Size())
	resPWL := NewPWL(ps)
	c = math.Mod(c, PERIOD)

	k := 0
	for k < f.Size() && Lt(f.get(k).GetX(), c) {
		k++
	}

	for i := k; i < f.Size(); i++ {
		// process f.points [c, PERIOD]
		fp := *f.get(i)
		fp.add(NewPoint(-c, c))
		resPWL.appendPoint(&fp)
	}

	for i := 0; i < k; i++ {
		// process f.points [0, c)
		fp := *f.get(i)
		fp.add(NewPoint(PERIOD-c, c))
		resPWL.appendPoint(&fp)
	}

	resPWL.updateMinMax()

	return resPWL
}

// merge(f,g): \tau -> min{g(\tau), f(\tau)}, untuk semua \tau \in [0,\Pi]
// O(|f| + |g|)
func Merge(f, g *PWL) *PWL {

	if g.isConst() {
		return mergeConst(f, g.points[0].GetY())
	}
	if f.isConst() {
		return mergeConstTwo(f.points[0].GetY(), g)
	}

	if Lt(f.max, g.min) {
		ff := *f
		return &ff
	} else if Lt(g.max, f.min) {
		gg := *g
		return &gg
	}

	ps := make([]*Point, 0, f.Size()+g.Size())
	resPWL := NewPWL(ps)

	i := 0
	j := 0

	n := f.Size()
	m := g.Size()

	for i < n || j < m {

		if eq(f.get(i).GetX(), g.get(j).GetX()) {

			if eq(f.get(i).GetY(), g.get(j).GetY()) {
				resPWL.appendPoint(f.get(i))
			} else if Lt(f.get(i).GetY(), g.get(j).GetY()) {
				resPWL.appendPoint(f.get(i))
			} else {
				resPWL.appendPoint(g.get(j))
			}
			i++
			j++
		} else if Lt(f.get(i).GetX(), g.get(j).GetX()) {

			if ccw(g.get(j-1), f.get(i), g.get(j)) {

				resPWL.appendPoint(f.get(i))
			} else if collinear(g.get(j-1), f.get(i), g.get(j)) {

				if (ccw(g.get(j-1), f.get(i-1), f.get(i))) || ccw(f.get(i), f.get(i+1), g.get(j)) {
					resPWL.appendPoint(f.get(i))
				} else if resPWL.Size() == 0 {

					resPWL.appendPoint(f.get(i))
				}
			}

			i++
		} else {

			if ccw(f.get(i-1), g.get(j), f.get(i)) {

				resPWL.appendPoint(g.get(j))
			} else if collinear(f.get(i-1), g.get(j), f.get(i)) {

				if (ccw(g.get(j-1), g.get(j), f.get(i-1))) || ccw(g.get(j), g.get(j+1), f.get(i)) {
					resPWL.appendPoint(g.get(j))
				}
				if resPWL.Size() == 0 {
					resPWL.appendPoint(g.get(j))
				}
			}
			j++
		}
	}

	if pLt(f.get(n-1), g.get(m-1)) {
		resPWL.appendPoint(f.get(n - 1))
	} else {
		resPWL.appendPoint(g.get(m - 1))
	}

	resPWL.updateMinMax()

	return resPWL
}

func mergeConst(f *PWL, c float64) *PWL {

	ps := make([]*Point, 0, f.Size())
	resPWL := NewPWL(ps)

	if Le(c, f.min) {
		return NewPWL([]*Point{NewPoint(0, c)})
	}

	n := f.Size()
	for i := 0; i < n; i++ {
		if eq(f.get(i).GetY(), c) {
			if Lt(f.get(i-1).GetY(), c) || Lt(f.get(i+1).GetY(), c) {
				resPWL.appendPoint(NewPoint(f.get(i).GetX(), c))
			} else if resPWL.Size() == 0 {
				resPWL.appendPoint(NewPoint(f.get(i).GetX(), math.Min(f.get(i).GetY(), c)))
			}
		} else if Lt(f.get(i).GetY(), c) {
			if Lt(c, f.get(i-1).GetY()) {

				itPoint := intersectionPointHorizontalLine(f.get(i-1), f.get(i), c)
				if itPoint.GetX() >= 0 {
					resPWL.appendPoint(itPoint)
				}
			}

			resPWL.appendPoint(f.get(i))
		} else if Lt(f.get(i-1).GetY(), c) {

			itPoint := intersectionPointHorizontalLine(f.get(i-1), f.get(i), c)
			if itPoint.GetX() >= 0 {
				resPWL.appendPoint(itPoint)
			}
		}
	}

	if Lt(f.get(n-1).GetY(), c) && Lt(c, f.get(n).GetY()) || Lt(c, f.get(n-1).GetY()) && Lt(f.get(n).GetY(), c) {
		itPoint := intersectionPointHorizontalLine(f.get(n-1), f.get(n), c)

		if Le(itPoint.GetX(), PERIOD) {

			resPWL.appendPoint(itPoint)
		}
	}

	if resPWL.Size() == 0 {
		resPWL.appendPoint(NewPoint(0, c))
	}

	resPWL.updateMinMax()

	return resPWL
}

func mergeConstTwo(c float64, f *PWL) *PWL {
	return mergeConst(f, c)
}

func (pwl *PWL) Size() int {
	return len(pwl.points)
}

func (pwl *PWL) appendPoint(p *Point) {

	n := len(pwl.points)
	if n != 0 && eq(pwl.get(n-1).GetX(), p.GetX()) {
		if eq(pwl.get(n-1).GetY(), p.GetY()) {
			return
		}

		lastPoint := pwl.get(n - 1)

		var pp *Point
		pp = NewPoint(math.Max(lastPoint.GetX()+EPS, p.GetX()+EPS), p.GetY())

		lastPoint.x = math.Min(lastPoint.GetX(), p.GetX())

		pwl.points = append(pwl.points, pp)
		return
	}

	if n != 0 && Lt(p.GetX(), pwl.get(n-1).GetX()) {

		var pp *Point
		lastPoint := pwl.get(n - 1)

		pp = NewPoint(lastPoint.GetX(), p.GetY())
		lastPoint.x = p.GetX()

		pwl.points = append(pwl.points, pp)
		return
	}

	if n > 1 {
		if collinear(pwl.get(n-2), pwl.get(n-1), p) {
			pwl.set(n-1, p)
			return
		}
	}

	pwl.points = append(pwl.points, p)
}

// smallest index in the sorted pwl.points[].x where the element is greater than or equal to the xx.
// O(log(N)), N = len(pwl.points)
func (pwl *PWL) lowerBound(xx float64) int {
	xx = math.Mod(xx, PERIOD)

	l, r := 0, len(pwl.points)-1
	id := len(pwl.points)
	for l <= r {
		mid := l + (r-l)/2
		if pwl.get(mid).GetX() >= xx {
			id = mid
			r = mid - 1

		} else {
			l = mid + 1
		}
	}

	return id
}

func ReadTravelTimeProfile(filepath string) (map[int64]*PWL, error) {
	f, err := os.Open(filepath)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	r := csv.NewReader(f)

	records, err := r.ReadAll()
	if err != nil {
		return nil, err
	}

	header := records[0]
	timeId := 0

	ttProfile := make(map[int64]*PWL)

	for colIdx, colName := range header {
		if colIdx == timeId {
			continue
		}

		points := []*Point{}
		for _, row := range records[1:] {

			timeSec, err := strconv.ParseFloat(row[timeId], 64)
			if err != nil {
				log.Fatal(err)
			}
			if !eq(math.Mod(timeSec, 7200), 0.0) {
				continue
			}
			travelTime, err := strconv.ParseFloat(row[colIdx], 64)
			if err != nil {
				log.Fatal(err)
			}
			points = append(points, NewPoint(timeSec, travelTime))
		}

		osmID, err := strconv.ParseInt(colName, 10, 64)
		if err != nil {
			log.Fatal(err)
		}

		ttProfile[osmID] = NewPWL(points)
	}

	return ttProfile, nil
}
