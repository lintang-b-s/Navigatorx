package datastructure

import "testing"

func TestIntersect(t *testing.T) {
	ps := []*Point{

		NewPoint(2, 2),
		NewPoint(4, 3),
		NewPoint(2, 4),
		NewPoint(6, 6),
		NewPoint(2, 6),
		NewPoint(6, 5),
		NewPoint(8, 6),
		NewPoint(4, 5),
	}

	testCases := []struct {
		name               string
		p1                 *Point
		p2                 *Point
		p3                 *Point
		p4                 *Point
		want               bool
		wantIntersectPoint *Point
	}{

		{
			name: "test intersect 1",

			p1:                 ps[0],
			p2:                 ps[6],
			p3:                 ps[1],
			p4:                 ps[7],
			wantIntersectPoint: NewPoint(4.0, 3.3333),
			want:               true,
		},
	}

	for _, tt := range testCases {
		t.Run(tt.name, func(t *testing.T) {

			got := intersect(tt.p1, tt.p2, tt.p3, tt.p4)
			intersectPoint := intersectionPoint(tt.p1, tt.p2, tt.p3, tt.p4)
			if got != tt.want || (!Eq(intersectPoint.x, tt.wantIntersectPoint.x) &&
				!Eq(intersectPoint.y, tt.wantIntersectPoint.y)) {
				t.Error("l1 & l2 should intersect")
			}

		})
	}
}

func TestIntersectHorizontalLine(t *testing.T) {
	ps := []*Point{

		NewPoint(2, 2),
		NewPoint(0, 3),
		NewPoint(2, 4),
	}

	testCases := []struct {
		name               string
		p1                 *Point
		p2                 *Point
		p3                 *Point
		p4                 *Point
		want               bool
		wantIntersectPoint *Point
	}{

		{
			name: "test intersect 1",

			p1:                 ps[0],
			p2:                 ps[2],
			p3:                 ps[1],
			wantIntersectPoint: NewPoint(2, 3),
			want:               true,
		},
	}

	for _, tt := range testCases {
		t.Run(tt.name, func(t *testing.T) {

			intersectPoint := intersectionPointHorizontalLine(tt.p1, tt.p2, tt.p3.GetY())
			if !Eq(intersectPoint.x, tt.wantIntersectPoint.x) &&
				!Eq(intersectPoint.y, tt.wantIntersectPoint.y) {
				t.Error("l1 & l2 should intersect")
			}

		})
	}
}
