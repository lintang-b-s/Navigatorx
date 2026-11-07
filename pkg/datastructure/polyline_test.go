package datastructure

import (
	"iter"
	"math/rand"
	"testing"
	"time"
)

func TestGooglePolyline(t *testing.T) {
	testCases := []struct {
		name             string
		coords           *Coordinates
		expectedPolyline string
	}{
		{
			name: "contoh polyline (https://developers.google.com/maps/documentation/utilities/polylinealgorithm)",
			coords: NewCoordinatesWithInitialValues([]Coordinate{
				NewCoordinate(38.5, -120.2),
				NewCoordinate(40.7, -120.95),
				NewCoordinate(43.252, -126.453),
			}),
			expectedPolyline: "_p~iF~ps|U_ulLnnqC_mqNvxq`@",
		},
		{
			/*
				enter
				 [[110.80038513680178,-7.554285198505083],[110.8007140023431,-7.554422883228756],[110.8009335894681,-7.5545265409446145],[110.8012856259149,-7.554650930172774],[110.80149127103199,-7.554754587832986],
				 [110.80177708286887,-7.554889342754199],[110.80288896053356,-7.555442183021017],[110.80320962744626,-7.554606011843249],[110.80352863552638,-7.553588163637354],[110.80377959224067,-7.552862557528317],[110.80259394816888
				,-7.5523549204738565],[110.80251029593077,-7.552337644102797],[110.80184456353597,-7.5525069525095505],[110.80108127449583,-7.552745369915522],[110.8008582018609,-7.552786833165814],[110.80078152064264,-7.552883580734364],
				[110.8006386147359,-7.552942320318985],[110.80050965086883,-7.5530839863431884],[110.80033188986289,-7.553180733845202],[110.80023778109502,-7.5533258550576425],[110.79956856319025,-7.553909794681649]]
				in https://developers.google.com/maps/documentation/utilities/polylineutility
			*/
			name: "polyline with 21 coordinates",
			coords: NewCoordinatesWithInitialValues([]Coordinate{
				NewCoordinate(-7.554285198505083, 110.80038513680178),
				NewCoordinate(-7.554422883228756, 110.8007140023431),
				NewCoordinate(-7.5545265409446145, 110.8009335894681),
				NewCoordinate(-7.554650930172774, 110.8012856259149),
				NewCoordinate(-7.554754587832986, 110.80149127103199),
				NewCoordinate(-7.554889342754199, 110.80177708286887),
				NewCoordinate(-7.555442183021017, 110.80288896053356),
				NewCoordinate(-7.554606011843249, 110.80320962744626),
				NewCoordinate(-7.553588163637354, 110.80352863552638),
				NewCoordinate(-7.552862557528317, 110.80377959224067),
				NewCoordinate(-7.5523549204738565, 110.80259394816888),
				NewCoordinate(-7.552337644102797, 110.80251029593077),
				NewCoordinate(-7.5525069525095505, 110.80184456353597),
				NewCoordinate(-7.552745369915522, 110.80108127449583),
				NewCoordinate(-7.552786833165814, 110.8008582018609),
				NewCoordinate(-7.552883580734364, 110.80078152064264),
				NewCoordinate(-7.552942320318985, 110.8006386147359),
				NewCoordinate(-7.5530839863431884, 110.80050965086883),
				NewCoordinate(-7.553180733845202, 110.80033188986289),
				NewCoordinate(-7.5533258550576425, 110.80023778109502),
				NewCoordinate(-7.553909794681649, 110.79956856319025),
			}),
			expectedPolyline: "hmbm@mugcTX_ATk@VgARg@Zy@lB}EeD_AkE_AqCq@eBlFAN`@dCn@vCFj@PNJZZXRb@\\PrBdC",
		},
	}

	forCoord := func(coords *Coordinates) iter.Seq[Coordinate] {
		return func(yield func(Coordinate) bool) {
			for _, c := range *coords {
				if !yield(c) {
					return
				}
			}
		}
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			_, _, _, _, expectedCap := capacity(forCoord(tc.coords), len(*tc.coords))
			got := GooglePolylineFromCoords(forCoord(tc.coords), len(*tc.coords))
			if got != tc.expectedPolyline {
				t.Errorf("expected %v, got %v", tc.expectedPolyline, got)
			}
			gotBytes := []byte(got)
			if cap(gotBytes) != expectedCap {
				t.Errorf("expected capacity %v, got %v", expectedCap, cap(gotBytes))
			}

			got = GooglePolylineFromCoords(forCoord(tc.coords), len(*tc.coords))
			if got != tc.expectedPolyline {
				t.Errorf("expected %v, got %v", tc.expectedPolyline, got)
			}
		})
	}

	// test random coordinate, cuma assert expected capacity
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	bb := NewBoundingBox(-8.2618, 110.132, -6.888, 110.9221)
	for i := 0; i < 1000; i++ {
		coords := NewCoordinatesWithCap(1000)
		for j := 0; j < 1000; j++ {
			coords.Append([]Coordinate{RandomCoordinate(bb, rd)})
		}

		_, _, _, _, expectedCap := capacity(forCoord(coords), len(*coords))
		got := GooglePolylineFromCoords(forCoord(coords), len(*coords))

		gotBytes := []byte(got)
		gotCap := cap(gotBytes)
		if gotCap != expectedCap {
			t.Errorf("expected capacity %v, got %v", expectedCap, gotCap)
		}
	}
}

func RandomCoordinate(bb *BoundingBox, rd *rand.Rand) Coordinate {

	lat := bb.GetMinLat() + rd.Float64()*(bb.GetMaxLat()-bb.GetMinLat())
	lon := bb.GetMinLon() + rd.Float64()*(bb.GetMaxLon()-bb.GetMinLon())
	return NewCoordinate(lat, lon)
}

// cd pkg/datastructure && go test -run=^$ -bench=^BenchmarkPolylineEncode$ -benchmem -cpuprofile polyline.cpu -memprofile polyline.mem -benchtime=15s -o polyline.test
// BenchmarkPolylineEncode benchmark
func BenchmarkPolylineEncode(b *testing.B) {
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	bb := NewBoundingBox(-8.2618, 110.132, -6.888, 110.9221)

	forCoord := func(coords *Coordinates) iter.Seq[Coordinate] {
		return func(yield func(Coordinate) bool) {
			for _, c := range *coords {
				if !yield(c) {
					return
				}
			}
		}
	}

	coords := NewCoordinatesWithCap(3000)
	for j := 0; j < 3000; j++ {
		coords.AppendCoordinate(RandomCoordinate(bb, rd))
	}
	start := time.Now()

	for b.Loop() {
		GooglePolylineFromCoords(forCoord(coords), len(*coords))
	}

	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(b.N)
	throughput := float64(b.N) / b.Elapsed().Seconds()

	b.ReportMetric(msPerOp, "ms/op")
	b.ReportMetric(throughput, "ops/sec")
}

func BenchmarkBytesToString(b *testing.B) {
	data := make([]byte, 1024)
	b.SetBytes(int64(len(data)))
	b.ReportAllocs()
	b.ResetTimer()

	for b.Loop() {
		bytesToString(data)
	}
}

func TestZigZagEncoding(t *testing.T) {
	testCases := []struct {
		name     string
		input    int32
		expected uint32
	}{ // test cases from: https://lemire.me/blog/2022/11/25/making-all-your-integers-positive-with-zigzag-encoding/
		{name: "-20", input: -20, expected: 39},
		{name: "-19", input: -19, expected: 37},
		{name: "-18", input: -18, expected: 35},
		{name: "-17", input: -17, expected: 33},
		{name: "-16", input: -16, expected: 31},
		{name: "-15", input: -15, expected: 29},
		{name: "-14", input: -14, expected: 27},
		{name: "-13", input: -13, expected: 25},
		{name: "-12", input: -12, expected: 23},
		{name: "-11", input: -11, expected: 21},
		{name: "-10", input: -10, expected: 19},
		{name: "-9", input: -9, expected: 17},
		{name: "-8", input: -8, expected: 15},
		{name: "-7", input: -7, expected: 13},
		{name: "-6", input: -6, expected: 11},
		{name: "-5", input: -5, expected: 9},
		{name: "-4", input: -4, expected: 7},
		{name: "-3", input: -3, expected: 5},
		{name: "-2", input: -2, expected: 3},
		{name: "-1", input: -1, expected: 1},
		{name: "0", input: 0, expected: 0},
		{name: "1", input: 1, expected: 2},
		{name: "2", input: 2, expected: 4},
		{name: "3", input: 3, expected: 6},
		{name: "4", input: 4, expected: 8},
		{name: "5", input: 5, expected: 10},
		{name: "6", input: 6, expected: 12},
		{name: "7", input: 7, expected: 14},
		{name: "8", input: 8, expected: 16},
		{name: "9", input: 9, expected: 18},
		{name: "10", input: 10, expected: 20},
		{name: "11", input: 11, expected: 22},
		{name: "12", input: 12, expected: 24},
		{name: "13", input: 13, expected: 26},
		{name: "14", input: 14, expected: 28},
		{name: "15", input: 15, expected: 30},
		{name: "16", input: 16, expected: 32},
		{name: "17", input: 17, expected: 34},
		{name: "18", input: 18, expected: 36},
		{name: "19", input: 19, expected: 38},
		{name: "20", input: 20, expected: 40},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			got := zigzagEncoding(tc.input)
			if got != tc.expected {
				t.Errorf("zigzagEncoding(%d) = %d, expected %d", tc.input, got, tc.expected)
			}
		})
	}
}
