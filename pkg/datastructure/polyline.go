package datastructure

import (
	"iter"
	"math/bits"
	"sync"
	"unsafe"
)

// zigzagEncoding is zigzag encoding, to make all integers positive & small. with negative integers become odd integers, and positive integers become even integers.
// https://lemire.me/blog/2022/11/25/making-all-your-integers-positive-with-zigzag-encoding/
// because golang int32 is already represented as two complement, if x negative, right shift will fill the missing msb bits as 1's
// so (2*x) ^ (x >> 31) will result in (2*x) ^ (-1) or (2*x) ^ (0xFFFFFFFF) which is same as ^(2*x)=-(2x)-1 which converts the negative x into 2*abs(x)-1
func zigzagEncoding(x int32) uint32 {
	return uint32((2 * x) ^ (x >> (int32Size - 1)))
}

// Break the binary value out into 5-bit chunks (starting from the right hand side):
// Place the 5-bit chunks into reverse order:
// OR each value with 0x20 if another bit chunk follows:
// Add 63 to each value, such that it become ascii characters.
func breakToChunkAndDoOrOp(buf []byte, u uint32) []byte {
	// Break the binary value out into 5-bit chunks (starting from the right hand side):
	for u >= 32 {
		// OR each value with 0x20 if another bit chunk follows:
		// Add 63 to each value:
		buf = append(buf, byte((u&31)|0x20)+63)
		u >>= 5
	}
	// Add 63 to each value:
	buf = append(buf, byte(u)+63)
	return buf
}

// https://go.dev/src/strings/builder.go#L45
func bytesToString(b []byte) string {
	return unsafe.String(unsafe.SliceData(b), len(b)) //nolint:gosec
}

// encodedBytesLength computes the number of 5-bit chunks.
func encodedBytesLength(u uint32) int {
	u |= 1
	b := bits.Len32(u) // number of bit to represent u
	// do ceil(b/5)
	q := b / 5
	if b%5 != 0 {
		q++
	}
	return int(q)
}

// capacity precomputes capacity of the buffer containing polyline string and precomputes delta & zigzag encoded (lat,lon) values..
func capacity(coords iter.Seq[Coordinate], n int) (*[]uint32, *[]uint32, []uint32, []uint32, int) {
	deltaLatsPtr := getCoords(n)
	deltaLonsPtr := getCoords(n)
	deltaLats := (*deltaLatsPtr)[:n]
	deltaLons := (*deltaLonsPtr)[:n]

	total := 0
	prevLat, prevLon := int32(0), int32(0)
	i := 0
	for c := range coords {
		exLat := c.LatE5()
		exLon := c.LonE5()
		dLat := exLat - prevLat
		dLon := exLon - prevLon
		zdLat := zigzagEncoding(dLat)
		zdLon := zigzagEncoding(dLon)
		deltaLats[i] = zdLat
		deltaLons[i] = zdLon
		total += encodedBytesLength(zdLat)
		total += encodedBytesLength(zdLon)
		prevLat = exLat
		prevLon = exLon
		i++
	}
	return deltaLatsPtr, deltaLonsPtr, deltaLats, deltaLons, total
}

// deltaEncode computes delta encoding of google polyline compression.
// https://developers.google.com/maps/documentation/utilities/polylinealgorithm
// other good explanation: https://diragb.dev/blog/polyline-algorithm/
// in any routes in road networks, consecutive coordinates of the route have small deltas,
// we can computes delta encoding of them so we can represent the deltas using fewer ascii characters..
func deltaEncode(buf []byte, deltaLats []uint32, deltaLons []uint32) []byte {
	for i := range deltaLats {
		dLat := deltaLats[i]
		dLon := deltaLons[i]
		buf = breakToChunkAndDoOrOp(buf, dLat)
		buf = breakToChunkAndDoOrOp(buf, dLon)
	}
	return buf
}

// GooglePolylineFromCoords compress route path coordinates using google polyline compression algorithm.
func GooglePolylineFromCoords(path iter.Seq[Coordinate], n int) string {
	deltaLatsPtr, deltaLonsPtr, deltaLats, deltaLons, pcap := capacity(path, n)
	buf := make([]byte, 0, pcap) // precomputes the exact size of the buffer, so we get zero allocation slice append() function  in deltaEncode
	buf = deltaEncode(buf, deltaLats, deltaLons)
	putCoords(deltaLatsPtr)
	putCoords(deltaLonsPtr)
	return bytesToString(buf)
}

const (
	int32Size = 32
)

var (
	// inspired by http/2 sync.pool example in: https://wundergraph.com/blog/golang-sync-pool#3.-complexity-trade-offs
	coordsSizeClasses = [9]int{
		1 << 7,
		1 << 8,
		1 << 9,
		1 << 10,
		1 << 11,
		1 << 12,
		1 << 13,
		1 << 14,
		1 << 15,
	}

	coordPools [9]sync.Pool = [9]sync.Pool{
		{New: func() interface{} {
			s := make([]uint32, 1<<7)
			return &s
		}},
		{New: func() interface{} {
			s := make([]uint32, 1<<8)
			return &s
		}},
		{New: func() interface{} {
			s := make([]uint32, 1<<9)
			return &s
		}},
		{New: func() interface{} {
			s := make([]uint32, 1<<10)
			return &s
		}},
		{New: func() interface{} {
			s := make([]uint32, 1<<11)
			return &s
		}},
		{New: func() interface{} {
			s := make([]uint32, 1<<12)
			return &s
		}},
		{New: func() interface{} {
			s := make([]uint32, 1<<13)
			return &s
		}},
		{New: func() interface{} {
			s := make([]uint32, 1<<14)
			return &s
		}},
		{New: func() interface{} {
			s := make([]uint32, 1<<15)
			return &s
		}},
	}
)

func getCoords(size int) *[]uint32 {
	for i := range 9 {
		if size <= coordsSizeClasses[i] {
			return coordPools[i].Get().(*[]uint32)
		}
	}
	s := make([]uint32, size)
	return &s
}

func putCoords(sp *[]uint32) {
	c := cap(*sp)
	for i := range 9 {
		if c == coordsSizeClasses[i] {
			coordPools[i].Put(sp)
			return
		}
	}
}
