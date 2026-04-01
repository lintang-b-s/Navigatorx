package datastructure

import (
	"math"
	"unsafe"
)

// Convert the decimal value to binary. Note that a negative value must be calculated using its two's complement
func decimalToBinary(x float64, negative bool) int {
	if negative { // golang -x udah two complement
		return int(-x)
	}

	return int(x)
}

// round. rounding float to nearest integer.
func round(x float64) int {
	if x < 0 {
		return decimalToBinary(math.Round(-x), true)
	}
	return decimalToBinary(math.Round(x), false)
}

// Take the decimal value and multiply it by 1e5, rounding the result:
func multiplyAndRound(x float64) int {
	return round(1e5 * x)
}

// https://developers.google.com/maps/documentation/utilities/polylinealgorithm
func DeltaEncode(buf []byte, coords []Coordinate) []byte {
	prevLat := 0
	prevLon := 0
	for _, coord := range coords {
		exLat := multiplyAndRound(coord.GetLat())
		u := leftShift(exLat - prevLat)
		buf = breakToChunkAndDoOrOp(buf, u)
		prevLat = exLat

		exLon := multiplyAndRound(coord.GetLon())
		u = leftShift(exLon - prevLon)
		buf = breakToChunkAndDoOrOp(buf, u)
		prevLon = exLon
	}
	return buf
}

// Left-shift the binary value one bit:
func leftShift(i int) uint {
	var u uint
	if i < 0 {
		// If the original decimal value is negative, invert this encoding:
		u = uint(^(i << 1))
	} else {
		u = uint(i << 1)
	}
	return u
}

// Break the binary value out into 5-bit chunks (starting from the right hand side):
// Place the 5-bit chunks into reverse order:
// OR each value with 0x20 if another bit chunk follows:
// Add 63 to each value:
func breakToChunkAndDoOrOp(buf []byte, u uint) []byte {
	// Break the binary value out into 5-bit chunks (starting from the right hand side):
	for u >= 32 {
		// OR each value with 0x20 if another bit chunk follows:
		buf = append(buf, byte((u&31)|0x20)+63)
		u >>= 5
	}

	buf = append(buf, byte(u)+63)

	return buf
}

func capacity(coords []Coordinate) int {
	total := 0
	prevLat, prevLon := 0, 0
	for _, c := range coords {
		exLat := multiplyAndRound(c.GetLat())
		exLon := multiplyAndRound(c.GetLon())
		total += encodedBytesLength(leftShift(exLat - prevLat))
		total += encodedBytesLength(leftShift(exLon - prevLon))
		prevLat = exLat
		prevLon = exLon
	}
	return total
}

func encodedBytesLength(u uint) int {
	length := 0
	for u >= 32 {
		u >>= 5
		length++
	}
	length++
	return length
}

// https://developers.google.com/maps/documentation/utilities/polylinealgorithm
// byte slice initialized using its exact capacity to prevent array doubling (https://go.dev/blog/slices-intro )
func GooglePoylineFromCoords(path []Coordinate) string {
	return bytesToString(DeltaEncode(make([]byte, 0, capacity(path)), path))
}

// https://go.dev/src/strings/builder.go#L45
func bytesToString(b []byte) string {
	return unsafe.String(unsafe.SliceData(b), len(b))
}
