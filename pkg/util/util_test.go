package util

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"io"
	"math"
	"reflect"
	"testing"
)

func TestReadLine(t *testing.T) {
	reader := bufio.NewReaderSize(bytes.NewBufferString("first line\r\nsecond line\nlonger than buffer"), 4)

	line, err := ReadLine(reader)
	if err != nil || line != "first line" {
		t.Fatalf("ReadLine() = %q, %v", line, err)
	}

	line, err = ReadLine(reader)
	if err != nil || line != "second line" {
		t.Fatalf("ReadLine() = %q, %v", line, err)
	}

	line, err = ReadLine(reader)
	if err != nil || line != "longer than buffer" {
		t.Fatalf("ReadLine() final line = %q, %v", line, err)
	}

	line, err = ReadLine(reader)
	if err != io.EOF || line != "" {
		t.Fatalf("ReadLine() at EOF = %q, %v", line, err)
	}
}

func TestFields(t *testing.T) {
	got := Fields("  alpha\tbeta  gamma ")
	want := []string{"alpha", "beta", "gamma"}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("Fields() = %q, want %q", got, want)
	}
}

func TestBinaryParsers(t *testing.T) {
	var buffer [8]byte

	int32Value := int32(-42)
	binary.LittleEndian.PutUint32(buffer[:4], uint32(int32Value))
	if value, err := ParseInt32(buffer[:4]); err != nil || value != -42 {
		t.Fatalf("ParseInt32() = %d, %v", value, err)
	}
	int64Value := int64(-42)
	binary.LittleEndian.PutUint64(buffer[:], uint64(int64Value))
	if value, err := ParseInt64(buffer[:]); err != nil || value != -42 {
		t.Fatalf("ParseInt64() = %d, %v", value, err)
	}
	buffer[0] = 255
	if value, err := ParseUInt8(buffer[:1]); err != nil || value != 255 {
		t.Fatalf("ParseUInt8() = %d, %v", value, err)
	}
	binary.LittleEndian.PutUint32(buffer[:4], math.MaxUint32)
	if value, err := ParseUInt32(buffer[:4]); err != nil || value != math.MaxUint32 {
		t.Fatalf("ParseUInt32() = %d, %v", value, err)
	}
	binary.LittleEndian.PutUint64(buffer[:], math.MaxUint64)
	if value, err := ParseUInt64(buffer[:]); err != nil || value != math.MaxUint64 {
		t.Fatalf("ParseUInt64() = %d, %v", value, err)
	}

	for _, value := range []float64{0, math.Copysign(0, -1), math.Inf(1), math.Inf(-1), math.NaN()} {
		binary.LittleEndian.PutUint64(buffer[:], math.Float64bits(value))
		got, err := ParseFloat64(buffer[:])
		if err != nil || math.Float64bits(got) != math.Float64bits(value) {
			t.Fatalf("ParseFloat64(%x) = %x, %v", math.Float64bits(value), math.Float64bits(got), err)
		}
	}
}

func TestBinaryParsersRejectMalformedBuffers(t *testing.T) {
	if _, err := ParseUInt32(make([]byte, 3)); err == nil {
		t.Fatal("ParseUInt32() accepted a short buffer")
	}
	if _, err := ParseFloat64(make([]byte, 9)); err == nil {
		t.Fatal("ParseFloat64() accepted a long buffer")
	}
	if _, err := ParseBool([]byte{2}); err == nil {
		t.Fatal("ParseBool() accepted an invalid boolean")
	}
}

func TestBinaryParsersDoNotAllocate(t *testing.T) {
	var buffer [8]byte
	allocs := testing.AllocsPerRun(1000, func() {
		_, _ = ParseInt64(buffer[:])
		_, _ = ParseUInt32(buffer[:4])
		_, _ = ParseFloat64(buffer[:])
	})
	if allocs != 0 {
		t.Fatalf("parsers allocated %f times per run, want 0", allocs)
	}
}
