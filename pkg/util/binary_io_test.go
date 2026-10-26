package util

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestCompressedFileRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "artifact.bin")
	if err := WriteCompressedFile(path, func(w *BinaryWriter) error {
		if err := w.Uint32(42); err != nil {
			return err
		}
		return w.String("navigatorx")
	}); err != nil {
		t.Fatal(err)
	}
	file, reader, err := OpenCompressedFile(path)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	value, err := reader.Uint32()
	if err != nil || value != 42 {
		t.Fatalf("Uint32() = %d, %v", value, err)
	}
	text, err := reader.String()
	if err != nil || text != "navigatorx" {
		t.Fatalf("String() = %q, %v", text, err)
	}
}

func TestBinaryWriterWriteFloat64s(t *testing.T) {
	tests := [][]float64{
		{},
		{1.5, math.Copysign(0, -1), math.Inf(1), math.Inf(-1), -123.456},
		make([]float64, (CHUNK_SIZE/8)+25),
	}
	for i := range tests[2] {
		tests[2][i] = float64(i) * 1.5
	}

	for _, values := range tests {
		var output bytes.Buffer
		if err := NewBinaryWriter(&output).WriteFloat64s(values); err != nil {
			t.Fatal(err)
		}

		reader := NewBinaryReader(bytes.NewReader(output.Bytes()))
		got, err := reader.ReadFloat64s()
		if err != nil {
			t.Fatal(err)
		}
		if len(got) != len(values) {
			t.Fatalf("got len %d, want %d", len(got), len(values))
		}
		for i := range values {
			if math.Float64bits(got[i]) != math.Float64bits(values[i]) {
				t.Fatalf("value[%d] bits = %#x, want %#x", i, math.Float64bits(got[i]), math.Float64bits(values[i]))
			}
		}
	}
}

func testRoundTripSlice[T comparable](
	t *testing.T,
	writeFn func(*BinaryWriter, []T) error,
	readFn func(*BinaryReader) ([]T, error),
	tests [][]T,
) {
	t.Helper()
	for _, values := range tests {
		var output bytes.Buffer
		if err := writeFn(NewBinaryWriter(&output), values); err != nil {
			t.Fatal(err)
		}

		reader := NewBinaryReader(bytes.NewReader(output.Bytes()))
		got, err := readFn(reader)
		if err != nil {
			t.Fatal(err)
		}
		if len(got) != len(values) {
			t.Fatalf("got len %d, want %d", len(got), len(values))
		}
		for i := range values {
			if got[i] != values[i] {
				t.Fatalf("value[%d] = %v, want %v", i, got[i], values[i])
			}
		}
	}
}

func TestBinaryWriterWriteInt32s(t *testing.T) {
	tests := [][]int32{
		{},
		{0, -1, 1, math.MinInt32, math.MaxInt32},
		make([]int32, (CHUNK_SIZE/4)+25),
	}
	for i := range tests[2] {
		tests[2][i] = int32(i*3 - 100)
	}

	testRoundTripSlice(t, (*BinaryWriter).WriteInt32s, (*BinaryReader).ReadInt32s, tests)
}

func TestBinaryWriterWriteUint32s(t *testing.T) {
	tests := [][]uint32{
		{},
		{0, 1, 42, math.MaxUint32},
		make([]uint32, (CHUNK_SIZE/4)+25),
	}
	for i := range tests[2] {
		tests[2][i] = uint32(i * 7)
	}

	testRoundTripSlice(t, (*BinaryWriter).WriteUint32s, (*BinaryReader).ReadUint32s, tests)
}

func TestBinaryWriterWriteUint64s(t *testing.T) {
	tests := [][]uint64{
		{},
		{0, 1, math.MaxUint64},
		make([]uint64, (CHUNK_SIZE/8)+25),
	}
	for i := range tests[2] {
		tests[2][i] = uint64(i * 13)
	}

	testRoundTripSlice(t, (*BinaryWriter).WriteUint64s, (*BinaryReader).ReadUint64s, tests)
	testRoundTripSlice(t, (*BinaryWriter).WriteUInt64s, (*BinaryReader).ReadUInt64s, tests)
}

func TestBinaryWriterWriteInt64s(t *testing.T) {
	tests := [][]int64{
		{},
		{0, -1, 1, math.MinInt64, math.MaxInt64},
		make([]int64, (CHUNK_SIZE/8)+25),
	}
	for i := range tests[2] {
		tests[2][i] = int64(i*17 - 500)
	}

	testRoundTripSlice(t, (*BinaryWriter).WriteInt64s, (*BinaryReader).ReadInt64s, tests)
}

func TestBinaryWriterWriteUint16s(t *testing.T) {
	tests := [][]uint16{
		{},
		{0, 1, 42, math.MaxUint16},
		make([]uint16, (CHUNK_SIZE/2)+25),
	}
	for i := range tests[2] {
		tests[2][i] = uint16(i * 3)
	}

	testRoundTripSlice(t, (*BinaryWriter).WriteUint16s, (*BinaryReader).ReadUint16s, tests)
}

func TestBinaryWriterWriteInts(t *testing.T) {
	tests := [][]int{
		{},
		{0, -1, 1, -123456789, 123456789},
		make([]int, (CHUNK_SIZE/8)+25),
	}
	for i := range tests[2] {
		tests[2][i] = i*19 - 1000
	}

	for _, values := range tests {
		var output bytes.Buffer
		if err := NewBinaryWriter(&output).WriteInts(values); err != nil {
			t.Fatal(err)
		}

		reader := NewBinaryReader(bytes.NewReader(output.Bytes()))
		got, err := reader.ReadInts()
		if err != nil {
			t.Fatal(err)
		}
		if len(got) != len(values) {
			t.Fatalf("got len %d, want %d", len(got), len(values))
		}
		for i := range values {
			if got[i] != values[i] {
				t.Fatalf("value[%d] = %d, want %d", i, got[i], values[i])
			}
		}
	}
}

func TestCompressedFileHeader(t *testing.T) {
	path := filepath.Join(t.TempDir(), "artifact.bin")
	if err := WriteCompressedFile(path, func(*BinaryWriter) error { return nil }); err != nil {
		t.Fatal(err)
	}
	file, err := os.Open(filepath.Clean(path))
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	header := make([]byte, 10)
	if _, err := io.ReadFull(file, header); err != nil {
		t.Fatal(err)
	}
	want := make([]byte, 10)
	copy(want, magicNumber[:])
	binary.LittleEndian.PutUint16(want[4:6], SoftwareVersionMajor)
	binary.LittleEndian.PutUint16(want[6:8], SoftwareVersionMinor)
	binary.LittleEndian.PutUint16(want[8:10], SoftwareVersionPatch)
	if !bytes.Equal(header, want) {
		t.Fatalf("header = %v, want %v", header, want)
	}
}

func TestCompressedFileRejectsInvalidEnvelope(t *testing.T) {
	tests := []struct {
		name   string
		header func([]byte)
		want   string
	}{
		{name: "magic", header: func(header []byte) { header[0] = 'X' }, want: "legacy or invalid"},
		{name: "major", header: func(header []byte) { binary.LittleEndian.PutUint16(header[4:6], 99) }, want: fmt.Sprintf("99.%d.%d", SoftwareVersionMinor, SoftwareVersionPatch)},
		{name: "minor", header: func(header []byte) { binary.LittleEndian.PutUint16(header[6:8], 99) }, want: fmt.Sprintf("%d.99.%d", SoftwareVersionMajor, SoftwareVersionPatch)},
		{name: "patch", header: func(header []byte) { binary.LittleEndian.PutUint16(header[8:10], 99) }, want: fmt.Sprintf("%d.%d.99", SoftwareVersionMajor, SoftwareVersionMinor)},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			header := make([]byte, 10)
			copy(header, magicNumber[:])
			binary.LittleEndian.PutUint16(header[4:6], SoftwareVersionMajor)
			binary.LittleEndian.PutUint16(header[6:8], SoftwareVersionMinor)
			binary.LittleEndian.PutUint16(header[8:10], SoftwareVersionPatch)
			tt.header(header)
			path := filepath.Join(t.TempDir(), "artifact.bin")
			if err := os.WriteFile(path, header, 0600); err != nil {
				t.Fatal(err)
			}
			_, _, err := OpenCompressedFile(path)
			if err == nil || !strings.Contains(err.Error(), tt.want) {
				t.Fatalf("OpenCompressedFile() error = %v, want %q", err, tt.want)
			}
		})
	}
}

func TestCompressedFileRejectsTruncatedPayload(t *testing.T) {
	path := filepath.Join(t.TempDir(), "artifact.bin")
	if err := WriteCompressedFile(path, func(w *BinaryWriter) error {
		return w.Uint64(42)
	}); err != nil {
		t.Fatal(err)
	}
	info, err := os.Stat(path)
	if err != nil {
		t.Fatal(err)
	}
	if err := os.Truncate(path, info.Size()-1); err != nil {
		t.Fatal(err)
	}
	file, reader, err := OpenCompressedFile(path)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	if _, err := reader.Uint64(); err == nil {
		t.Fatal("truncated payload was accepted")
	}
}
