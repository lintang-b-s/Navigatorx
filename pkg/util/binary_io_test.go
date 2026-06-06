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

func TestCompressedArtifactRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "artifact.bin")
	if err := WriteCompressedArtifact(path, func(w *BinaryWriter) error {
		if err := w.Uint32(42); err != nil {
			return err
		}
		return w.String("navigatorx")
	}); err != nil {
		t.Fatal(err)
	}
	file, reader, err := OpenCompressedArtifact(path)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	value, err := reader.Uint32()
	if err != nil || value != 42 {
		t.Fatalf("Uint32() = %d, %v", value, err)
	}
	text, err := reader.String(32)
	if err != nil || text != "navigatorx" {
		t.Fatalf("String() = %q, %v", text, err)
	}
}

func TestBinaryWriterWriteFloat64s(t *testing.T) {
	values := []float64{1.5, math.Copysign(0, -1), math.Inf(1)}
	var output bytes.Buffer
	if err := NewBinaryWriter(&output).WriteFloat64s(values); err != nil {
		t.Fatal(err)
	}

	reader := NewBinaryReader(bytes.NewReader(output.Bytes()))
	length, err := reader.Length(uint32(len(values)))
	if err != nil {
		t.Fatal(err)
	}
	if length != uint32(len(values)) {
		t.Fatalf("length = %d, want %d", length, len(values))
	}
	got := make([]float64, length)
	if err := reader.ReadFloat64s(got); err != nil {
		t.Fatal(err)
	}
	for i := range values {
		if math.Float64bits(got[i]) != math.Float64bits(values[i]) {
			t.Fatalf("value[%d] bits = %#x, want %#x", i, math.Float64bits(got[i]), math.Float64bits(values[i]))
		}
	}
}

func TestCompressedArtifactHeader(t *testing.T) {
	path := filepath.Join(t.TempDir(), "artifact.bin")
	if err := WriteCompressedArtifact(path, func(*BinaryWriter) error { return nil }); err != nil {
		t.Fatal(err)
	}
	file, err := os.Open(path)
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

func TestCompressedArtifactRejectsInvalidEnvelope(t *testing.T) {
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
			_, _, err := OpenCompressedArtifact(path)
			if err == nil || !strings.Contains(err.Error(), tt.want) {
				t.Fatalf("OpenCompressedArtifact() error = %v, want %q", err, tt.want)
			}
		})
	}
}

func TestCompressedArtifactRejectsTruncatedPayload(t *testing.T) {
	path := filepath.Join(t.TempDir(), "artifact.bin")
	if err := WriteCompressedArtifact(path, func(w *BinaryWriter) error {
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
	file, reader, err := OpenCompressedArtifact(path)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	if _, err := reader.Uint64(); err == nil {
		t.Fatal("truncated payload was accepted")
	}
}
