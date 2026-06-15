package util

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"

	"github.com/klauspost/compress/s2"
)

const (
	SoftwareVersionMajor uint16 = 0
	SoftwareVersionMinor uint16 = 1
	SoftwareVersionPatch uint16 = 1
)

var magicNumber = [4]byte{'n', 'a', 'v', 'x'} // https://gist.github.com/leommoore/f9e57ba2aa4bf197ebc5

type BinaryWriter struct {
	w   io.Writer
	buf [8]byte // reusable buffer
}

func NewBinaryWriter(w io.Writer) *BinaryWriter {
	return &BinaryWriter{w: w}
}

func (w *BinaryWriter) Bytes(value []byte) error {
	_, err := w.w.Write(value)
	return err
}

func (w *BinaryWriter) Uint8(value uint8) error {
	w.buf[0] = value
	_, err := w.w.Write(w.buf[:1])
	return err
}

func (w *BinaryWriter) Bool(value bool) error {
	if value {
		return w.Uint8(1)
	}
	return w.Uint8(0)
}

func (w *BinaryWriter) Uint16(value uint16) error {
	binary.LittleEndian.PutUint16(w.buf[:2], value)
	_, err := w.w.Write(w.buf[:2])
	return err
}

func (w *BinaryWriter) Uint32(value uint32) error {
	binary.LittleEndian.PutUint32(w.buf[:4], value)
	_, err := w.w.Write(w.buf[:4])
	return err
}

func (w *BinaryWriter) Int32(value int32) error {
	return w.Uint32(uint32(value))
}

func (w *BinaryWriter) Uint64(value uint64) error {
	binary.LittleEndian.PutUint64(w.buf[:8], value)
	_, err := w.w.Write(w.buf[:8])
	return err
}

func (w *BinaryWriter) Int64(value int64) error {
	return w.Uint64(uint64(value))
}

func (w *BinaryWriter) Float64(value float64) error {
	return w.Uint64(math.Float64bits(value))
}

func (w *BinaryWriter) WriteFloat64s(values []float64) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	for _, value := range values {
		if err := w.Float64(value); err != nil {
			return err
		}
	}
	return nil
}

func (w *BinaryWriter) WriteInt32s(values []int32) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	for _, value := range values {
		if err := w.Int32(value); err != nil {
			return err
		}
	}
	return nil
}

func (w *BinaryWriter) WriteUint32s(values []uint32) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	for _, value := range values {
		if err := w.Uint32(value); err != nil {
			return err
		}
	}
	return nil
}

func (w *BinaryWriter) WriteUint16s(values []uint16) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	for _, value := range values {
		if err := w.Uint16(value); err != nil {
			return err
		}
	}
	return nil
}

func (w *BinaryWriter) Length(length int) error {
	if uint64(length) > math.MaxUint32 {
		return fmt.Errorf("length %d exceeds uint32", length)
	}
	return w.Uint32(uint32(length))
}

func (w *BinaryWriter) Blob(value []byte) error {
	if err := w.Length(len(value)); err != nil {
		return err
	}
	return w.Bytes(value)
}

func (w *BinaryWriter) String(value string) error {
	return w.Blob([]byte(value))
}

type BinaryReader struct {
	r *bufio.Reader
	// reusable buffer
	buf     [8]byte
	bulkBuf []byte
}

func NewBinaryReader(r io.Reader) *BinaryReader {
	return NewBinaryReaderSize(r, BUFIO_SIZE)
}

func NewBinaryReaderSize(r io.Reader, size int) *BinaryReader {
	if buffered, ok := r.(*bufio.Reader); ok {
		return &BinaryReader{r: buffered}
	}
	return &BinaryReader{r: bufio.NewReaderSize(r, size)}
}

func (r *BinaryReader) read(size int) ([]byte, error) {
	_, err := io.ReadFull(r.r, r.buf[:size])
	return r.buf[:size], err
}

func (r *BinaryReader) Uint8() (uint8, error) {
	value, err := r.read(1)
	if err != nil {
		return 0, err
	}
	return ParseUInt8(value)
}

func (r *BinaryReader) Bool() (bool, error) {
	value, err := r.read(1)
	if err != nil {
		return false, err
	}
	return ParseBool(value)
}

func (r *BinaryReader) Uint16() (uint16, error) {
	value, err := r.read(2)
	if err != nil {
		return 0, err
	}
	return binary.LittleEndian.Uint16(value), nil
}

func (r *BinaryReader) Uint32() (uint32, error) {
	value, err := r.read(4)
	if err != nil {
		return 0, err
	}
	return ParseUInt32(value)
}

func (r *BinaryReader) Int32() (int32, error) {
	value, err := r.read(4)
	if err != nil {
		return 0, err
	}
	return ParseInt32(value)
}

func (r *BinaryReader) Uint64() (uint64, error) {
	value, err := r.read(8)
	if err != nil {
		return 0, err
	}
	return ParseUInt64(value)
}

func (r *BinaryReader) Int64() (int64, error) {
	value, err := r.read(8)
	if err != nil {
		return 0, err
	}
	return ParseInt64(value)
}

func (r *BinaryReader) Float64() (float64, error) {
	value, err := r.read(8)
	if err != nil {
		return 0, err
	}
	return ParseFloat64(value)
}

func (r *BinaryReader) ReadInt32s(values []int32) error {
	const itemSize = 4
	return r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = int32(binary.LittleEndian.Uint32(chunk[i : i+itemSize]))
		}
	})
}

func (r *BinaryReader) ReadUint16s(values []uint16) error {
	for i := range values {
		value, err := r.Uint16()
		if err != nil {
			return err
		}
		values[i] = value
	}
	return nil
}

func (r *BinaryReader) ReadInt32Pairs(count int, set func(index int, first, second int32)) error {
	const itemSize = 8
	return r.readBulk(count, itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			set(
				offset+i/itemSize,
				int32(binary.LittleEndian.Uint32(chunk[i:i+4])),
				int32(binary.LittleEndian.Uint32(chunk[i+4:i+itemSize])),
			)
		}
	})
}

func (r *BinaryReader) ReadUint32s(values []uint32) error {
	const itemSize = 4
	return r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = binary.LittleEndian.Uint32(chunk[i : i+itemSize])
		}
	})
}

func (r *BinaryReader) ReadUint64s(values []uint64) error {
	const itemSize = 8
	return r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = binary.LittleEndian.Uint64(chunk[i : i+itemSize])
		}
	})
}

func (r *BinaryReader) ReadFloat64s(values []float64) error {
	const itemSize = 8
	return r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = math.Float64frombits(binary.LittleEndian.Uint64(chunk[i : i+itemSize]))
		}
	})
}

func (r *BinaryReader) readBulk(count, itemSize int, decode func([]byte, int)) error {
	numItemsPerChunk := targetChunkSize / itemSize
	if capacity := numItemsPerChunk * itemSize; cap(r.bulkBuf) < capacity {
		r.bulkBuf = make([]byte, capacity)
	}
	for offset := 0; offset < count; {
		n := min(numItemsPerChunk, count-offset)
		chunk := r.bulkBuf[:n*itemSize]
		if _, err := io.ReadFull(r.r, chunk); err != nil {
			return err
		}
		decode(chunk, offset)
		offset += n
	}
	return nil
}

func (r *BinaryReader) Length(max uint32) (uint32, error) {
	length, err := r.Uint32()
	if err != nil {
		return 0, err
	}
	if length > max {
		return 0, fmt.Errorf("length %d exceeds maximum %d", length, max)
	}
	return length, nil
}

func (r *BinaryReader) Blob(max uint32) ([]byte, error) {
	length, err := r.Length(max)
	if err != nil {
		return nil, err
	}
	value := make([]byte, length)
	_, err = io.ReadFull(r.r, value)
	return value, err
}

func (r *BinaryReader) String(max uint32) (string, error) {
	value, err := r.Blob(max)
	if err != nil {
		return "", err
	}
	return string(value), nil
}

func WriteCompressedArtifact(filename string, writePayload func(*BinaryWriter) error) error {
	dir := filepath.Dir(filename)
	if err := os.MkdirAll(dir, 0755); err != nil {
		return err
	}
	file, err := os.CreateTemp(dir, "."+filepath.Base(filename)+".tmp-*")
	if err != nil {
		return err
	}
	defer file.Close()
	tempName := file.Name()
	succeeded := false
	defer func() {
		if !succeeded {
			_ = os.Remove(tempName)
		}
	}()

	header := NewBinaryWriter(file)
	if err := header.Bytes(magicNumber[:]); err != nil {
		_ = file.Close()
		return err
	}
	if err := header.Uint16(SoftwareVersionMajor); err != nil {
		_ = file.Close()
		return err
	}
	if err := header.Uint16(SoftwareVersionMinor); err != nil {
		_ = file.Close()
		return err
	}
	if err := header.Uint16(SoftwareVersionPatch); err != nil {
		_ = file.Close()
		return err
	}

	compressed := s2.NewWriter(file)
	buffered := bufio.NewWriterSize(compressed, BUFIO_SIZE)
	if err := writePayload(NewBinaryWriter(buffered)); err != nil {
		_ = compressed.Close()
		return err
	}
	if err := buffered.Flush(); err != nil {
		_ = compressed.Close()
		return err
	}
	if err := compressed.Close(); err != nil {
		_ = file.Close()
		return err
	}
	if err := file.Sync(); err != nil {
		_ = file.Close()
		return err
	}
	if err := file.Close(); err != nil {
		return err
	}
	if err := os.Rename(tempName, filename); err != nil {
		return err
	}
	succeeded = true
	return nil
}

func OpenCompressedArtifact(filename string) (*os.File, *BinaryReader, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, nil, err
	}
	fail := func(format string, args ...any) (*os.File, *BinaryReader, error) {
		_ = file.Close()
		return nil, nil, fmt.Errorf(format, args...)
	}
	var header [10]byte
	if _, err := io.ReadFull(file, header[:]); err != nil {
		return fail("%s is not a NavigatorX binary artifact; regenerate it: %w", filename, err)
	}
	var magic [4]byte
	copy(magic[:], header[:4])
	if magic != magicNumber {
		return fail("%s is a legacy or invalid artifact; regenerate it with the preprocessor/customizer", filename)
	}
	major := binary.LittleEndian.Uint16(header[4:6])
	minor := binary.LittleEndian.Uint16(header[6:8])
	patch := binary.LittleEndian.Uint16(header[8:10])
	if major != SoftwareVersionMajor || minor != SoftwareVersionMinor || patch != SoftwareVersionPatch {
		return fail(
			"%s was written by NavigatorX %d.%d.%d, expected %d.%d.%d; regenerate it",
			filename,
			major,
			minor,
			patch,
			SoftwareVersionMajor,
			SoftwareVersionMinor,
			SoftwareVersionPatch,
		)
	}
	return file, NewBinaryReader(s2.NewReader(file)), nil
}
