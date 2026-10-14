package util

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"

	"github.com/bits-and-blooms/bitset"
	"github.com/klauspost/compress/s2"
)

const (
	SoftwareVersionMajor uint16 = 0
	SoftwareVersionMinor uint16 = 1
	SoftwareVersionPatch uint16 = 2
)

var magicNumber = [4]byte{'n', 'a', 'v', 'x'} // https://gist.github.com/leommoore/f9e57ba2aa4bf197ebc5

type BinaryWriter struct {
	w       io.Writer
	buf     [8]byte // reusable buffer
	bulkBuf []byte
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

func (w *BinaryWriter) Bitset(bitset *bitset.BitSet) error {
	_, err := bitset.WriteTo(w.w)
	return err
}

func (w *BinaryWriter) WriteFloat64s(values []float64) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	const itemSize = 8
	return w.writeBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			binary.LittleEndian.PutUint64(chunk[i:i+itemSize], math.Float64bits(values[offset+i/itemSize]))
		}
	})
}

func (w *BinaryWriter) WriteInt32s(values []int32) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	const itemSize = 4
	return w.writeBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			binary.LittleEndian.PutUint32(chunk[i:i+itemSize], uint32(values[offset+i/itemSize]))
		}
	})
}

func (w *BinaryWriter) WriteUint32s(values []uint32) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	const itemSize = 4
	return w.writeBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			binary.LittleEndian.PutUint32(chunk[i:i+itemSize], values[offset+i/itemSize])
		}
	})
}

func (w *BinaryWriter) WriteUint64s(values []uint64) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	const itemSize = 8
	return w.writeBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			binary.LittleEndian.PutUint64(chunk[i:i+itemSize], values[offset+i/itemSize])
		}
	})
}

func (w *BinaryWriter) WriteUInt64s(values []uint64) error {
	return w.WriteUint64s(values)
}

func (w *BinaryWriter) WriteInt64s(values []int64) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	const itemSize = 8
	return w.writeBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			binary.LittleEndian.PutUint64(chunk[i:i+itemSize], uint64(values[offset+i/itemSize]))
		}
	})
}

func (w *BinaryWriter) WriteUint16s(values []uint16) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	const itemSize = 2
	return w.writeBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			binary.LittleEndian.PutUint16(chunk[i:i+itemSize], values[offset+i/itemSize])
		}
	})
}

func (w *BinaryWriter) WriteInts(values []int) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	const itemSize = 8
	return w.writeBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			binary.LittleEndian.PutUint64(chunk[i:i+itemSize], uint64(values[offset+i/itemSize]))
		}
	})
}

// writeBulk writes slice of data with chunking strategy
// similiar to writeUint64Array() in https://github.com/bits-and-blooms/bitset/blob/master/bitset.go
func (w *BinaryWriter) writeBulk(count, itemSize int, encode func([]byte, int)) error {
	numItemsPerChunk := CHUNK_SIZE / itemSize
	if capacity := numItemsPerChunk * itemSize; cap(w.bulkBuf) < capacity {
		w.bulkBuf = make([]byte, capacity)
	}
	for offset := 0; offset < count; {
		n := min(numItemsPerChunk, count-offset)
		chunk := w.bulkBuf[:n*itemSize]
		encode(chunk, offset)
		if _, err := w.w.Write(chunk); err != nil {
			return err
		}
		offset += n
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

func EnsureDirExists(filename string) error {
	if err := os.MkdirAll(filepath.Dir(filename), 0700); err != nil {
		return fmt.Errorf("failed to create directory: %w", err)
	}
	return nil
}

func WriteCompressedFile(filename string, writePayload func(*BinaryWriter) error) error {
	err := EnsureDirExists(filename)
	if err != nil {
		return err
	}
	file, err := os.OpenFile(filename, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0600)
	if err != nil {
		return err
	}
	defer file.Close()

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
	return nil
}
