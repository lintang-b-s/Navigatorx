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

// BinaryReader wraps a buffered reader for sequential binary deserialization.
type BinaryReader struct {
	r *bufio.Reader
	// reusable buffer
	buf     [8]byte
	bulkBuf []byte
}

// NewBinaryReader creates a new BinaryReader with default buffer size.
func NewBinaryReader(r io.Reader) *BinaryReader {
	return NewBinaryReaderSize(r, BufioSize)
}

// NewBinaryReaderSize creates a new BinaryReader with a custom buffer size.
func NewBinaryReaderSize(r io.Reader, size int) *BinaryReader {
	if buffered, ok := r.(*bufio.Reader); ok {
		return &BinaryReader{r: buffered}
	}
	return &BinaryReader{r: bufio.NewReaderSize(r, size)}
}

// GetReader returns the underlying bufio.Reader.
func (r *BinaryReader) GetReader() *bufio.Reader {
	return r.r
}

func (r *BinaryReader) read(size int) ([]byte, error) {
	_, err := io.ReadFull(r.r, r.buf[:size])
	return r.buf[:size], err
}

// Uint8 reads and returns a single uint8 value.
func (r *BinaryReader) Uint8() (uint8, error) {
	value, err := r.read(1)
	if err != nil {
		return 0, err
	}
	return ParseUInt8(value)
}

// Bool reads and returns a single boolean value.
func (r *BinaryReader) Bool() (bool, error) {
	value, err := r.read(1)
	if err != nil {
		return false, err
	}
	return ParseBool(value)
}

// Uint16 reads and returns a little-endian uint16 value.
func (r *BinaryReader) Uint16() (uint16, error) {
	value, err := r.read(2)
	if err != nil {
		return 0, err
	}
	return binary.LittleEndian.Uint16(value), nil
}

// Uint32 reads and returns a little-endian uint32 value.
func (r *BinaryReader) Uint32() (uint32, error) {
	value, err := r.read(4)
	if err != nil {
		return 0, err
	}
	return ParseUInt32(value)
}

// Int32 reads and returns a little-endian int32 value.
func (r *BinaryReader) Int32() (int32, error) {
	value, err := r.read(4)
	if err != nil {
		return 0, err
	}
	return ParseInt32(value)
}

// Uint64 reads and returns a little-endian uint64 value.
func (r *BinaryReader) Uint64() (uint64, error) {
	value, err := r.read(8)
	if err != nil {
		return 0, err
	}
	return ParseUInt64(value)
}

// Int64 reads and returns a little-endian int64 value.
func (r *BinaryReader) Int64() (int64, error) {
	value, err := r.read(8)
	if err != nil {
		return 0, err
	}
	return ParseInt64(value)
}

// Float64 reads and returns an 8-byte float64 value.
func (r *BinaryReader) Float64() (float64, error) {
	value, err := r.read(8)
	if err != nil {
		return 0, err
	}
	return ParseFloat64(value)
}

// ReadBitset reads and deserializes a bitset.
func (r *BinaryReader) ReadBitset() (*bitset.BitSet, error) {
	bb := bitset.New(100)
	_, err := bb.ReadFrom(r.r)
	return bb, err
}

// ReadInt32s reads a length-prefixed slice of int32 values.
func (r *BinaryReader) ReadInt32s() ([]int32, error) {
	length, err := r.Length()
	if err != nil {
		return make([]int32, 0), err
	}
	values := make([]int32, length)
	const itemSize = 4
	err = r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = int32(binary.LittleEndian.Uint32(chunk[i : i+itemSize]))
		}
	})

	if err != nil {
		return make([]int32, 0), err
	}
	return values, nil
}

// ReadUint8s reads a length-prefixed slice of uint8 values.
func (r *BinaryReader) ReadUint8s() ([]uint8, error) {
	length, err := r.Length()
	if err != nil {
		return make([]uint8, 0), err
	}
	values := make([]uint8, length)
	const itemSize = 1
	err = r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = uint8(chunk[i])
		}
	})

	if err != nil {
		return make([]uint8, 0), err
	}
	return values, nil
}

// ReadUint16s reads a length-prefixed slice of uint16 values.
func (r *BinaryReader) ReadUint16s() ([]uint16, error) {
	length, err := r.Length()
	if err != nil {
		return make([]uint16, 0), err
	}
	values := make([]uint16, length)
	const itemSize = 2
	err = r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = binary.LittleEndian.Uint16(chunk[i : i+itemSize])
		}
	})

	if err != nil {
		return make([]uint16, 0), err
	}
	return values, nil
}

// ReadUint32s reads a length-prefixed slice of uint32 values.
func (r *BinaryReader) ReadUint32s() ([]uint32, error) {
	length, err := r.Length()
	if err != nil {
		return make([]uint32, 0), err
	}
	values := make([]uint32, length)
	const itemSize = 4
	err = r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = binary.LittleEndian.Uint32(chunk[i : i+itemSize])
		}
	})
	if err != nil {
		return make([]uint32, 0), err
	}
	return values, nil
}

// ReadUint64s reads a length-prefixed slice of uint64 values.
func (r *BinaryReader) ReadUint64s() ([]uint64, error) {
	length, err := r.Length()
	if err != nil {
		return make([]uint64, 0), err
	}
	values := make([]uint64, length)
	const itemSize = 8
	err = r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = binary.LittleEndian.Uint64(chunk[i : i+itemSize])
		}
	})
	if err != nil {
		return make([]uint64, 0), err
	}
	return values, nil
}

// ReadUInt64s is an alias for ReadUint64s.
func (r *BinaryReader) ReadUInt64s() ([]uint64, error) {
	return r.ReadUint64s()
}

// ReadInt64s reads a length-prefixed slice of int64 values.
func (r *BinaryReader) ReadInt64s() ([]int64, error) {
	length, err := r.Length()
	if err != nil {
		return make([]int64, 0), err
	}
	values := make([]int64, length)
	const itemSize = 8
	err = r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = int64(binary.LittleEndian.Uint64(chunk[i : i+itemSize]))
		}
	})
	if err != nil {
		return make([]int64, 0), err
	}
	return values, nil
}

// ReadInts reads a length-prefixed slice of int values.
func (r *BinaryReader) ReadInts() ([]int, error) {
	length, err := r.Length()
	if err != nil {
		return make([]int, 0), err
	}
	values := make([]int, length)
	const itemSize = 8
	err = r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = int(binary.LittleEndian.Uint64(chunk[i : i+itemSize]))
		}
	})
	if err != nil {
		return make([]int, 0), err
	}
	return values, nil
}

// ReadFloat64s reads a length-prefixed slice of float64 values.
func (r *BinaryReader) ReadFloat64s() ([]float64, error) {
	length, err := r.Length()
	if err != nil {
		return make([]float64, 0), err
	}
	values := make([]float64, length)
	const itemSize = 8
	err = r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = math.Float64frombits(binary.LittleEndian.Uint64(chunk[i : i+itemSize]))
		}
	})
	if err != nil {
		return make([]float64, 0), err
	}
	return values, nil
}

// ReadInt32Pairs reads count pairs of int32 values using a chunked buffer.
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

// readBulk read slice of data with chunking strategy
// similiar to readUint64Array() in https://github.com/bits-and-blooms/bitset/blob/master/bitset.go
func (r *BinaryReader) readBulk(count, itemSize int, decode func([]byte, int)) error {
	numItemsPerChunk := ChunkSize / itemSize
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

// Length reads a 32-bit length prefix.
func (r *BinaryReader) Length() (uint32, error) {
	length, err := r.Uint32()
	if err != nil {
		return 0, err
	}
	return length, nil
}

// Blob reads a length-prefixed byte slice.
func (r *BinaryReader) Blob() ([]byte, error) {
	length, err := r.Length()
	if err != nil {
		return nil, err
	}
	value := make([]byte, length)
	_, err = io.ReadFull(r.r, value)
	return value, err
}

// String reads a length-prefixed string.
func (r *BinaryReader) String() (string, error) {
	value, err := r.Blob()
	if err != nil {
		return "", err
	}
	return string(value), nil
}

// OpenCompressedFile opens an s2-compressed binary artifact file and returns a BinaryReader.
func OpenCompressedFile(filename string) (*os.File, *BinaryReader, error) {
	file, err := os.Open(filepath.Clean(filename))
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
	compressed := s2.NewReader(file)
	buffered := bufio.NewReaderSize(compressed, BufioSize)
	return file, NewBinaryReader(buffered), nil
}
