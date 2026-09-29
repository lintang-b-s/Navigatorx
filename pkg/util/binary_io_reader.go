package util

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"

	"github.com/bits-and-blooms/bitset"
	"github.com/klauspost/compress/s2"
)

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

func (r *BinaryReader) ReadBitset() (*bitset.BitSet, error) {
	bb := bitset.New(100)
	_, err := bb.ReadFrom(r.r)
	return bb, err
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

func (r *BinaryReader) ReadUint16s() ([]uint16, error) {
	length, err := r.Length()
	if err != nil {
		return make([]uint16, 0), err
	}
	values := make([]uint16, length)
	const itemSize = 4
	err = r.readBulk(len(values), itemSize, func(chunk []byte, offset int) {
		for i := 0; i < len(chunk); i += itemSize {
			values[offset+i/itemSize] = uint16(binary.LittleEndian.Uint16(chunk[i : i+itemSize]))
		}
	})

	if err != nil {
		return make([]uint16, 0), err
	}
	return values, nil
}

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

func (r *BinaryReader) Length() (uint32, error) {
	length, err := r.Uint32()
	if err != nil {
		return 0, err
	}
	return length, nil
}

func (r *BinaryReader) Blob() ([]byte, error) {
	length, err := r.Length()
	if err != nil {
		return nil, err
	}
	value := make([]byte, length)
	_, err = io.ReadFull(r.r, value)
	return value, err
}

func (r *BinaryReader) String() (string, error) {
	value, err := r.Blob()
	if err != nil {
		return "", err
	}
	return string(value), nil
}

func OpenCompressedFile(filename string) (*os.File, *BinaryReader, error) {
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
