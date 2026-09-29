package datastructure

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	segmentKeyLen = 2
	turnKeyLen    = 3
)

func (kv KeyVal) write(w *util.BinaryWriter) error {
	if err := w.WriteUInt64s(kv.keys); err != nil {
		return err
	}
	return w.Uint32(uint32(kv.val))
}

func readKeyVal(r *util.BinaryReader, wantKeys int) (KeyVal, error) {
	keys, err := r.ReadUint64s()
	if err != nil {
		return KeyVal{}, err
	}
	if len(keys) != wantKeys {
		return KeyVal{}, fmt.Errorf("got %d keys, want %d", len(keys), wantKeys)
	}
	v, err := r.Uint32()
	if err != nil {
		return KeyVal{}, err
	}
	return KeyVal{keys: keys, val: Index(v)}, nil
}

func (s *SegmentKV) WriteToFile(w *util.BinaryWriter) error {
	return s.kv.write(w)
}

func (t *TurnKV) WriteToFile(w *util.BinaryWriter) error {
	return t.kv.write(w)
}

func readSegmentKVFromFile(r *util.BinaryReader) (*SegmentKV, error) {
	kv, err := readKeyVal(r, segmentKeyLen)
	if err != nil {
		return nil, fmt.Errorf("read segment kv: %w", err)
	}
	return &SegmentKV{kv: kv}, nil
}

func readTurnKVFromFile(r *util.BinaryReader) (*TurnKV, error) {
	kv, err := readKeyVal(r, turnKeyLen)
	if err != nil {
		return nil, fmt.Errorf("read turn kv: %w", err)
	}
	return &TurnKV{kv: kv}, nil
}

func (lt *LookupTable[T]) WriteToFile(filename string) error {
	return util.WriteCompressedFile(filename, func(w *util.BinaryWriter) error {
		if err := w.Uint32(uint32(len(lt.data))); err != nil {
			return err
		}
		for _, d := range lt.data {
			if err := d.WriteToFile(w); err != nil {
				return err
			}
		}
		return nil
	})
}

func readLookupTableFromFile[T LookupKV[T]](
	filename string,
	readEntry func(*util.BinaryReader) (T, error),
) (*LookupTable[T], error) {
	f, r, err := util.OpenCompressedFile(filename)
	if err != nil {
		return nil, fmt.Errorf("open lookup table %q: %w", filename, err)
	}
	defer f.Close()

	length, err := r.Uint32()
	if err != nil {
		return nil, fmt.Errorf("read lookup table length: %w", err)
	}

	data := make([]T, 0, length)
	for i := uint32(0); i < length; i++ {
		e, err := readEntry(r)
		if err != nil {
			return nil, fmt.Errorf("read lookup table entry %d: %w", i, err)
		}
		data = append(data, e)
	}

	return &LookupTable[T]{data: data}, nil
}

func ReadSegmentTable(filename string) (*LookupTable[*SegmentKV], error) {
	return readLookupTableFromFile(filename, readSegmentKVFromFile)
}

func ReadTurnTable(filename string) (*LookupTable[*TurnKV], error) {
	return readLookupTableFromFile(filename, readTurnKVFromFile)
}
