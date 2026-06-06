package landmark

import (
	"bufio"
	"fmt"
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const maxLandmarkItems = uint32(math.MaxInt32)

func (lm *Landmark) WriteLandmark(filename string, n int) error {
	return util.WriteCompressedArtifact(filename, func(w *util.BinaryWriter) error {
		landmarks := lm.landmarks.Load().([]da.Index)
		lw := lm.lw.Load().(*landmarkTable)
		vlw := lm.vlw.Load().(*landmarkTable)
		if lw.rows != da.Index(len(landmarks)) || lw.cols != da.Index(n) || vlw.rows != da.Index(n) || vlw.cols != da.Index(len(landmarks)) {
			return fmt.Errorf("landmark dimensions do not match k=%d n=%d", len(landmarks), n)
		}
		if err := w.Length(len(landmarks)); err != nil {
			return err
		}
		if err := w.Length(n); err != nil {
			return err
		}
		for _, id := range landmarks {
			if err := w.Uint32(uint32(id)); err != nil {
				return err
			}
		}
		for _, value := range lw.values {
			if err := w.Float64(value); err != nil {
				return err
			}
		}
		for _, value := range vlw.values {
			if err := w.Float64(value); err != nil {
				return err
			}
		}
		return nil
	})
}

func ReadLandmark(filename string, _ *bufio.Reader) (*Landmark, error) {
	file, r, err := util.OpenCompressedArtifact(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	k, err := r.Length(maxLandmarkItems)
	if err != nil {
		return nil, err
	}
	n, err := r.Length(maxLandmarkItems)
	if err != nil {
		return nil, err
	}

	landmarks := make([]da.Index, k)
	for i := range landmarks {
		value, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		landmarks[i] = da.Index(value)
	}
	lw := newLandmarkTable(da.Index(k), da.Index(n))
	if err := r.ReadFloat64s(lw.values); err != nil {
		return nil, err
	}
	vlw := newLandmarkTable(da.Index(n), da.Index(k))
	if err := r.ReadFloat64s(vlw.values); err != nil {
		return nil, err
	}
	lm := NewLandmark()
	lm.landmarks.Store(landmarks)
	lm.lw.Store(lw)
	lm.vlw.Store(vlw)
	return lm, nil
}
