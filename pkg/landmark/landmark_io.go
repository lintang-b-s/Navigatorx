package landmark

import (
	"bufio"
	"fmt"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	maxLandmarkItems = uint32(math.MaxInt32)
	landmarkMagic    = uint32(0x324d4c4e)
)

func (lm *Landmark[W]) WriteLandmark(filename string, n int) error {
	return util.WriteCompressedArtifact(filename, func(w *util.BinaryWriter) error {
		landmarks := *lm.landmarks.Load()
		lw := lm.lw.Load()
		vlw := lm.vlw.Load()
		if lw.rows != da.Index(len(landmarks)) || lw.cols != da.Index(n) ||
			vlw.rows != da.Index(n) || vlw.cols != da.Index(len(landmarks)) {
			return fmt.Errorf("landmark dimensions do not match k=%d n=%d", len(landmarks), n)
		}
		if err := w.Uint32(landmarkMagic); err != nil {
			return err
		}
		if err := w.Uint8(costfunction.NumericMarker[W]()); err != nil {
			return err
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
		if err := costfunction.WriteRoutingNumbers(w, lw.values); err != nil {
			return err
		}
		return costfunction.WriteRoutingNumbers(w, vlw.values)
	})
}

func ReadLandmark[W util.RoutingNumber](
	filename string,
	_ *bufio.Reader,
) (*Landmark[W], error) {
	file, r, err := util.OpenCompressedArtifact(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	magic, err := r.Uint32()
	if err != nil || magic != landmarkMagic {
		return nil, fmt.Errorf("unsupported legacy landmark artifact; regenerate it with cmd/customizer")
	}
	marker, err := r.Uint8()
	if err != nil {
		return nil, err
	}
	expectedMarker := costfunction.NumericMarker[W]()
	if marker != expectedMarker {
		return nil, fmt.Errorf("landmark numeric representation %d does not match expected %d", marker, expectedMarker)
	}
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
	lw := newLandmarkTable[W](da.Index(k), da.Index(n))
	lwCount, err := r.Length(maxLandmarkItems)
	if err != nil || lwCount != uint32(len(lw.values)) {
		return nil, fmt.Errorf("invalid landmark-to-vertex value count %d", lwCount)
	}
	if err := costfunction.ReadRoutingNumbers(r, lw.values); err != nil {
		return nil, err
	}
	vlw := newLandmarkTable[W](da.Index(n), da.Index(k))
	vlwCount, err := r.Length(maxLandmarkItems)
	if err != nil || vlwCount != uint32(len(vlw.values)) {
		return nil, fmt.Errorf("invalid vertex-to-landmark value count %d", vlwCount)
	}
	if err := costfunction.ReadRoutingNumbers(r, vlw.values); err != nil {
		return nil, err
	}
	lm := NewLandmark[W]()
	lm.landmarks.Store(&landmarks)
	lm.lw.Store(lw)
	lm.vlw.Store(vlw)
	return lm, nil
}
