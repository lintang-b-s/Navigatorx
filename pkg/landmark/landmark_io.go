package landmark

import (
	"fmt"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (lm *Landmark[W]) WriteLandmark(filename string, n int) error {
	return util.WriteCompressedFile(filename, func(w *util.BinaryWriter) error {

		landmarks := *lm.landmarks.Load()
		lw := lm.lw.Load()
		vlw := lm.vlw.Load()

		if err := w.Uint8(met.NumericMarker[W]()); err != nil {
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
		if err := met.WriteRoutingNumbers(w, *lw); err != nil {
			return err
		}
		return met.WriteRoutingNumbers(w, *vlw)
	})
}

func ReadLandmark[W util.RoutingNumber](
	filename string,
) (*Landmark[W], error) {
	file, r, err := util.OpenCompressedFile(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()
	marker, err := r.Uint8()
	if err != nil {
		return nil, err
	}
	expectedMarker := met.NumericMarker[W]()
	if marker != expectedMarker {
		return nil, fmt.Errorf("landmark numeric representation %d does not match expected %d", marker, expectedMarker)
	}
	k, err := r.Length()
	if err != nil {
		return nil, err
	}
	n, err := r.Length()
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
	lw, err := met.ReadRoutingNumbers[W](r)
	if err != nil {
		return nil, err
	}
	vlw, err := met.ReadRoutingNumbers[W](r)
	if err != nil {
		return nil, err
	}
	lm := NewLandmark[W]()
	lm.landmarks.Store(&landmarks)
	lm.lw.Store(&lw)
	lm.vlw.Store(&vlw)
	lm.n = da.Index(n)
	lm.k = da.Index(k)
	return lm, nil
}
