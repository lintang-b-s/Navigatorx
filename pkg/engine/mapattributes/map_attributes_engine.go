// Package mapattributes berisi MapAttributes Engine (see  https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e)
package mapattributes

import (
	"bytes"
	"fmt"
	"sort"

	s2geo "github.com/golang/geo/s2"
	"github.com/klauspost/compress/s2"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

// MapAttributesEngine engine untuk get subset of RoadNetworkGraph yang berada didalam quadKey web mercator tiles. terinspirasi dari MapAttributes service: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
type MapAttributesEngine[W util.RoutingNumber] struct {
	g      *da.Graph
	rn     *da.RoadNetworkDataContainer
	met    *met.Metric[W]
	idx    *spatialindex.S2RoadSegmentsIndex
	logger *zap.Logger
}

func NewMapAttributesEngine[W util.RoutingNumber](graph *da.Graph, rn *da.RoadNetworkDataContainer, logger *zap.Logger, met *met.Metric[W], idx *spatialindex.S2RoadSegmentsIndex) *MapAttributesEngine[W] {

	engine := &MapAttributesEngine[W]{
		g:      graph,
		rn:     rn,
		logger: logger,
		met:    met,
		idx:    idx,
	}

	return engine
}

// GetMapAttributes get MapAttributes by s2 CellId and radius.
func (me *MapAttributesEngine[W]) GetMapAttributes(s2CellId s2geo.CellID, radius float64) ([]byte, error) {
	// query road segments from s2 index
	segmentIds := me.idx.GetCellSegments(s2CellId, radius)

	buf := &bytes.Buffer{}
	sn := s2.NewWriter(buf)
	bw := util.NewBinaryWriter(sn)
	err := bw.Length(len(segmentIds))
	if err != nil {
		return make([]byte, 0), fmt.Errorf("failed to write uint32: %w", err)
	}

	segSet := make(map[da.Index]struct{})
	for _, segId := range segmentIds {
		segSet[segId] = struct{}{}
	}

	sort.Slice(segmentIds, func(i, j int) bool {
		return segmentIds[i] < segmentIds[j]
	})

	n := len(segmentIds)
	outDegs := make([]da.Index, n)
	nt := da.Index(0)
	for i, segId := range segmentIds {
		speed := me.met.GetSegmentSpeed(segId)
		length := me.met.GetSegmentLength(segId)
		geom := me.rn.GetSegmentGeometry(segId)
		polyline := da.GooglePoylineFromCoords(geom)
		err = bw.Uint32(uint32(segId))
		if err != nil {
			return make([]byte, 0), fmt.Errorf("MapAttributesEngine.GetMapAttributes: failed to write uint32: %w", err)
		}
		err = bw.Float64(speed)
		if err != nil {
			return make([]byte, 0), fmt.Errorf("MapAttributesEngine.GetMapAttributes: failed to write float64: %w", err)
		}
		err = bw.Float64(length)
		if err != nil {
			return make([]byte, 0), fmt.Errorf("MapAttributesEngine.GetMapAttributes: failed to write float64: %w", err)
		}
		err = bw.String(polyline)
		if err != nil {
			return make([]byte, 0), fmt.Errorf("MapAttributesEngine.GetMapAttributes: failed to write string: %w", err)
		}

		outDeg := da.Index(0)
		u := segId
		me.g.ForOutEdgesOf(u, func(eId, v, _ da.Index) {
			_, ok := segSet[v]
			if ok {
				outDeg++
			}
		})

		outDegs[i] = outDeg
		nt += outDeg
	}

	err = bw.Uint32(uint32(nt))
	if err != nil {
		return make([]byte, 0), fmt.Errorf("MapAttributesEngine.GetMapAttributes: failed to write uint32: %w", err)
	}
	for i, u := range segmentIds {
		if outDegs[i] == 0 {
			continue
		}
		var err error
		me.g.ForOutEdgesOf(u, func(eId, v, _ da.Index) {
			_, ok := segSet[v]
			if !ok {
				return
			}
			if err != nil {
				return
			}
			weight := me.met.GetWeight(eId)
			if err = bw.Uint32(uint32(weight)); err != nil {
				return
			}
			if err = bw.Uint32(uint32(u)); err != nil {
				return
			}
			err = bw.Uint32(uint32(v))
		})
		if err != nil {
			return make([]byte, 0), fmt.Errorf("MapAttributesEngine.GetMapAttributes: failed to write uint32: %w", err)
		}
	}

	err = sn.Close()
	if err != nil {
		return make([]byte, 0), fmt.Errorf("MapAttributesEngine.GetMapAttributes: failed to close snappy: %w", err)
	}

	return buf.Bytes(), nil
}
