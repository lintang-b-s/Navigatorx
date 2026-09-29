// Package tiler berisi RoadNetworkGraph tiling service (see  https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e)
package tiler

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"path/filepath"

	"github.com/cockroachdb/errors"
	"github.com/klauspost/compress/s2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/mmcloughlin/geohash"
	"go.uber.org/zap"
)

// TilingEngine engine untuk get subset of RoadNetworkGraph yang berada didalam userGeohash cell. terinspirasi dari: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
type TilingEngine[W util.RoutingNumber] struct {
	graph        *da.Graph
	rn           *da.RoadNetworkDataContainer
	timeFunction *met.TimeFunction[W]
	logger       *zap.Logger
}

func NewTilingEngine[W util.RoutingNumber](graph *da.Graph, rn *da.RoadNetworkDataContainer, logger *zap.Logger, timeFunction *met.TimeFunction[W]) *TilingEngine[W] {
	engine := &TilingEngine[W]{
		graph:        graph,
		rn:           rn,
		logger:       logger,
		timeFunction: timeFunction,
	}

	return engine
}

// GetTileFilePath get tile file path based on user geohash (6 precision)
func (te *TilingEngine[W]) GetTileFilePath(userGeohash string) string {
	filePath := filepath.Join(MapTileFilePathPrefix(), userGeohash+".tile")
	return filePath
}

func (te *TilingEngine[W]) GetNumberOfVertices() int {
	return te.graph.NumberOfVertices()
}

func (te *TilingEngine[W]) PreprocessTiles() error {
	eTileMap := make(map[uint64][]da.Index)

	te.graph.ForVertices(func(_ da.Vertex, segId da.Index) {
		eGeoHashInt := te.rn.GetSegmentGeohash(segId)
		eTileMap[eGeoHashInt] = append(eTileMap[eGeoHashInt], segId)
	})

	te.logger.Sugar().Infof("writing %v graph tiles to files... ", len(eTileMap))

	// reuse writers
	s2w := s2.NewWriter(io.Discard)
	bw := bufio.NewWriterSize(s2w, 64*1024)

	tileGeohashes := make(map[uint64]struct{}, len(eTileMap))
	for geohashInt := range eTileMap {
		tileGeohashes[geohashInt] = struct{}{}
		for _, neighbor := range geohash.NeighborsIntWithPrecision(geohashInt, uint(GeohashBits)) {
			if eids, ok := eTileMap[neighbor]; !ok || len(eids) == 0 {
				tileGeohashes[neighbor] = struct{}{}
			}
		}
	}

	// write tiles ke file "<geohash_p_6_string>.tile"
	for geohashInt := range tileGeohashes {
		geohashStr := geohash.ConvertIntToString(geohashInt, uint(GeohashPrecision))
		neighbors := geohash.NeighborsIntWithPrecision(geohashInt, uint(GeohashBits))

		err := te.writeTileToFile(geohashStr, eTileMap[geohashInt], neighbors, eTileMap, s2w, bw)
		if err != nil {
			return fmt.Errorf("tilingEngine.PreprocessTiles: failed to writeTileToFile: %v", geohashStr)
		}
	}

	te.logger.Info("completed writing tiles to files")
	return nil
}

func (te *TilingEngine[W]) writeTileToFile(currGeohash string, segIds []da.Index, neighbors []uint64, eTileMap map[uint64][]da.Index, s2w *s2.Writer, bw *bufio.Writer) error {
	filePath := filepath.Join(MapTileFilePathPrefix(), currGeohash+".tile")
	dir := filepath.Dir(filePath)
	if _, err := os.Stat(dir); os.IsNotExist(err) {
		if err := os.MkdirAll(dir, 0755); err != nil {
			return err
		}
	}

	f, err := os.Create(filePath)
	if err != nil {
		return errors.Wrapf(err, "tilingEngine.writeTileToFile: failed to create file: %s", filePath)
	}
	defer f.Close()

	// reset writer
	s2w.Reset(f)
	bw.Reset(s2w)
	binaryWriter := util.NewBinaryWriter(bw)

	// eIds adalah id dari road segments yang inside currGeohash
	for _, segId := range segIds {
		if err := te.writeSegment(binaryWriter, segId); err != nil {
			return errors.Wrapf(err, "tilingEngine.writeTileToFile: failed to writeSegment: %s, segId: %v", filePath, segId)
		}
	}

	// road segments yang inside neighbor geohashes (8 neighbors dari currGeohash)
	for _, nGh := range neighbors {
		if neighborEIds, ok := eTileMap[nGh]; ok {
			for _, segId := range neighborEIds {
				if err := te.writeSegment(binaryWriter, segId); err != nil {
					return errors.Wrapf(err, "tilingEngine.writeTileToFile: failed to writeSegment (neighbor): %s, segId: %v", filePath, segId)
				}
			}
		}
	}

	if err := bw.Flush(); err != nil {
		return err
	}
	return s2w.Close()
}

func (te *TilingEngine[W]) writeSegment(w *util.BinaryWriter, segId da.Index) error {
	if err := w.Uint32(uint32(segId)); err != nil {
		return err
	}
	l := util.DistanceToMeters(te.timeFunction.GetSegmentLength(segId))
	if err := w.Float64(l); err != nil {
		return err
	}
	eGeom := te.rn.GetSegmentGeometry(segId)
	if err := w.Length(len(eGeom)); err != nil {
		return err
	}
	for _, coord := range eGeom {
		if err := w.Int32(coord.GetFixedLat()); err != nil {
			return err
		}
		if err := w.Int32(coord.GetFixedLon()); err != nil {
			return err
		}
	}
	return nil
}
