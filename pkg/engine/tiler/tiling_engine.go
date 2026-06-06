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
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/mmcloughlin/geohash"
	"go.uber.org/zap"
)

// TilingEngine engine untuk get subset of RoadNetworkGraph yang berada didalam userGeohash cell. terinspirasi dari: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
type TilingEngine struct {
	graph        *da.Graph
	timeFunction *costfunction.TimeFunction
	logger       *zap.Logger
}

func NewTilingEngine(graph *da.Graph, logger *zap.Logger, timeFunction *costfunction.TimeFunction) *TilingEngine {
	engine := &TilingEngine{
		graph:        graph,
		logger:       logger,
		timeFunction: timeFunction,
	}

	return engine
}

// GetTileFilePath get tile file path based on user geohash (6 precision)
func (te *TilingEngine) GetTileFilePath(userGeohash string) string {
	filePath := filepath.Join(MapTileFilePathPrefix(), userGeohash+".tile")
	return filePath
}

func (te *TilingEngine) GetNumberOfVertices() int {
	return te.graph.NumberOfVertices()
}

func (te *TilingEngine) PreprocessTiles() error {
	eTileMap := make(map[uint64][]da.Index)
	te.graph.ForOutEdges(func(exitPoint, head, tail, entryId, entryPoint da.Index, percentage float64, eId da.Index) {
		eGeoHashInt := te.graph.GetEdgeGeohash(eId)
		eTileMap[eGeoHashInt] = append(eTileMap[eGeoHashInt], eId)
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

func (te *TilingEngine) writeTileToFile(currGeohash string, eIds []da.Index, neighbors []uint64, eTileMap map[uint64][]da.Index, s2w *s2.Writer, bw *bufio.Writer) error {
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

	// eIds adalah id dari edges yang inside currGeohash
	for _, eId := range eIds {
		if err := te.writeEdge(binaryWriter, eId); err != nil {
			return errors.Wrapf(err, "tilingEngine.writeTileToFile: failed to writeEdge: %s, eId: %v", filePath, eId)
		}
	}

	// edges yang inside neighbor geohashes (8 neighbors dari currGeohash)
	for _, nGh := range neighbors {
		if neighborEIds, ok := eTileMap[nGh]; ok {
			for _, eId := range neighborEIds {
				if err := te.writeEdge(binaryWriter, eId); err != nil {
					return errors.Wrapf(err, "tilingEngine.writeTileToFile: failed to writeEdge (neighbor): %s, eId: %v", filePath, eId)
				}
			}
		}
	}

	if err := bw.Flush(); err != nil {
		return err
	}
	return s2w.Close()
}

func (te *TilingEngine) writeEdge(w *util.BinaryWriter, eId da.Index) error {
	if err := w.Uint32(uint32(eId)); err != nil {
		return err
	}
	tailVId := te.graph.GetTailOfOutedge(eId)
	if err := w.Uint32(uint32(tailVId)); err != nil {
		return err
	}
	headVId := te.graph.GetHeadOfOutEdge(eId)
	if err := w.Uint32(uint32(headVId)); err != nil {
		return err
	}
	if err := w.Float64(te.timeFunction.GetSegmentLength(eId)); err != nil {
		return err
	}

	eGeom := te.graph.GetEdgeGeometry(eId)
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
