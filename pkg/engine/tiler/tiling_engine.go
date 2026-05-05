// Package tiler berisi RoadNetworkGraph tiling service (see  https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e)
package tiler

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"strconv"

	"github.com/cockroachdb/errors"
	"github.com/klauspost/compress/s2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/mmcloughlin/geohash"
	"go.uber.org/zap"
)

// TilingEngine engine untuk get subset of RoadNetworkGraph yang berada didalam userGeohash cell. terinspirasi dari: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
type TilingEngine struct {
	graph  *da.Graph
	logger *zap.Logger
}

func NewTilingEngine(graph *da.Graph, logger *zap.Logger) *TilingEngine {
	return &TilingEngine{
		graph:  graph,
		logger: logger,
	}
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
	lineBuf := make([]byte, 0, 1024)
	eTileMap := make(map[uint64][]da.Index)
	te.graph.ForOutEdges(func(exitPoint, head, tail, entryId, entryPoint da.Index, percentage float64, eId da.Index) {
		eGeoHashInt := te.graph.GetEdgeGeohash(eId)
		eTileMap[eGeoHashInt] = append(eTileMap[eGeoHashInt], eId)
	})

	te.logger.Sugar().Infof("writing %v graph tiles to file... ", len(eTileMap))

	// reuse writers
	s2w := s2.NewWriter(io.Discard)
	bw := bufio.NewWriterSize(s2w, 64*1024)

	// write tiles ke file "<geohash_p_6_string>.tile"
	for geohashInt, eIds := range eTileMap {
		geohashStr := geohash.ConvertIntToString(geohashInt, uint(GeohashPrecision))
		neighbors := geohash.NeighborsIntWithPrecision(geohashInt, uint(GeohashBits))

		err := te.writeTileToFile(geohashStr, eIds, neighbors, eTileMap, s2w, bw, lineBuf)
		if err != nil {
			return fmt.Errorf("tilingEngine.PreprocessTiles: failed to writeTileToFile: %v", geohashStr)
		}
	}

	te.logger.Info("completed writing tiles to file")
	return nil
}

func (te *TilingEngine) writeTileToFile(currGeohash string, eIds []da.Index, neighbors []uint64, eTileMap map[uint64][]da.Index, s2w *s2.Writer, bw *bufio.Writer,
	lineBuf []byte) error {
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

	// eIds adalah id dari edges yang inside currGeohash
	for _, eId := range eIds {
		if err := te.writeEdge(bw, eId, lineBuf); err != nil {
			return errors.Wrapf(err, "tilingEngine.writeTileToFile: failed to writeEdge: %s, eId: %v", filePath, eId)
		}
	}

	// edges yang inside neighbor geohashes (8 neighbors dari currGeohash)
	for _, nGh := range neighbors {
		if neighborEIds, ok := eTileMap[nGh]; ok {
			for _, eId := range neighborEIds {
				if err := te.writeEdge(bw, eId, lineBuf); err != nil {
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

func (te *TilingEngine) writeEdge(w *bufio.Writer, eId da.Index, lineBuf []byte) error {
	lineBuf = lineBuf[:0]
	lineBuf = strconv.AppendInt(lineBuf, int64(eId), 10)
	lineBuf = append(lineBuf, ' ')
	tailVId := te.graph.GetTailOfOutedge(eId)
	lineBuf = strconv.AppendInt(lineBuf, int64(tailVId), 10)
	lineBuf = append(lineBuf, ' ')
	headVId := te.graph.GetHeadOfOutEdge(eId)
	lineBuf = strconv.AppendInt(lineBuf, int64(headVId), 10)
	lineBuf = append(lineBuf, ' ')
	lineBuf = strconv.AppendFloat(lineBuf, te.graph.GetOutEdgeLength(eId), 'f', -1, 64)
	lineBuf = append(lineBuf, ' ')

	// tail dan head coordinate
	tCoord := te.graph.GetVertexCoordinate(tailVId)
	tLat, tLon := tCoord.GetLat(), tCoord.GetLon()
	hCoord := te.graph.GetVertexCoordinate(headVId)
	hLat, hLon := hCoord.GetLat(), hCoord.GetLon()

	lineBuf = strconv.AppendFloat(lineBuf, tLat, 'f', -1, 64)
	lineBuf = append(lineBuf, ' ')
	lineBuf = strconv.AppendFloat(lineBuf, tLon, 'f', -1, 64)
	lineBuf = append(lineBuf, ' ')
	lineBuf = strconv.AppendFloat(lineBuf, hLat, 'f', -1, 64)
	lineBuf = append(lineBuf, ' ')
	lineBuf = strconv.AppendFloat(lineBuf, hLon, 'f', -1, 64)

	eGeom := te.graph.GetEdgeGeometry(eId)
	lineBuf = append(lineBuf, ' ')
	lineBuf = strconv.AppendInt(lineBuf, int64(len(eGeom)), 10)
	for _, coord := range eGeom {
		lineBuf = append(lineBuf, ' ')
		lineBuf = strconv.AppendFloat(lineBuf, coord.Lat, 'f', -1, 64)
		lineBuf = append(lineBuf, ' ')
		lineBuf = strconv.AppendFloat(lineBuf, coord.Lon, 'f', -1, 64)
	}
	lineBuf = append(lineBuf, '\n')
	_, err := w.Write(lineBuf)

	return err
}
