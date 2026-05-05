package usecases

import (
	"context"

	"go.uber.org/zap"
)

type TileService struct {
	log          *zap.Logger
	tilingEngine TilingEngine
}

func NewTileService(log *zap.Logger, tilingEngine TilingEngine,
) *TileService {
	return &TileService{
		log:          log,
		tilingEngine: tilingEngine,
	}
}

func (ms *TileService) GetTileFilePath(ctx context.Context, userGeohash string) string {

	tileFilePath := ms.tilingEngine.GetTileFilePath(userGeohash)
	return tileFilePath
}

func (ms *TileService) GetNumberOfVertices(ctx context.Context) int {
	return ms.tilingEngine.GetNumberOfVertices()
}
