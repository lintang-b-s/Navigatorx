package usecases

import (
	"context"

	"go.uber.org/zap"
)

type MapAttributesService struct {
	log                 *zap.Logger
	mapAttributesEngine MapAttributesEngine
}

func NewMapAttributesService(log *zap.Logger, mapAttributesEngine MapAttributesEngine,
) *MapAttributesService {
	return &MapAttributesService{
		log:                 log,
		mapAttributesEngine: mapAttributesEngine,
	}
}

func (ms *MapAttributesService) GetMapAttributes(ctx context.Context, h3CellId string) ([]byte, error) {
	mvt, err := ms.mapAttributesEngine.GetMapAttributes(h3CellId)
	return mvt, err
}
