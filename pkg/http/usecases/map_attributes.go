package usecases

import (
	"context"

	"github.com/golang/geo/s2"
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

func (ms *MapAttributesService) GetMapAttributes(ctx context.Context, s2CellId s2.CellID) ([]byte, error) {
	mvt, err := ms.mapAttributesEngine.GetMapAttributes(s2CellId)
	return mvt, err
}
