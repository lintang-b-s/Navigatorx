package usecases

import (
	"context"

	"github.com/golang/geo/s2"
	"go.uber.org/zap"
)

// MapAttributesService represents map attributes service.
type MapAttributesService struct {
	log                 *zap.Logger
	mapAttributesEngine MapAttributesEngine
}

// NewMapAttributesService creates and returns map attributes service.
func NewMapAttributesService(log *zap.Logger, mapAttributesEngine MapAttributesEngine,
) *MapAttributesService {
	return &MapAttributesService{
		log:                 log,
		mapAttributesEngine: mapAttributesEngine,
	}
}

// GetMapAttributes returns map attributes.
func (ms *MapAttributesService) GetMapAttributes(_ context.Context, s2CellId s2.CellID, radius float64) ([]byte, error) {
	mvt, err := ms.mapAttributesEngine.GetMapAttributes(s2CellId, radius)
	return mvt, err
}
