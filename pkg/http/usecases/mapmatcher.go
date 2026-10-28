package usecases

import (
	"context"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"go.uber.org/zap"
)

// MapMatcherService represents map matcher service.
type MapMatcherService struct {
	log          *zap.Logger
	onlineEngine OnlineMapMatcherEngine
}

// NewMapMatcherService creates and returns map matcher service.
func NewMapMatcherService(log *zap.Logger, onlineEngine OnlineMapMatcherEngine,
) *MapMatcherService {
	return &MapMatcherService{
		log:          log,
		onlineEngine: onlineEngine,
	}
}

// OnlineMapMatch handles online map matching.
func (ms *MapMatcherService) OnlineMapMatch(_ context.Context, gps *da.GPSPoint, k int,
	candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64, error) {

	matchedPoint, cands, speedMean, speedStd := ms.onlineEngine.OnlineMapMatch(gps, k, candidates, speedMeanK, speedStdK, lastBearing)
	return matchedPoint, cands, speedMean, speedStd, nil
}
