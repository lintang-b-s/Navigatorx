package usecases

import (
	"context"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"go.uber.org/zap"
)

type MapMatcherService struct {
	log          *zap.Logger
	onlineEngine OnlineMapMatcherEngine
}

func NewMapMatcherService(log *zap.Logger, onlineEngine OnlineMapMatcherEngine,
) *MapMatcherService {
	return &MapMatcherService{
		log:          log,
		onlineEngine: onlineEngine,
	}
}

func (ms *MapMatcherService) OnlineMapMatch(ctx context.Context, gps *da.GPSPoint, k int,
	candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64, error) {

	matchedPoint, cands, speedMean, speedStd := ms.onlineEngine.OnlineMapMatch(gps, k, candidates, speedMeanK, speedStdK, lastBearing)
	return matchedPoint, cands, speedMean, speedStd, nil
}
