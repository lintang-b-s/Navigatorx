package usecases

import (
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"go.uber.org/zap"
)

type MapMatcherService struct {
	log    *zap.Logger
	engine MapMatcherEngine
}

func NewMapMatcherService(log *zap.Logger, engine MapMatcherEngine) *MapMatcherService {
	return &MapMatcherService{
		log:    log,
		engine: engine,
	}
}

func (ms *MapMatcherService) OnlineMapMatch(gps *datastructure.GPSPoint, k int,
	candidates []*online.Candidate, speedMeanK, speedStdK, lastBearing float64) (*datastructure.MatchedGPSPoint, []*online.Candidate, float64, float64) {
	return ms.engine.OnlineMapMatch(gps, k, candidates, speedMeanK, speedStdK, lastBearing)
}
