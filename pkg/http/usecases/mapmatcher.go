package usecases

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"go.uber.org/zap"
)

type MapMatcherService struct {
	log           *zap.Logger
	onlineEngine  OnlineMapMatcherEngine
	offlineEngine OfflineMapMatcherEngine
}

func NewMapMatcherService(log *zap.Logger, onlineEngine OnlineMapMatcherEngine,
	offlineEngine OfflineMapMatcherEngine) *MapMatcherService {
	return &MapMatcherService{
		log:           log,
		onlineEngine:  onlineEngine,
		offlineEngine: offlineEngine,
	}
}

func (ms *MapMatcherService) OnlineMapMatch(gps *da.GPSPoint, k int,
	candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64) {
	return ms.onlineEngine.OnlineMapMatch(gps, k, candidates, speedMeanK, speedStdK, lastBearing)
}

func (ms *MapMatcherService) OfflineMapMatch(gpsTraj []*da.GPSPoint) ([]*da.MatchedGPSPoint, string) {
	matchedPoints := ms.offlineEngine.MapMatch(gpsTraj)

	matchedCoords := make([]da.Coordinate, len(matchedPoints))
	for i := 0; i < len(matchedPoints); i++ {
		matchedCoords[i] = matchedPoints[i].GetMatchedCoord()
	}

	polyline := da.PoylineFromCoords(matchedCoords)
	return matchedPoints, polyline
}
