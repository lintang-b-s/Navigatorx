package usecases

import (
	"context"
	"testing"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/stretchr/testify/assert"
	"go.uber.org/zap"
)

func TestMapMatcherService_OnlineMapMatch(t *testing.T) {
	log := zap.NewNop()
	mockEngine := new(MockOnlineMapMatcherEngine)
	ms := NewMapMatcherService(log, mockEngine)

	ctx := context.Background()
	gpsTime := time.Date(2026, 4, 29, 10, 0, 0, 0, time.UTC)
	gps := da.NewGPSPoint(yogyakartaOriginLat, yogyakartaOriginLon, gpsTime, 0, 0, false)

	t.Run("Success", func(t *testing.T) {
		candidates := []*ma.Candidate{}
		mockEngine.On("OnlineMapMatch", gps, 1, candidates, 0.0, 0.0, 0.0).
			Return(da.NewMatchedGPSPoint(gps, 0, da.NewCoordinate(yogyakartaOriginLat, yogyakartaOriginLon), 0.0), []*ma.Candidate{}, 0.0, 0.0)

		res, cands, speedMean, speedStd, err := ms.OnlineMapMatch(ctx, gps, 1, candidates, 0.0, 0.0, 0.0)

		assert.NoError(t, err)
		assert.NotNil(t, res)
		assert.NotNil(t, cands)
		assert.Equal(t, 0.0, speedMean)
		assert.Equal(t, 0.0, speedStd)
	})

	t.Run("Timeout", func(t *testing.T) {
		timeoutCtx, cancel := context.WithCancel(ctx)
		cancel() // cancel immediately

		_, _, _, _, err := ms.OnlineMapMatch(timeoutCtx, gps, 1, []*ma.Candidate{}, 0.0, 0.0, 0.0)

		assert.Error(t, err)
		assert.Contains(t, err.Error(), "request timeout")
	})
}
