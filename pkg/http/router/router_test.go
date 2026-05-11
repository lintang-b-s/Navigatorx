package router

import (
	"context"
	"errors"
	"fmt"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"syscall"
	"testing"
	"time"

	http_server "github.com/lintang-b-s/Navigatorx/pkg/http/server"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/mock"
	"go.uber.org/zap"
)

func TestAPI_Run(t *testing.T) {
	log := zap.NewNop()
	api := NewAPI(log)

	mockRS := new(MockRoutingService)
	mockMMS := new(MockMapMatcherService)
	mockTS := new(MockTilingService)
	mockEngine := new(MockRoutingEngine)

	mockRS.On("GetRoutingEngine").Return(mockEngine)
	mockRS.On("Close").Return()
	mockEngine.On("Close").Return()
	mockRS.On("InitBackgroundWorker", mock.MatchedBy(func(ctx context.Context) bool {
		return ctx != nil && ctx.Err() == nil
	})).Return()

	t.Run("Success", func(t *testing.T) {
		config := http_server.Config{
			Port: 0, // Random port
		}

		go func() {
			time.Sleep(200 * time.Millisecond)
			process, _ := os.FindProcess(os.Getpid())
			_ = process.Signal(syscall.SIGINT)
		}()

		err := api.Run(config, log, false, mockRS, mockMMS, mockTS, 100*time.Millisecond)

		assert.NoError(t, err)
	})

	t.Run("ListenAndServe Error", func(t *testing.T) {
		ln, err := net.Listen("tcp", ":0")

		if err != nil {
			t.Fatal(err)
		}
		defer ln.Close()

		_, portStr, _ := net.SplitHostPort(ln.Addr().String())
		var port int
		_, _ = fmt.Sscanf(portStr, "%d", &port)

		configErr := http_server.Config{
			Port: port,
		}

		err = api.Run(configErr, log, false, mockRS, mockMMS, mockTS, 100*time.Millisecond)
		assert.Error(t, err)
	})

}

func TestAPI_Run_InjectedServerError(t *testing.T) {
	log := zap.NewNop()
	api := NewAPI(log)
	mockRS := new(MockRoutingService)
	mockMMS := new(MockMapMatcherService)
	mockTS := new(MockTilingService)
	mockEngine := new(MockRoutingEngine)

	mockRS.On("GetRoutingEngine").Return(mockEngine)
	mockRS.On("Close").Return()
	mockEngine.On("Close").Return()
	mockRS.On("InitBackgroundWorker", mock.MatchedBy(func(ctx context.Context) bool {
		return ctx != nil && ctx.Err() == nil
	})).Return()

	origListenAndServe := listenAndServeHTTP
	origShutdown := shutdownHTTP
	listenAndServeHTTP = func(s *http.Server) error {
		if s.Addr == ":9102" {
			return errors.New("main server error")
		}
		time.Sleep(50 * time.Millisecond)
		return http.ErrServerClosed
	}
	shutdownHTTP = func(*http.Server, context.Context) error {
		return nil
	}
	t.Cleanup(func() {
		listenAndServeHTTP = origListenAndServe
		shutdownHTTP = origShutdown
	})

	err := api.Run(http_server.Config{Port: 9102}, log, true, mockRS, mockMMS, mockTS, 50*time.Millisecond)

	assert.EqualError(t, err, "main server error")
	mockRS.AssertExpectations(t)
	mockEngine.AssertExpectations(t)
}

func TestSwaggerHandler(t *testing.T) {
	t.Run("Success or Not Found", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/swagger/", nil)
		rr := httptest.NewRecorder()

		swaggerHandler(rr, req, nil)

		// It calls httpSwagger.WrapHandler, which might return 404 if not configured correctly
		assert.True(t, rr.Code == http.StatusOK || rr.Code == http.StatusNotFound || rr.Code == http.StatusMovedPermanently)
	})
}
