package http

import (
	"context"
	"os"
	"strings"
	"syscall"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/mock"
	"go.uber.org/zap"
)

func skipHTTPServerSocketPermission(t *testing.T, err error) {
	t.Helper()
	if err != nil && strings.Contains(err.Error(), "operation not permitted") {
		t.Skipf("local sockets are not permitted in this sandbox: %v", err)
	}
}

func TestNewServer(t *testing.T) {
	log := zap.NewNop()
	srv := NewServer(log)
	assert.NotNil(t, srv)
	assert.Equal(t, log, srv.Log)
}

func TestServer_Use(t *testing.T) {
	log := zap.NewNop()
	srv := NewServer(log)

	mockRE := &mockRoutingEngine{}
	mockRS := &mockRoutingService{}
	mockRS.On("GetRoutingEngine").Return(mockRE)
	mockRS.On("InitBackgroundWorker", mock.MatchedBy(func(ctx context.Context) bool {
		return ctx != nil && ctx.Err() == nil
	})).Return()
	mockRS.On("Close").Return()
	mockRE.On("Close").Return()

	mockMMS := &mockMapMatcherService{}

	go func() {
		time.Sleep(1000 * time.Millisecond)
		p, _ := os.FindProcess(os.Getpid())
		_ = p.Signal(syscall.SIGINT)
	}()

	err := srv.Use(log, false, mockRS, mockMMS, 100*time.Millisecond)
	skipHTTPServerSocketPermission(t, err)
	assert.NoError(t, err)
}

type mockRoutingEngine struct {
	controllers.RoutingEngine
	mock.Mock
}

func (m *mockRoutingEngine) Close() { m.Called() }

type mockRoutingService struct {
	controllers.RoutingService
	mock.Mock
}

func (m *mockRoutingService) Close() { m.Called() }
func (m *mockRoutingService) GetRoutingEngine() controllers.RoutingEngine {
	args := m.Called()
	return args.Get(0).(controllers.RoutingEngine)
}
func (m *mockRoutingService) InitBackgroundWorker(ctx context.Context) { m.Called(ctx) }

type mockMapMatcherService struct {
	controllers.MapMatcherService
	mock.Mock
}
