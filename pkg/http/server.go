package http

import (
	"context"

	http_router "github.com/lintang-b-s/Navigatorx/pkg/http/router"
	"github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	http_server "github.com/lintang-b-s/Navigatorx/pkg/http/server"
	"github.com/spf13/viper"
	"go.uber.org/zap"
	"golang.org/x/sync/errgroup"
)

type Server struct {
	Log *zap.Logger
}

func NewServer(log *zap.Logger) *Server {
	return &Server{Log: log}
}

func (s *Server) Use(
	ctx context.Context,
	log *zap.Logger,

	useRateLimit bool,
	routingService controllers.RoutingService,
	mapMatcherService controllers.MapMatcherService,

) (*Server, error) {
	viper.SetDefault("API_PORT", 6060)
	viper.SetDefault("WEBSOCKET_PORT", 6666)

	viper.SetDefault("API_TIMEOUT", "1000s")

	config := http_server.Config{
		Port:          viper.GetInt("API_PORT"),
		WebsocketPort: viper.GetInt("WEBSOCKET_PORT"),
		Timeout:       viper.GetDuration("API_TIMEOUT"),
		ProxyPort:     6767,
	}

	server := http_router.NewAPI(log)

	g := errgroup.Group{}

	g.Go(func() error {
		return server.Run(
			ctx, config, log,
			useRateLimit, routingService, mapMatcherService,
		)
	})

	return s, nil
}
