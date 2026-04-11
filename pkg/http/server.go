package http

import (
	"time"

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
	log *zap.Logger,

	useRateLimit bool,
	routingService controllers.RoutingService,
	mapMatcherService controllers.MapMatcherService,
	shutdownPeriod time.Duration,

) error {
	viper.SetDefault("http_port", 6060)
	viper.SetDefault("websocket_port", 6666)
	viper.SetDefault("proxy_port", 6767)

	viper.SetDefault("server.api_timeout", "2s")

	config := http_server.Config{
		Port:          viper.GetInt("http_port"),
		WebsocketPort: viper.GetInt("websocket_port"),
		Timeout:       viper.GetDuration("server.api_timeout"),
		ProxyPort:     viper.GetInt("proxy_port"),
	}

	server := http_router.NewAPI(log)

	g := errgroup.Group{}

	g.Go(func() error {
		return server.Run(
			config, log,
			useRateLimit, routingService, mapMatcherService,
			shutdownPeriod,
		)
	})

	return g.Wait()
}
