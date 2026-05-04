package router

import (
	"context"
	"fmt"
	"net"
	"os"
	"os/signal"
	"strconv"
	"syscall"
	"time"

	"github.com/NYTimes/gziphandler"
	"github.com/julienschmidt/httprouter"
	"github.com/justinas/alice"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	router_helper "github.com/lintang-b-s/Navigatorx/pkg/http/router/routerhelper"
	http_server "github.com/lintang-b-s/Navigatorx/pkg/http/server"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/mailru/easygo/netpoll"
	"github.com/rs/cors"
	"github.com/spf13/viper"
	_ "github.com/swaggo/http-swagger"
	httpSwagger "github.com/swaggo/http-swagger"
	"go.uber.org/zap"

	"net/http"
	_ "net/http/pprof"
)

type API struct {
	log    *zap.Logger
	hub    *controllers.Hub
	poller netpoll.Poller
	pool   *concurrent.WorkerPool[int, int]
}

var (
	listenAndServeHTTP = func(s *http.Server) error { return s.ListenAndServe() }
	shutdownHTTP       = func(s *http.Server, ctx context.Context) error { return s.Shutdown(ctx) }
)

func NewAPI(log *zap.Logger) *API {
	return &API{log: log}
}

//	@title			Navigatorx API
//	@version		1.0
//	@description	This is a routing engine for openstreetmap .

//	@contact.name	Lintang Birda Saputra
//	@contact.url	_
//	@contact.email	lintangbirdasaputra23@gmail.com

//	@license.name	BSD License
//	@license.url	https://opensource.org/license/bsd-2-clause

// @host		localhost
// @BasePath	/api
func (api *API) Run(
	config http_server.Config,
	log *zap.Logger,

	useRateLimit bool,
	routingService controllers.RoutingService,
	mapMatcherService controllers.MapMatcherService,
	tilingService controllers.TilingService,
	shutdownPeriod time.Duration,
) error {
	isShuttingDown.Store(false)
	ctx, cleanup := NewContext(routingService.GetRoutingEngine(), routingService)
	defer func() {
		cleanup()
		ctxWithTimeout, cancel := context.WithTimeout(context.Background(), 2*time.Second)
		_ = util.Sleep(ctxWithTimeout, readinessDrainDelay)
		cancel()
	}()

	viper.SetDefault("server.read_timeout", "2s")
	viper.SetDefault("server.write_timeout", "2s")
	viper.SetDefault("server.idle_timeout", "2s")
	viper.SetDefault("server.read_header_timeout", "2s")

	log.Info("Run httprouter API")

	routingService.InitBackgroundWorker(ctx)
	router := httprouter.New()

	corsHandler := cors.New(cors.Options{ //nolint:gocritic // ignore
		AllowedOrigins:   []string{"*"},
		AllowedMethods:   []string{"GET", "POST", "DELETE", "OPTIONS"},
		AllowedHeaders:   []string{"Accept", "Authorization", "Content-Type", "X-CSRF-Token"},
		ExposedHeaders:   []string{"Link"},
		AllowCredentials: true,
		MaxAge:           300, //nolint:mnd // ignore
	})

	router.GET("/doc/*any", swaggerHandler)

	router.Handler(http.MethodGet, "/debug/pprof/*item", http.DefaultServeMux)

	group := router_helper.NewRouteGroup(router, "/api")

	navigatorRoutes := controllers.New(routingService, log, mapMatcherService, tilingService)

	navigatorRoutes.Routes(group)

	var (
		errChan       = make(chan error, 1)
		errProxyChan  = make(chan error, 1)
		wsProxyServer *http.Server
	)

	mux := http.NewServeMux()
	mux.HandleFunc("/ws", api.upstream("online map matcher", "tcp", "127.0.0.1"+":"+strconv.Itoa(config.WebsocketPort)))

	wsProxyServer = &http.Server{
		Addr:    fmt.Sprintf(":%d", config.ProxyPort),
		Handler: mux,
		BaseContext: func(_ net.Listener) context.Context {
			return ctx
		},

		ReadTimeout:       viper.GetDuration("server.read_timeout"),
		WriteTimeout:      config.Timeout + viper.GetDuration("server.write_timeout"),
		IdleTimeout:       viper.GetDuration("server.idle_timeout"),
		ReadHeaderTimeout: viper.GetDuration("server.read_header_timeout"),
	}

	go func() {
		api.log.Info(fmt.Sprintf("WebSocket proxy running on port %d", config.ProxyPort))

		if err := listenAndServeHTTP(wsProxyServer); err != nil {
			errProxyChan <- err
		}
	}()

	var mwChain []alice.Constructor
	if useRateLimit {
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, api.Heartbeat("healthz"), Logger(log), Labels, api.Limit, gziphandler.GzipHandler)
	} else {
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, api.Heartbeat("healthz"), Logger(log), Labels, gziphandler.GzipHandler)
	}
	mainMwChain := alice.New(mwChain...).Then(router)

	srv := http_server.New(ctx, mainMwChain, config, false)
	log.Info(fmt.Sprintf("API run on port %d", config.Port))

	go func() {
		api.handleWebsocket(ctx, config, mapMatcherService,
			false, errChan)
	}()

	serverErr := make(chan error, 1)
	go func() {
		serverErr <- listenAndServeHTTP(srv)
	}()

	shutdownSignals := make(chan os.Signal, 1)
	signal.Notify(
		shutdownSignals,
		syscall.SIGINT,
		syscall.SIGQUIT,
		syscall.SIGTERM,
	)
	defer signal.Stop(shutdownSignals)

	// https://victoriametrics.com/blog/go-graceful-shutdown/
	shutdownCtx, cancelShutdownCtx := context.WithTimeout(context.Background(), shutdownPeriod)
	defer cancelShutdownCtx()
	shutdown := func() {
		err := shutdownHTTP(srv, shutdownCtx)
		if err != nil {
			// dari docs nya:  Make sure the program doesn't exit and waits instead for Shutdown to return.
			log.Error("Failed to wait for ongoing requests to finish, waiting for forced cancellation.")
			shutdownHardCtx, cancelShutdownHardCtx := context.WithTimeout(context.Background(), shutdownPeriod)
			defer cancelShutdownHardCtx()
			err = util.Sleep(shutdownHardCtx, shutdownHardPeriod)
			if err != nil {
				log.Error("Failed to wait for ongoing requests to finish, waiting for forced cancellation.")
			}
		}
		if wsProxyServer != nil {
			if err := shutdownHTTP(wsProxyServer, shutdownCtx); err != nil {
				log.Error("WebSocket proxy shutdown error", zap.Error(err))
			}
		}
	}

	select {
	case err := <-errChan:
		log.Error("Websocket error, shutting down server", zap.Error(err))
		isShuttingDown.Store(true)
		shutdown()
		return err
	case err := <-errProxyChan:
		log.Error("Websocket Proxy error, shutting down server", zap.Error(err))
		isShuttingDown.Store(true)
		shutdown()
		return err
	case err := <-serverErr:
		log.Info("HTTP server stopped", zap.Error(err))
		isShuttingDown.Store(true)
		if wsProxyServer != nil {
			if err := shutdownHTTP(wsProxyServer, shutdownCtx); err != nil {
				log.Error("WebSocket proxy shutdown error", zap.Error(err))
			}
		}
		return err
	case <-ctx.Done():
		log.Info("Context canceled, shutting down server")
		isShuttingDown.Store(true)
		shutdown()
		return ctx.Err()
	case sig := <-shutdownSignals:
		isShuttingDown.Store(true)
		err := util.Sleep(ctx, readinessDrainDelay)
		shutdown()
		api.log.Info("Navigatorx Routing Engine Server Stopped", zap.String("signal", sig.String()))
		return err
	}
}

func swaggerHandler(res http.ResponseWriter, req *http.Request, p httprouter.Params) {
	httpSwagger.WrapHandler(res, req)
}

func NewContext(re controllers.RoutingEngine, rs controllers.RoutingService) (context.Context, func()) {
	ctx, cancel := context.WithCancel(context.Background())
	cb := func() {
		re.Close()
		rs.Close()
		cancel()
	}

	return ctx, cb
}
