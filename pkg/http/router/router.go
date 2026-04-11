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
	"github.com/bytedance/gopkg/util/logger"
	"github.com/julienschmidt/httprouter"
	"github.com/justinas/alice"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
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

func NewAPI(log *zap.Logger) *API {
	return &API{log: log}
}

//	@title			Navigatorx API
//	@version		1.0
//	@description	This is an traffic aware routing engine for openstreetmap server.

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
	shutdownPeriod time.Duration,
) error {
	ctx, cleanup := NewContext(routingService.GetRoutingEngine(), routingService)
	defer cleanup()
	log.Info("Run httprouter API")

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

	navigatorRoutes := controllers.New(routingService, log, mapMatcherService)

	navigatorRoutes.Routes(group)

	var (
		errChan      chan error = make(chan error, 1)
		errProxyChan chan error = make(chan error, 1)
		wsServer     *http.Server
	)

	go func() {
		mux := http.NewServeMux()
		mux.HandleFunc("/ws", api.upstream("online map matcher", "tcp", "localhost"+":"+strconv.Itoa(config.WebsocketPort)))

		wsServer = &http.Server{
			Addr:    fmt.Sprintf(":%d", config.ProxyPort),
			Handler: mux,
			BaseContext: func(_ net.Listener) context.Context {
				return ctx
			},

			ReadTimeout:       viper.GetDuration("HTTP_SERVER_READ_TIMEOUT"),
			WriteTimeout:      config.Timeout + viper.GetDuration("HTTP_SERVER_WRITE_TIMEOUT"),
			IdleTimeout:       viper.GetDuration("HTTP_SERVER_IDLE_TIMEOUT"),
			ReadHeaderTimeout: viper.GetDuration("HTTP_SERVER_READ_HEADER_TIMEOUT"),
		}
		api.log.Info(fmt.Sprintf("WebSocket proxy running on port %d", config.ProxyPort))

		if err := wsServer.ListenAndServe(); err != nil {
			errProxyChan <- err
		}
	}()

	var mwChain []alice.Constructor
	if useRateLimit {
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, Heartbeat("healthz"), Logger(log), Labels, Limit, gziphandler.GzipHandler)
	} else {
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, Heartbeat("healthz"), Logger(log), Labels, gziphandler.GzipHandler)
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
		serverErr <- srv.ListenAndServe()
	}()

	// https://victoriametrics.com/blog/go-graceful-shutdown/
	shutdownCtx, cancelShutdownCtx := context.WithTimeout(context.Background(), shutdownPeriod)
	defer cancelShutdownCtx()
	shutdown := func() {
		err := srv.Shutdown(shutdownCtx)
		if err != nil {
			// dari docs nya:  Make sure the program doesn't exit and waits instead for Shutdown to return.
			log.Error("Failed to wait for ongoing requests to finish, waiting for forced cancellation.")
			shutdownHardCtx, cancelShutdownHardCtx := context.WithTimeout(context.Background(), shutdownPeriod)
			defer cancelShutdownHardCtx()
			util.Sleep(shutdownHardCtx, shutdownHardPeriod)
		}
		if wsServer != nil {
			if err := wsServer.Shutdown(shutdownCtx); err != nil {
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
		if wsServer != nil {
			if err := wsServer.Shutdown(shutdownCtx); err != nil {
				log.Error("WebSocket proxy shutdown error", zap.Error(err))
			}
		}
		return err
	case <-ctx.Done():
		log.Info("Context canceled, shutting down server")
		isShuttingDown.Store(true)
		shutdown()
		return ctx.Err()
	case sig := <-GracefulShutdown():
		isShuttingDown.Store(true)
		util.Sleep(ctx, readinessDrainDelay)
		shutdown()
		logger.Info("Navigatorx Routing Engine Server Stopped", zap.String("signal", sig.String()))
		return nil
	}
}

func swaggerHandler(res http.ResponseWriter, req *http.Request, p httprouter.Params) {
	httpSwagger.WrapHandler(res, req)
}

func NewContext(re *routing.CRPRoutingEngine, rs controllers.RoutingService) (context.Context, func()) {
	ctx, cancel := context.WithCancel(context.Background())
	cb := func() {
		re.Close()
		rs.Close()
		cancel()
	}

	return ctx, cb
}

func GracefulShutdown() <-chan os.Signal {
	shutdownSignals := make(chan os.Signal, 1)

	signal.Notify(
		shutdownSignals,
		syscall.SIGINT,
		syscall.SIGQUIT,
		syscall.SIGTERM,
	)

	return shutdownSignals
}
