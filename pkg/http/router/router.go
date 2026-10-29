package router

import (
	"context"
	"fmt"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/NYTimes/gziphandler"
	"github.com/julienschmidt/httprouter"
	"github.com/justinas/alice"
	"github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	router_helper "github.com/lintang-b-s/Navigatorx/pkg/http/router/routerhelper"
	http_server "github.com/lintang-b-s/Navigatorx/pkg/http/server"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/rs/cors"
	"github.com/spf13/viper"
	httpSwagger "github.com/swaggo/http-swagger"
	"go.uber.org/zap"

	"net/http"
	"net/http/pprof"
)

// API represents http api.
type API struct {
	log *zap.Logger
}

var (
	listenAndServeHTTP = func(s *http.Server) error { return s.ListenAndServe() }
	shutdownHTTP       = func(s *http.Server, ctx context.Context) error { return s.Shutdown(ctx) }
)

// NewAPI creates and returns http api.
func NewAPI(log *zap.Logger) *API {
	return &API{log: log}
}

//	@title			Navigatorx API
//	@version		1.0
//	@description	This is a routing engine for openstreetmap .

//	@contact.name	Lintang Birda Saputra
//	@contact.url	_
//	@contact.email	lintangbirdasaputra23@gmail.com

//	@license.name	MIT License
//	@license.url	https://opensource.org/license/mit

// Run runs .
// @host		localhost
// @BasePath	/api
func (api *API) Run(
	config http_server.Config,
	log *zap.Logger,

	useRateLimit bool,
	routingService controllers.RoutingService,
	mapAttributesService controllers.MapAttributesService,
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
	viper.SetDefault("server.idle_timeout", "240s") // keep-alive  timeout
	viper.SetDefault("server.read_header_timeout", "2s")
	viper.SetDefault("enable_pprof", false)
	viper.SetDefault("server.allowed_origins", []string{"*"})

	log.Info("Run httprouter API")

	routingService.InitBackgroundWorker(ctx)
	router := httprouter.New()

	corsHandler := cors.New(cors.Options{ //nolint:gocritic // ignore
		AllowedOrigins:   viper.GetStringSlice("server.allowed_origins"),
		AllowedMethods:   []string{"GET", "POST", "OPTIONS"},
		AllowedHeaders:   []string{"Accept", "Authorization", "Content-Type", "X-CSRF-Token"},
		ExposedHeaders:   []string{"Link"},
		AllowCredentials: true,
		MaxAge:           300, //nolint:mnd // ignore
	})

	router.GET("/doc/*any", swaggerHandler)

	if viper.GetBool("enable_pprof") {
		log.Warn("pprof endpoint enabled at /debug/pprof/*item")

		mux := http.NewServeMux()
		mux.HandleFunc("/debug/pprof/", pprof.Index)
		mux.HandleFunc("/debug/pprof/cmdline", pprof.Cmdline)
		mux.HandleFunc("/debug/pprof/profile", pprof.Profile)
		mux.HandleFunc("/debug/pprof/symbol", pprof.Symbol)
		mux.HandleFunc("/debug/pprof/trace", pprof.Trace)

		router.Handler(http.MethodGet, "/debug/pprof/*item", mux)
		router.Handler(http.MethodPost, "/debug/pprof/*item", mux) // Required by `go tool pprof` for symbol lookups
	}

	group, err := router_helper.NewRouteGroup(router, "/api")
	if err != nil {
		return fmt.Errorf("api.Run: failed to NewRouteGroup: %w", err)
	}
	navigatorRoutes := controllers.New(routingService, log, mapAttributesService)

	navigatorRoutes.Routes(group)

	var mwChain []alice.Constructor
	const healthEndpoint = "healthz"

	if useRateLimit {
		// useRateLimit = true ->  dideploy pakai nginx reverse proxy (udah ada gzip nya).
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, api.Heartbeat(healthEndpoint), Logger(log), Labels, api.Limit)
	} else {
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, api.Heartbeat(healthEndpoint), Logger(log), Labels, gziphandler.GzipHandler)
	}

	mainMwChain := alice.New(mwChain...).Then(router)

	srv := http_server.New(ctx, mainMwChain, config)
	log.Info(fmt.Sprintf("API run on port %d", config.Port))

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
	}

	select {
	case err := <-serverErr:
		log.Info("HTTP server stopped", zap.Error(err))
		isShuttingDown.Store(true)
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

func swaggerHandler(res http.ResponseWriter, req *http.Request, _ httprouter.Params) {
	httpSwagger.WrapHandler(res, req)
}

// NewContext creates and returns context.
func NewContext(re controllers.RoutingEngine, rs controllers.RoutingService) (context.Context, func()) {
	ctx, cancel := context.WithCancel(context.Background())
	cb := func() {
		re.Close()
		rs.Close()
		cancel()
	}

	return ctx, cb
}
