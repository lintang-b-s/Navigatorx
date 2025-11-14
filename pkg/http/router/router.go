package router

import (
	"context"
	"fmt"
	"net"
	"net/http"
	"strconv"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	router_helper "github.com/lintang-b-s/Navigatorx/pkg/http/router/routerhelper"
	http_server "github.com/lintang-b-s/Navigatorx/pkg/http/server"
	"github.com/mailru/easygo/netpoll"
	"github.com/spf13/viper"

	"github.com/julienschmidt/httprouter"
	"github.com/justinas/alice"
	"github.com/rs/cors"
	"go.uber.org/zap"

	_ "github.com/swaggo/http-swagger"

	httpSwagger "github.com/swaggo/http-swagger"
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
//	@contact.email	lintang.birda.saputra@mail.ugm.ac.id

//	@license.name	BSD License
//	@license.url	https://opensource.org/license/bsd-2-clause

// @host		localhost
// @BasePath	/api
func (api *API) Run(
	ctx context.Context,
	config http_server.Config,
	log *zap.Logger,

	useRateLimit bool,
	routingService controllers.RoutingService,
	mapMatcherService controllers.MapMatcherService,
) error {
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

	navigatorRoutes := controllers.New(routingService, log)

	navigatorRoutes.Routes(group)

	var (
		errChan      chan error = make(chan error)
		errProxyChan chan error = make(chan error)
		wsServer     *http.Server
	)

	go func() {
		api.handleWebsocket(ctx, config, mapMatcherService,
			false, errChan)
	}()

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
			RealIP, Heartbeat("healthz"), Logger(log), Labels, Limit)
	} else {
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, Heartbeat("healthz"), Logger(log), Labels)
	}
	mainMwChain := alice.New(mwChain...).Then(router)

	srv := http_server.New(ctx, mainMwChain, config, false)
	log.Info(fmt.Sprintf("API run on port %d", config.Port))

	serverErr := make(chan error, 1)
	go func() {
		serverErr <- srv.ListenAndServe()
	}()

	select {
	case err := <-errChan:
		log.Error("Websocket error, shutting down server", zap.Error(err))
		_ = srv.Shutdown(ctx)
		wsServer.Shutdown(ctx)
		return err
	case err := <-errProxyChan:
		log.Error("Websocket error, shutting down server", zap.Error(err))
		_ = srv.Shutdown(ctx)
		wsServer.Shutdown(ctx)
		return err
	case err := <-serverErr:
		log.Info("HTTP server stopped", zap.Error(err))
		wsServer.Shutdown(ctx)
		return err

	case <-ctx.Done():
		log.Info("Context canceled, shutting down server")
		_ = srv.Shutdown(context.Background())
		wsServer.Shutdown(ctx)
		return ctx.Err()
	}
}

func swaggerHandler(res http.ResponseWriter, req *http.Request, p httprouter.Params) {
	httpSwagger.WrapHandler(res, req)
}
