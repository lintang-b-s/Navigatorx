package router

import (
	"context"
	"fmt"
	"net/http"

	"github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	router_helper "github.com/lintang-b-s/Navigatorx/pkg/http/router/routerhelper"
	http_server "github.com/lintang-b-s/Navigatorx/pkg/http/server"

	"github.com/julienschmidt/httprouter"
	"github.com/justinas/alice"
	"github.com/rs/cors"
	"go.uber.org/zap"

	_ "github.com/swaggo/http-swagger"

	httpSwagger "github.com/swaggo/http-swagger"
	_ "net/http/pprof"
)

type API struct {
	log *zap.Logger
}

func NewAPI(log *zap.Logger) *API {
	return &API{log: log}
}

//	@title			OSM Search Engine API
//	@version		1.0
//	@description	This is a openstreetmap search engine server.

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

	searcherRoutes := controllers.New(routingService, log)

	searcherRoutes.Routes(group)

	var mwChain []alice.Constructor
	if useRateLimit {
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, Heartbeat("healthz"), Logger(log), Labels, Limit)
	} else {
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, Heartbeat("healthz"), Logger(log), Labels)
	}
	mainMwChain := alice.New(mwChain...).Then(router)

	srv := http_server.New(ctx, mainMwChain, config)
	log.Info(fmt.Sprintf("API run on port %d", config.Port))

	err := srv.ListenAndServe()
	if err != nil {
		return err
	}

	return nil
}

func swaggerHandler(res http.ResponseWriter, req *http.Request, p httprouter.Params) {
	httpSwagger.WrapHandler(res, req)
}
