package controllers

import (
	"errors"
	"fmt"
	"net/http"
	"strconv"

	"github.com/mmcloughlin/geohash"

	"github.com/go-playground/locales/en"
	ut "github.com/go-playground/universal-translator"
	"github.com/go-playground/validator/v10"
	enTranslations "github.com/go-playground/validator/v10/translations/en"
	"github.com/julienschmidt/httprouter"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	helper "github.com/lintang-b-s/Navigatorx/pkg/http/router/routerhelper"
	"go.uber.org/zap"
)

type routingAPI struct {
	routingService RoutingService
	tilingService  TilingService

	log *zap.Logger

	validate *validator.Validate

	trans ut.Translator
}

func New(routingService RoutingService, log *zap.Logger, tilingService TilingService) *routingAPI {
	validate := validator.New()
	english := en.New()
	uni := ut.New(english, english)
	trans, _ := uni.GetTranslator("en")
	_ = enTranslations.RegisterDefaultTranslations(validate, trans)

	return &routingAPI{
		routingService: routingService,
		log:            log,
		validate:       validate,
		tilingService:  tilingService,
		trans:          trans,
	}

}

func (api *routingAPI) Routes(group *helper.RouteGroup) {
	group.GET("/computeRoutes", api.shortestPath)
	group.GET("/computeAlternativeRoutes", api.AlternativeRoutes)
	group.GET("/boundingBox", api.GetBoundingBox)

	group.GET("/tile/:userGeohash", api.getTile)
	group.GET("/tile-init", api.initClientSideRealTimeMapMatching)
	group.GET("/tile-init-transition-matrix", api.initClientSideRealTimeMapMatchingTransitionMatrix)
}

func (api *routingAPI) shortestPath(w http.ResponseWriter, r *http.Request, p httprouter.Params) {
	var (
		request shortestPathRequest
		err     error
	)

	query := r.URL.Query()
	useAnnotation := false
	useAnnotationStr := query.Get("useAnnotation")
	if useAnnotationStr != "" {
		useAnnotation, err = strconv.ParseBool(useAnnotationStr)
		if err != nil {
			api.BadRequestResponse(w, r, errors.New("useAnnotation must be boolean"))
			return
		}
	}

	request.OriginLat, err = strconv.ParseFloat(query.Get("origin_lat"), 64)
	if err != nil {
		api.BadRequestResponse(w, r, errors.New("origin_lat is required and must be a valid float"))
		return
	}
	request.OriginLon, err = strconv.ParseFloat(query.Get("origin_lon"), 64)
	if err != nil {
		api.BadRequestResponse(w, r, errors.New("origin_lon is required and must be a valid float"))
		return
	}
	request.DestinationLat, err = strconv.ParseFloat(query.Get("destination_lat"), 64)
	if err != nil {
		api.BadRequestResponse(w, r, errors.New("destination_lat is required and must be a valid float"))
		return
	}
	request.DestinationLon, err = strconv.ParseFloat(query.Get("destination_lon"), 64)
	if err != nil {
		api.BadRequestResponse(w, r, errors.New("destination_lon is required and must be a valid float"))
		return
	}

	if err := api.validate.Struct(request); err != nil {
		vv := translateError(err, api.trans)
		vvString := []string{}
		for _, v := range vv {
			vvString = append(vvString, v.Error())
		}
		api.BadRequestResponse(w, r, fmt.Errorf("validation error: %v", vvString))
		return
	}

	startEdgeId := da.INVALID_EDGE_ID
	rerouteStr := query.Get("reroute")
	reroute := false
	if rerouteStr != "" {
		reroute, err = strconv.ParseBool(rerouteStr)
		if err != nil {
			api.BadRequestResponse(w, r, errors.New("reroute must be boolean"))
			return
		}

		startEdgeId, err = da.ParseIndex(query.Get("start_edge_id"))
		if err != nil {
			api.BadRequestResponse(w, r, errors.New("edgeId must be unsigned integer 32 bit"))
			return
		}
	}

	newCtx := r.Context()

	travelTime, dist, pathPolyline, drivingDirections, ok, err := api.routingService.ShortestPath(newCtx, request.OriginLat, request.OriginLon,
		request.DestinationLat, request.DestinationLon, reroute, startEdgeId)
	if err != nil {
		api.getStatusCode(w, r, err)
		return
	}

	if !ok {
		api.NotFoundResponse(w, r)
		return
	}

	headers := make(http.Header)

	if err := api.writeJSON(w, http.StatusOK, envelope{"data": NewShortestPathResponse(travelTime, dist, pathPolyline,
		NewDrivingDirections(drivingDirections, useAnnotation))}, headers); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}
}

func (api *routingAPI) AlternativeRoutes(w http.ResponseWriter, r *http.Request, p httprouter.Params) {
	var (
		request alternativeRoutesRequest
		err     error
	)

	query := r.URL.Query()
	useAnnotation := false
	useAnnotationStr := query.Get("useAnnotation")
	if useAnnotationStr != "" {
		useAnnotation, err = strconv.ParseBool(useAnnotationStr)
		if err != nil {
			api.BadRequestResponse(w, r, errors.New("useAnnotation must be boolean"))
			return
		}
	}

	request.OriginLat, err = strconv.ParseFloat(query.Get("origin_lat"), 64)
	if err != nil {
		api.BadRequestResponse(w, r, errors.New("origin_lat is required and must be a valid float"))
		return
	}
	request.OriginLon, err = strconv.ParseFloat(query.Get("origin_lon"), 64)
	if err != nil {
		api.BadRequestResponse(w, r, errors.New("origin_lon is required and must be a valid float"))
		return
	}
	request.DestinationLat, err = strconv.ParseFloat(query.Get("destination_lat"), 64)
	if err != nil {
		api.BadRequestResponse(w, r, errors.New("destination_lat is required and must be a valid float"))
		return
	}
	request.DestinationLon, err = strconv.ParseFloat(query.Get("destination_lon"), 64)
	if err != nil {
		api.BadRequestResponse(w, r, errors.New("destination_lon is required and must be a valid float"))
		return
	}
	request.K, err = strconv.ParseInt(query.Get("k"), 10, 64)
	if err != nil {
		api.BadRequestResponse(w, r, errors.New("number of alternatives k is required and must be a valid int"))
		return
	}
	if err := api.validate.Struct(request); err != nil {

		vv := translateError(err, api.trans)
		vvString := []string{}
		for _, v := range vv {
			vvString = append(vvString, v.Error())
		}
		api.BadRequestResponse(w, r, fmt.Errorf("validation error: %v", vvString))
		return
	}

	newCtx := r.Context()

	startEdgeId := da.INVALID_EDGE_ID
	rerouteStr := query.Get("reroute")
	reroute := false
	if rerouteStr != "" {
		reroute, err = strconv.ParseBool(rerouteStr)
		if err != nil {
			api.BadRequestResponse(w, r, errors.New("reroute must be boolean"))
			return
		}

		startEdgeId, err = da.ParseIndex(query.Get("start_edge_id"))
		if err != nil {
			api.BadRequestResponse(w, r, errors.New("edgeId must be unsigned integer 32 bit"))
			return
		}
	}

	alternatives, err := api.routingService.AlternativeRouteSearch(newCtx, request.OriginLat, request.OriginLon,
		request.DestinationLat, request.DestinationLon, int(request.K), reroute, startEdgeId)
	if err != nil {
		api.getStatusCode(w, r, err)
		return
	}

	headers := make(http.Header)

	if err := api.writeJSON(w, http.StatusOK, envelope{"data": NewAlternativeRoutesResponse(alternatives, useAnnotation)}, headers); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}
}

func (api *routingAPI) GetBoundingBox(w http.ResponseWriter, r *http.Request, p httprouter.Params) {

	newCtx := r.Context()

	bb := api.routingService.GetBoundingBox(newCtx)

	headers := make(http.Header)

	if err := api.writeJSON(w, http.StatusOK, envelope{"data": NewBoundingBox(bb)}, headers); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}
}

func (api *routingAPI) getTile(w http.ResponseWriter, r *http.Request, p httprouter.Params) {
	userGeohash := p.ByName("userGeohash")
	if userGeohash == "" {
		api.BadRequestResponse(w, r, errors.New("userGeohash is required"))
		return
	}

	// validate request
	if err := geohash.Validate(userGeohash); err != nil {
		api.BadRequestResponse(w, r, err)
		return
	}

	ctx := r.Context()
	tileFilePath := api.tilingService.GetTileFilePath(ctx, userGeohash)
	http.ServeFile(w, r, tileFilePath)
}

func (api *routingAPI) initClientSideRealTimeMapMatching(w http.ResponseWriter, r *http.Request, p httprouter.Params) {
	headers := make(http.Header)

	numberOfVertices := api.tilingService.GetNumberOfVertices(r.Context())
	if err := api.writeJSON(w, http.StatusOK, envelope{"data": NewStartClientSideRealtimeMapMatchingResponse(numberOfVertices)}, headers); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}
}

func (api *routingAPI) initClientSideRealTimeMapMatchingTransitionMatrix(w http.ResponseWriter, r *http.Request, p httprouter.Params) {
	http.ServeFile(w, r, GetMapMatchingTransitionFile())
}
