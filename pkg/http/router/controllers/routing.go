package controllers

import (
	"encoding/xml"
	"errors"
	"fmt"
	"net/http"
	"strconv"
	"time"

	"github.com/mmcloughlin/geohash"
	"github.com/spf13/viper"

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

	trans              ut.Translator
	maxRequestBodySize int64
}

func New(routingService RoutingService, log *zap.Logger, tilingService TilingService) *routingAPI {
	validate := validator.New()
	english := en.New()
	uni := ut.New(english, english)
	trans, _ := uni.GetTranslator("en")
	_ = enTranslations.RegisterDefaultTranslations(validate, trans)

	viper.SetDefault("server.max_request_body_size", int64(1))
	maxRequestBodySize := viper.GetInt64("server.max_request_body_size") * MB_TO_BYTES

	return &routingAPI{
		routingService:     routingService,
		log:                log,
		validate:           validate,
		tilingService:      tilingService,
		trans:              trans,
		maxRequestBodySize: maxRequestBodySize,
	}

}

func (api *routingAPI) Routes(group *helper.RouteGroup) {
	group.GET("/computeRoutes", api.shortestPath)
	group.GET("/computeAlternativeRoutes", api.AlternativeRoutes)
	group.GET("/boundingBox", api.GetBoundingBox)

	group.GET("/tile/:userGeohash", api.getTile)
	group.GET("/tile-init", api.initClientSideRealTimeMapMatching)
	group.GET("/tile-init-transition-matrix", api.initClientSideRealTimeMapMatchingTransitionMatrix)
	group.GET("/tile-init-transition-matrix/", api.initClientSideRealTimeMapMatchingTransitionMatrix)
	group.POST("/mapmatching", api.offlineMapMatching) // offline map-matching
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

	useSteps := true
	useStepsStr := query.Get("useSteps")
	if useStepsStr != "" {
		useSteps, err = strconv.ParseBool(useStepsStr)
		if err != nil {
			api.BadRequestResponse(w, r, errors.New("useSteps must be boolean"))
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
		request.DestinationLat, request.DestinationLon, reroute, startEdgeId, useAnnotation, useSteps)
	if err != nil {
		api.getStatusCode(w, r, err)
		return
	}

	if !ok {
		api.NotFoundResponse(w, r)
		return
	}

	response := shortestPathEnvelope{Data: NewShortestPathResponse(
		travelTime,
		dist,
		pathPolyline,
		NewDrivingDirections(drivingDirections, useAnnotation),
	)}
	if err := api.writeJSON(w, http.StatusOK, response); err != nil {
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

	useSteps := true
	useStepsStr := query.Get("useSteps")
	if useStepsStr != "" {
		useSteps, err = strconv.ParseBool(useStepsStr)
		if err != nil {
			api.BadRequestResponse(w, r, errors.New("useSteps must be boolean"))
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
		request.DestinationLat, request.DestinationLon, int(request.K), reroute, startEdgeId, useAnnotation, useSteps)
	if err != nil {
		api.getStatusCode(w, r, err)
		return
	}

	response := alternativeRoutesEnvelope{Data: NewAlternativeRoutesResponse(alternatives, useAnnotation)}
	if err := api.writeJSON(w, http.StatusOK, response); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}
}

func (api *routingAPI) GetBoundingBox(w http.ResponseWriter, r *http.Request, p httprouter.Params) {

	newCtx := r.Context()

	bb := api.routingService.GetBoundingBox(newCtx)

	if err := api.writeJSON(w, http.StatusOK, boundingBoxEnvelope{Data: NewBoundingBox(bb)}); err != nil {
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
	numberOfVertices := api.tilingService.GetNumberOfVertices(r.Context())
	if err := api.writeJSON(w, http.StatusOK, startClientSideRealtimeMapMatchingEnvelope{Data: *NewStartClientSideRealtimeMapMatchingResponse(numberOfVertices)}); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}
}

func (api *routingAPI) initClientSideRealTimeMapMatchingTransitionMatrix(w http.ResponseWriter, r *http.Request, p httprouter.Params) {
	r.Header.Del("If-Modified-Since")
	r.Header.Del("If-None-Match")
	w.Header().Set("Cache-Control", "no-store, no-cache, must-revalidate")
	w.Header().Set("Pragma", "no-cache")
	w.Header().Set("Expires", "0")
	http.ServeFile(w, r, GetMapMatchingTransitionFile())
}

func (api *routingAPI) offlineMapMatching(w http.ResponseWriter, r *http.Request, p httprouter.Params) {

	r.Body = http.MaxBytesReader(w, r.Body, api.maxRequestBodySize)

	var gpx GPX
	err := xml.NewDecoder(r.Body).Decode(&gpx)
	if err != nil {
		api.BadRequestResponse(w, r, fmt.Errorf("failed to parse GPX XML: %w", err))
		return
	}

	pts := gpx.Trk.TrkSeg.TrkPts
	if len(pts) == 0 {
		api.BadRequestResponse(w, r, errors.New("GPX contains no track points"))
		return
	}
	query := r.URL.Query()
	rawGPSRadiuses, hasGPSRadiuses, err := getRawQueryValue(r.URL.RawQuery, "gps_radiuses")

	if err != nil {
		api.BadRequestResponse(w, r, err)
		return
	}
	if hasGPSRadiuses {
		query.Set("gps_radiuses", rawGPSRadiuses)
	}
	if rawIncludeRoutePath := query.Get("include_route_path"); rawIncludeRoutePath != "" {
		if _, err := strconv.ParseBool(rawIncludeRoutePath); err != nil {
			api.BadRequestResponse(w, r, errors.New("include_route_path must be a boolean value"))
			return
		}
	}

	gpsRadiusesM, err := parseGPSRadiuses(query, len(pts))
	if err != nil {
		api.BadRequestResponse(w, r, err)
		return
	}

	gpsPoints := make([]*da.GPSPoint, len(pts))
	baseTime := time.Now()
	for i, pt := range pts {
		if pt.Lat < -90 || pt.Lat > 90 || pt.Lon < -180 || pt.Lon > 180 {
			api.BadRequestResponse(w, r, fmt.Errorf("invalid trackpoint coordinates at index %d: lat=%f, lon=%f", i, pt.Lat, pt.Lon))
			return
		}

		var ptTime time.Time
		if pt.Time != "" {
			var err error
			ptTime, err = time.Parse(time.RFC3339, pt.Time)
			if err != nil {
				ptTime, err = time.Parse(time.RFC3339Nano, pt.Time)
				if err != nil {
					api.BadRequestResponse(w, r, fmt.Errorf("invalid time format '%s' at trackpoint index %d: %w", pt.Time, i, err))
					return
				}
			}
		} else {
			ptTime = baseTime.Add(time.Duration(i) * time.Second)
		}

		gpsPoints[i] = da.NewGPSPoint(pt.Lat, pt.Lon, ptTime, 0, 0)
	}

	matchedPoints, routePath, err := api.routingService.OfflineMapMatch(r.Context(), gpsPoints, gpsRadiusesM)
	if err != nil {
		api.getStatusCode(w, r, err)
		return
	}

	if err := api.writeJSON(w, http.StatusOK, offlineMapMatchingEnvelope{Data: *NewOfflineMapMatchingResponse(matchedPoints, routePath)}); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}
}
