package controllers

import (
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"strconv"

	"github.com/go-playground/locales/en"
	ut "github.com/go-playground/universal-translator"
	"github.com/go-playground/validator/v10"
	enTranslations "github.com/go-playground/validator/v10/translations/en"
	"github.com/julienschmidt/httprouter"
	helper "github.com/lintang-b-s/Navigatorx/pkg/http/router/routerhelper"
	"go.uber.org/zap"
)

type routingAPI struct {
	routingService     RoutingService
	mapmatchingService MapMatcherService
	log                *zap.Logger
}

func New(routingService RoutingService, mapmatchingService MapMatcherService, log *zap.Logger) *routingAPI {
	return &routingAPI{
		routingService:     routingService,
		log:                log,
		mapmatchingService: mapmatchingService,
	}

}

func (api *routingAPI) Routes(group *helper.RouteGroup) {
	group.GET("/computeRoutes", api.shortestPath)
	group.POST("/onlineMapMatch", api.onlineMapMatch)
	group.GET("/computeAlternativeRoutes", api.alternativeRoutes)
}

func (api *routingAPI) shortestPath(w http.ResponseWriter, r *http.Request, p httprouter.Params) {
	var (
		request shortestPathRequest
		err     error
	)

	query := r.URL.Query()

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
	validate := validator.New()
	if err := validate.Struct(request); err != nil {
		english := en.New()
		uni := ut.New(english, english)
		trans, _ := uni.GetTranslator("en")
		_ = enTranslations.RegisterDefaultTranslations(validate, trans)
		vv := translateError(err, trans)
		vvString := []string{}
		for _, v := range vv {
			vvString = append(vvString, v.Error())
		}
		api.BadRequestResponse(w, r, fmt.Errorf("validation error: %v", vvString))
		return
	}

	travelTime, dist, pathPolyline, drivingDirections, _, err := api.routingService.ShortestPath(request.OriginLat, request.OriginLon,
		request.DestinationLat, request.DestinationLon)
	if err != nil {
		api.getStatusCode(w, r, err)
		return
	}

	headers := make(http.Header)

	if err := api.writeJSON(w, http.StatusOK, envelope{"data": NewShortestPathResponse(travelTime, dist, pathPolyline,
		NewDrivingDirections(drivingDirections))}, headers); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}
}

func (api *routingAPI) alternativeRoutes(w http.ResponseWriter, r *http.Request, p httprouter.Params) {
	var (
		request alternativeRoutesRequest
		err     error
	)

	query := r.URL.Query()

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
	validate := validator.New()
	if err := validate.Struct(request); err != nil {
		english := en.New()
		uni := ut.New(english, english)
		trans, _ := uni.GetTranslator("en")
		_ = enTranslations.RegisterDefaultTranslations(validate, trans)
		vv := translateError(err, trans)
		vvString := []string{}
		for _, v := range vv {
			vvString = append(vvString, v.Error())
		}
		api.BadRequestResponse(w, r, fmt.Errorf("validation error: %v", vvString))
		return
	}

	alternatives, _, err := api.routingService.AlternativeRouteSearch(request.OriginLat, request.OriginLon,
		request.DestinationLat, request.DestinationLon, int(request.K))
	if err != nil {
		api.getStatusCode(w, r, err)
		return
	}

	headers := make(http.Header)

	if err := api.writeJSON(w, http.StatusOK, envelope{"data": NewAlternativeRoutesResponse(alternatives)}, headers); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}
}

func (api *routingAPI) onlineMapMatch(w http.ResponseWriter, r *http.Request, p httprouter.Params) {
	var (
		request mapMatchRequest
		err     error
	)
	err = json.NewDecoder(r.Body).Decode(&request)
	if err != nil {
		api.BadRequestResponse(w, r, err)
		return
	}
	if err := r.Body.Close(); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}

	validate := validator.New()

	if err := validate.Struct(request); err != nil {
		english := en.New()
		uni := ut.New(english, english)
		trans, _ := uni.GetTranslator("en")
		_ = enTranslations.RegisterDefaultTranslations(validate, trans)
		vv := translateError(err, trans)
		vvString := []string{}
		for _, v := range vv {
			vvString = append(vvString, v.Error())
		}
		api.BadRequestResponse(w, r, fmt.Errorf("validation error: %v", vvString))
		return
	}

	mgpsPoint, cands, speedMeanK, speedStdK := api.mapmatchingService.OnlineMapMatch(request.Gps.ToDataGPS(), request.K, ToOnlineCandidates(request.Candidates),
		request.SpeedMeanK, request.SpeedStdK, request.LastBearing)
	headers := make(http.Header)

	if err := api.writeJSON(w, http.StatusOK, envelope{"data": NewMapmatchingResponse(mgpsPoint, cands, speedMeanK,
		speedStdK, mgpsPoint.GetBearing())}, headers); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}

}
