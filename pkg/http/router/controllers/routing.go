package controllers

import (
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
	routingService RoutingService
	log            *zap.Logger
}

func New(routingService RoutingService, log *zap.Logger) *routingAPI {
	return &routingAPI{
		routingService: routingService,
		log:            log,
	}

}

func (api *routingAPI) Routes(group *helper.RouteGroup) {
	group.GET("/computeRoutes", api.shortestPath)
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

	eta, dist, pathPolyline, _, err := api.routingService.ShortestPath(request.OriginLat, request.OriginLon,
		request.DestinationLat, request.DestinationLon)
	if err != nil {
		api.getStatusCode(w, r, err)
		return
	}

	headers := make(http.Header)

	if err := api.writeJSON(w, http.StatusOK, envelope{"data": NewShortestPathResponse(eta, dist, pathPolyline)}, headers); err != nil {
		api.ServerErrorResponse(w, r, err)
		return
	}
}
