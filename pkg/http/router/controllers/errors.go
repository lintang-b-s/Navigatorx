package controllers

import (
	"fmt"
	"net/http"

	"go.uber.org/zap"
)

type errorEnvelope struct {
	Error errorBody `json:"error"`
}

type errorBody struct {
	Code    string `json:"code"`
	Message string `json:"message"`
}

func (api *routingAPI) logError(r *http.Request, err error) {

	api.log.Error("internal server error", zap.Error(err), zap.String("request_method", r.Method),
		zap.String("request_uri", r.URL.String()))
}

// errorResponse method for sending JSON-formatted error messages to the client with a given status code.
func (api *routingAPI) errorResponse(w http.ResponseWriter, r *http.Request, status int, message interface{},
) {
	env := errorEnvelope{Error: errorBody{
		Code:    http.StatusText(status),
		Message: message.(string),
	}}

	err := api.writeJSON(w, status, env)
	if err != nil {

		api.logError(r, err)
		w.WriteHeader(500)
	}
}

func (api *routingAPI) ServerErrorResponse(w http.ResponseWriter, r *http.Request, err error) {

	api.logError(r, err)

	message := "the server encountered a problem and could not process your request"
	api.errorResponse(w, r, 500, message)
}

func (api *routingAPI) NotFoundResponse(w http.ResponseWriter, r *http.Request) {
	message := "the requested resource could not be found"
	api.errorResponse(w, r, http.StatusNotFound, message)
}

func (api *routingAPI) MethodNotAllowedResponse(w http.ResponseWriter, r *http.Request) {
	message := fmt.Sprintf("the %s method is not supported this resource", r.Method)
	api.errorResponse(w, r, http.StatusMethodNotAllowed, message)
}

func (api *routingAPI) BadRequestResponse(w http.ResponseWriter, r *http.Request, err error) {
	api.errorResponse(w, r, http.StatusBadRequest, err.Error())
}

func (api *routingAPI) EditConflictResponse(w http.ResponseWriter, r *http.Request) {
	message := "unable to update the record due to an edit conflict, please try again"
	api.errorResponse(w, r, http.StatusConflict, message)
}

func (api *routingAPI) InvalidCredentialsResponse(w http.ResponseWriter, r *http.Request) {
	message := "invalid authentication credentials"
	api.errorResponse(w, r, http.StatusUnauthorized, message)
}

func (api *routingAPI) InvalidAuthenticationTokenResponse(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("WWWW-Authenticate", "Bearer")

	message := "invalid or missing authentication token"
	api.errorResponse(w, r, http.StatusUnauthorized, message)
}
