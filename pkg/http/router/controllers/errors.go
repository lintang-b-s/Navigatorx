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

func (api *RoutingAPI) logError(r *http.Request, err error) {

	api.log.Error("internal server error", zap.Error(err), zap.String("request_method", r.Method),
		zap.String("request_uri", r.URL.String()))
}

// errorResponse method for sending JSON-formatted error messages to the client with a given status code.
func (api *RoutingAPI) errorResponse(w http.ResponseWriter, r *http.Request, status int, message any,
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

// ServerErrorResponse writes a server error response and logs the underlying error.
func (api *RoutingAPI) ServerErrorResponse(w http.ResponseWriter, r *http.Request, err error) {

	api.logError(r, err)

	message := "the server encountered a problem and could not process your request"
	api.errorResponse(w, r, 500, message)
}

// NotFoundResponse writes a not-found response.
func (api *RoutingAPI) NotFoundResponse(w http.ResponseWriter, r *http.Request) {
	message := "the requested resource could not be found"
	api.errorResponse(w, r, http.StatusNotFound, message)
}

// MethodNotAllowedResponse writes a response for an unsupported HTTP method.
func (api *RoutingAPI) MethodNotAllowedResponse(w http.ResponseWriter, r *http.Request) {
	message := fmt.Sprintf("the %s method is not supported this resource", r.Method)
	api.errorResponse(w, r, http.StatusMethodNotAllowed, message)
}

// BadRequestResponse writes a bad-request response with the supplied message.
func (api *RoutingAPI) BadRequestResponse(w http.ResponseWriter, r *http.Request, err error) {
	api.errorResponse(w, r, http.StatusBadRequest, err.Error())
}

// EditConflictResponse writes a conflict response.
func (api *RoutingAPI) EditConflictResponse(w http.ResponseWriter, r *http.Request) {
	message := "unable to update the record due to an edit conflict, please try again"
	api.errorResponse(w, r, http.StatusConflict, message)
}

// InvalidCredentialsResponse writes an unauthorized response for invalid credentials.
func (api *RoutingAPI) InvalidCredentialsResponse(w http.ResponseWriter, r *http.Request) {
	message := "invalid authentication credentials"
	api.errorResponse(w, r, http.StatusUnauthorized, message)
}

// InvalidAuthenticationTokenResponse writes an unauthorized response for an invalid token.
func (api *RoutingAPI) InvalidAuthenticationTokenResponse(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("WWWW-Authenticate", "Bearer")

	message := "invalid or missing authentication token"
	api.errorResponse(w, r, http.StatusUnauthorized, message)
}
