package controllers

import (
	"encoding/json"
	"errors"
	"net/http"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

// writeJSON marshals data structure to encoded JSON response.
func (api *routingAPI) writeJSON(w http.ResponseWriter, status int, data envelope,
	headers http.Header) error {
	js, err := json.MarshalIndent(data, "", "\t")
	if err != nil {
		return err
	}

	if err != nil {
		return err
	}

	js = append(js, '\n')
	for key, value := range headers {
		w.Header()[key] = value
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	if _, err := w.Write(js); err != nil {
		api.log.Error("failed to write JSON response", zap.Error(err))
		return err
	}

	return nil
}

type messageResponse struct {
	Message string `json:"message"`
}

func NewMessageResponse(msg string) messageResponse {
	return messageResponse{msg}
}

func (api *routingAPI) getStatusCode(w http.ResponseWriter, r *http.Request, err error) {
	if err == nil {
		headers := make(http.Header)

		if err := api.writeJSON(w, http.StatusOK, envelope{"data": NewMessageResponse("success")}, headers); err != nil {
			api.ServerErrorResponse(w, r, err)
		}
	}
	var ierr *util.Error
	if !errors.As(err, &ierr) {
		api.ServerErrorResponse(w, r, err)
	} else {
		switch ierr.Code() {
		case util.ErrInternalServerError:
			api.ServerErrorResponse(w, r, err)
		case util.ErrNotFound:
			api.NotFoundResponse(w, r)
		case util.ErrConflict:
			api.EditConflictResponse(w, r)
		case util.ErrBadParamInput:
			errMsg := errors.New(err.Error())
			api.BadRequestResponse(w, r, errMsg)
		default:
			api.ServerErrorResponse(w, r, err)
		}
	}
}
