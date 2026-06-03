package controllers

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"net/http"
	"net/url"
	"strconv"
	"strings"
	"sync"

	json "github.com/bytedance/sonic"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

var bufPool = sync.Pool{
	New: func() any {
		return bytes.NewBuffer(make([]byte, 0, JSON_BUF_POOL_SIZE))
	},
}

// writeJSON marshals data structure to encoded JSON response.
func (api *routingAPI) writeJSON(w http.ResponseWriter, status int, data envelope,
	headers http.Header) error {
	buf := bufPool.Get().(*bytes.Buffer)

	defer func() {
		bufPool.Put(buf)
		buf.Reset()
	}()

	enc := json.ConfigDefault.NewEncoder(buf)
	enc.SetIndent("", "\t")

	if err := enc.Encode(data); err != nil {
		return err
	}

	for key, value := range headers {
		w.Header()[key] = value
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	if _, err := buf.WriteTo(w); err != nil {
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
		return
	}
	if errors.Is(err, context.Canceled) {
		api.errorResponse(w, r, http.StatusRequestTimeout, "request was canceled")
		return
	}
	if errors.Is(err, context.DeadlineExceeded) {
		api.errorResponse(w, r, http.StatusRequestTimeout, "request timed out")
		return
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

func parseGPSRadiuses(query url.Values, pointsCount int) ([]float64, error) {
	const defaultGPSRadiusM = 40.0

	rawValue := query.Get("gps_radiuses")
	ok := rawValue != ""

	if !ok {
		rawValue := query.Get("gps_radius") // one gps accuracy radius
		ok := rawValue != ""
		if ok {
			radiusM, err := strconv.ParseFloat(rawValue, 64)
			if err != nil || radiusM < 0 {
				return nil, errors.New("gps_radius must be a non-negative float value")
			}
			gpsRadiusesM := make([]float64, pointsCount)
			for i := range gpsRadiusesM {
				gpsRadiusesM[i] = radiusM
			}
			return gpsRadiusesM, nil
		}

		gpsRadiusesM := make([]float64, pointsCount)
		for i := range gpsRadiusesM {
			gpsRadiusesM[i] = defaultGPSRadiusM
		}
		return gpsRadiusesM, nil
	}

	if rawValue == "" {
		return nil, errors.New("gps_radiuses must contain non-negative float values")
	}

	parts := strings.Split(rawValue, ";")
	if len(parts) != pointsCount {
		return nil, fmt.Errorf("gps_radiuses count must match GPX trackpoint count")
	}

	gpsRadiusesM := make([]float64, len(parts))
	for i, part := range parts {
		if part == "" {
			return nil, errors.New("gps_radiuses must contain non-negative float values")
		}
		radiusM, err := strconv.ParseFloat(part, 64)
		if err != nil || radiusM < 0 {
			return nil, errors.New("gps_radiuses must contain non-negative float values")
		}
		gpsRadiusesM[i] = radiusM
	}

	return gpsRadiusesM, nil
}
