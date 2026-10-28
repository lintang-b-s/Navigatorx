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

// JSONEncoderPool represents jsonencoder pool.
type JSONEncoderPool struct {
	pool sync.Pool
}

type encoderWrapper struct {
	buf     *bytes.Buffer
	encoder json.Encoder
}

// NewJSONEncoderPool creates and returns jsonencoder pool.
func NewJSONEncoderPool() *JSONEncoderPool {
	return &JSONEncoderPool{
		pool: sync.Pool{
			New: func() any {
				buf := bytes.NewBuffer(make([]byte, 0, JSONBufPoolSize))
				return &encoderWrapper{
					buf:     buf,
					encoder: json.ConfigDefault.NewEncoder(buf),
				}
			},
		},
	}
}

var (
	jencPool = NewJSONEncoderPool()
)

// writeJSON marshals data structure to encoded JSON response.
func (api *RoutingAPI) writeJSON(w http.ResponseWriter, status int, data any) error {
	jenc := jencPool.pool.Get().(*encoderWrapper)
	defer func() {
		jenc.buf.Reset()
		jencPool.pool.Put(jenc)
	}()

	if err := jenc.encoder.Encode(data); err != nil {
		return err
	}

	b := jenc.buf.Bytes()
	if len(b) > 0 && b[len(b)-1] == '\n' {
		b = b[:len(b)-1]
	}
	result := make([]byte, len(b))
	copy(result, b)

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	if _, err := w.Write(result); err != nil {
		api.log.Error("failed to write JSON response", zap.Error(err))
		return err
	}

	return nil
}

type messageEnvelope struct {
	Data MessageResponse `json:"data"`
}

// MessageResponse contains a message returned by the HTTP API.
type MessageResponse struct {
	Message string `json:"message"`
}

// NewMessageResponse creates and returns message response.
func NewMessageResponse(msg string) MessageResponse {
	return MessageResponse{msg}
}

func (api *RoutingAPI) getStatusCode(w http.ResponseWriter, r *http.Request, err error) {
	if err == nil {
		if err := api.writeJSON(w, http.StatusOK, messageEnvelope{Data: NewMessageResponse("success")}); err != nil {
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
	if ierr, ok := errors.AsType[*util.Error](err); !ok {
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
	const defaultGPSRadiusM = 5.0

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

func getRawQueryValue(rawQuery, key string) (string, bool, error) {
	for part := range strings.SplitSeq(rawQuery, "&") {
		if part == "" {
			continue
		}
		rawKey, rawValue, _ := strings.Cut(part, "=")
		decodedKey, err := url.QueryUnescape(rawKey)
		if err != nil {
			return "", false, err
		}
		if decodedKey != key {
			continue
		}
		decodedValue, err := url.QueryUnescape(rawValue)
		if err != nil {
			return "", false, err
		}
		return decodedValue, true, nil
	}

	return "", false, nil
}
