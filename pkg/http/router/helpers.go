// Package router defines the HTTP router and associated handlers.
package router

import (
	"bytes"
	"net/http"
	"sync"

	json "github.com/bytedance/sonic"
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

const (
	jsonBufPoolSize = 1 << 10
)

// NewJSONEncoderPool creates and returns jsonencoder pool.
func NewJSONEncoderPool() *JSONEncoderPool {
	return &JSONEncoderPool{
		pool: sync.Pool{
			New: func() any {
				buf := bytes.NewBuffer(make([]byte, 0, jsonBufPoolSize))
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
func (api *API) writeJSON(w http.ResponseWriter, status int, data any) error {
	jenc := jencPool.pool.Get().(*encoderWrapper)
	defer func(jenc *encoderWrapper) {
		jenc.buf.Reset()
		jencPool.pool.Put(jenc)
	}(jenc)

	if err := jenc.encoder.Encode(data); err != nil {
		return err
	}

	b := jenc.buf.Bytes()
	if len(b) > 0 && b[len(b)-1] == '\n' {
		b = b[:len(b)-1]
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	if _, err := w.Write(b); err != nil {
		api.log.Error("failed to write JSON response", zap.Error(err))
		return err
	}

	return nil
}
