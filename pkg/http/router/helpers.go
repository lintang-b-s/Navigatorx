// Package router defines the HTTP router and associated handlers.
package router

import (
	"bytes"
	"net/http"
	"sync"

	json "github.com/bytedance/sonic"
	"go.uber.org/zap"
)

var bufPool = sync.Pool{
	New: func() any {
		return bytes.NewBuffer(make([]byte, 0, 32<<10))
	},
}

// writeJSON marshals data structure to encoded JSON response.
func (api *API) writeJSON(w http.ResponseWriter, status int, data any) error {
	buf := bufPool.Get().(*bytes.Buffer)
	defer func() {
		buf.Reset()
		bufPool.Put(buf)
	}()

	enc := json.ConfigDefault.NewEncoder(buf)
	if err := enc.Encode(data); err != nil {
		return err
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	if _, err := buf.WriteTo(w); err != nil {
		api.log.Error("failed to write JSON response", zap.Error(err))
		return err
	}

	return nil
}
