// Package router defines the HTTP router and associated handlers.
package router

import (
	"net/http"

	json "github.com/bytedance/sonic"
	"go.uber.org/zap"
)

// writeJSON marshals data structure to encoded JSON response.
func (api *API) writeJSON(w http.ResponseWriter, status int, data any,
	headers http.Header) error {
	js, err := json.Marshal(data)
	if err != nil {
		return err
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	if _, err := w.Write(js); err != nil {
		api.log.Error("failed to write JSON response", zap.Error(err))
		return err
	}

	return nil
}
