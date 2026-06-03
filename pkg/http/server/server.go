// Package http_server implements the HTTP server for the routing engine.
package http_server

import (
	"context"
	"fmt"
	"net"
	"net/http"

	"github.com/spf13/viper"
)

const TimeoutMessage = `{"error":"context deadline exceeded"}`

func New(ctx context.Context, h http.Handler, config Config) *http.Server {
	handler := http.TimeoutHandler(h, config.Timeout, fmt.Sprintf(`{"error": %q}`, TimeoutMessage))

	httpVersion := viper.GetString("server.http_version")

	port := config.Port
	protocol := &http.Protocols{}

	switch httpVersion {
	case "http1.1":
		protocol.SetHTTP1(true)
		protocol.SetHTTP2(false)
		protocol.SetUnencryptedHTTP2(false)
	case "http2":
		protocol.SetUnencryptedHTTP2(true)
		protocol.SetHTTP2(false)
		protocol.SetHTTP1(false)
	}

	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", port),
		Handler: handler,
		BaseContext: func(_ net.Listener) context.Context {
			return ctx
		},

		Protocols:         protocol,
		ReadTimeout:       viper.GetDuration("server.read_timeout"),
		WriteTimeout:      config.Timeout + viper.GetDuration("server.write_timeout"),
		IdleTimeout:       viper.GetDuration("server.idle_timeout"),
		ReadHeaderTimeout: viper.GetDuration("server.read_header_timeout"),
	}

	return server
}
