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

func New(ctx context.Context, h http.Handler, config Config, websocket bool) *http.Server {

	handler := http.TimeoutHandler(h, config.Timeout, fmt.Sprintf(`{"error": %q}`, TimeoutMessage))

	port := config.Port
	if websocket {
		port = config.WebsocketPort
	}
	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", port),
		Handler: handler,
		BaseContext: func(_ net.Listener) context.Context {
			return ctx
		},

		ReadTimeout:       viper.GetDuration("server.read_timeout"),
		WriteTimeout:      config.Timeout + viper.GetDuration("server.write_timeout"),
		IdleTimeout:       viper.GetDuration("server.idle_timeout"),
		ReadHeaderTimeout: viper.GetDuration("server.read_header_timeout"),
	}

	return server
}

func NewWithoutSet(ctx context.Context, h http.Handler, config Config, websocket bool) *http.Server {
	handler := http.TimeoutHandler(h, config.Timeout, fmt.Sprintf(`{"error": %q}`, TimeoutMessage))

	port := config.Port
	if websocket {
		port = config.WebsocketPort
	}
	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", port),
		Handler: handler,
		BaseContext: func(_ net.Listener) context.Context {
			return ctx
		},

		ReadTimeout:       viper.GetDuration("server.read_timeout"),
		WriteTimeout:      config.Timeout + viper.GetDuration("server.write_timeout"),
		IdleTimeout:       viper.GetDuration("server.idle_timeout"),
		ReadHeaderTimeout: viper.GetDuration("server.read_header_timeout"),
	}
	return server
}
