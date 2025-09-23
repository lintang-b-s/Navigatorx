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
	viper.SetDefault("HTTP_SERVER_READ_TIMEOUT", "10s")
	viper.SetDefault("HTTP_SERVER_WRITE_TIMEOUT", "10s")
	viper.SetDefault("HTTP_SERVER_IDLE_TIMEOUT", "30s")
	viper.SetDefault("HTTP_SERVER_READ_HEADER_TIMEOUT", "2s")

	handler := http.TimeoutHandler(h, config.Timeout, fmt.Sprintf(`{"error": %q}`, TimeoutMessage))

	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.Port),
		Handler: handler,
		BaseContext: func(_ net.Listener) context.Context {
			return ctx
		},

		ReadTimeout:       viper.GetDuration("HTTP_SERVER_READ_TIMEOUT"),
		WriteTimeout:      config.Timeout + viper.GetDuration("HTTP_SERVER_WRITE_TIMEOUT"),
		IdleTimeout:       viper.GetDuration("HTTP_SERVER_IDLE_TIMEOUT"),
		ReadHeaderTimeout: viper.GetDuration("HTTP_SERVER_READ_HEADER_TIMEOUT"),
	}

	return server
}
