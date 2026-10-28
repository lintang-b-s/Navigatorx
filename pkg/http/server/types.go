package http_server

import "time"

// Config holds configuration parameters for the HTTP server.
type Config struct {
	Port    int
	Timeout time.Duration
}

// API represents the HTTP API router component.
type API struct {
}
