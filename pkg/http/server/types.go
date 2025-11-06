package http_server

import "time"

type Config struct {
	Port          int
	WebsocketPort int
	ProxyPort     int
	Timeout       time.Duration
}

type API struct {
}
