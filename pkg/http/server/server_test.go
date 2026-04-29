package http_server

import (
	"context"
	"net/http"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
)

func TestNew(t *testing.T) {
	ctx := context.Background()
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {})
	config := Config{
		Port:    8080,
		Timeout: 5 * time.Second,
	}

	server := New(ctx, handler, config, false)

	assert.NotNil(t, server)
	assert.Equal(t, ":8080", server.Addr)
	assert.NotNil(t, server.Handler)
	assert.Equal(t, ctx, server.BaseContext(nil))
}

func TestNew_Websocket(t *testing.T) {
	ctx := context.Background()
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {})
	config := Config{
		WebsocketPort: 8081,
		Timeout:       5 * time.Second,
	}

	server := New(ctx, handler, config, true)

	assert.NotNil(t, server)
	assert.Equal(t, ":8081", server.Addr)
	assert.Equal(t, ctx, server.BaseContext(nil))
}

func TestNewWithoutSet(t *testing.T) {
	ctx := context.Background()
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {})
	config := Config{
		Port:    8082,
		Timeout: 5 * time.Second,
	}

	server := NewWithoutSet(ctx, handler, config, false)

	assert.NotNil(t, server)
	assert.Equal(t, ":8082", server.Addr)
	assert.Equal(t, ctx, server.BaseContext(nil))
}

func TestNewWithoutSet_Websocket(t *testing.T) {
	ctx := context.Background()
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {})
	config := Config{
		WebsocketPort: 8083,
		Timeout:       5 * time.Second,
	}

	server := NewWithoutSet(ctx, handler, config, true)

	assert.NotNil(t, server)
	assert.Equal(t, ":8083", server.Addr)
	assert.Equal(t, ctx, server.BaseContext(nil))
}
