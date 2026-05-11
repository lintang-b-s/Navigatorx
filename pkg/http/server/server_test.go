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

	server := New(ctx, handler, config)

	assert.NotNil(t, server)
	assert.Equal(t, ":8080", server.Addr)
	assert.NotNil(t, server.Handler)
	assert.Equal(t, ctx, server.BaseContext(nil))
}

func TestNewWithoutSet(t *testing.T) {
	ctx := context.Background()
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {})
	config := Config{
		Port:    8082,
		Timeout: 5 * time.Second,
	}

	server := NewWithoutSet(ctx, handler, config)

	assert.NotNil(t, server)
	assert.Equal(t, ":8082", server.Addr)
	assert.Equal(t, ctx, server.BaseContext(nil))
}
