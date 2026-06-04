package http_server

import (
	"context"
	"net/http"
	"testing"
	"time"

	"github.com/spf13/viper"
	"github.com/stretchr/testify/assert"
)

func TestNew(t *testing.T) {
	viper.Set("server.write_timeout", 0)
	t.Cleanup(func() {
		viper.Reset()
	})

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
	assert.Equal(t, 5*time.Second, server.WriteTimeout)
}
