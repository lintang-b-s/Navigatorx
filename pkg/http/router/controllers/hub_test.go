package controllers

import (
	"context"
	"errors"
	"net"
	"testing"
	"time"

	json "github.com/bytedance/sonic"
	"github.com/gobwas/ws/wsutil"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/stretchr/testify/assert"
)

func TestHub(t *testing.T) {
	mockMMS := new(MockMapMatcherService)
	hub := NewHub(nil, mockMMS)

	t.Run("Register and Remove", func(t *testing.T) {
		c1, _ := net.Pipe()
		user := hub.Register(c1)
		assert.Equal(t, 1, len(hub.us))
		assert.Equal(t, user, hub.ns[user.id])

		hub.Remove(user)
		assert.Equal(t, 0, len(hub.us))
		_, exists := hub.ns[user.id]
		assert.False(t, exists)
	})

	t.Run("Remove Non-existent User", func(t *testing.T) {
		user := &User{id: 999}
		hub.Remove(user) // Should not panic
	})

	t.Run("RemoveAllUser", func(t *testing.T) {
		c1, _ := net.Pipe()
		hub.Register(c1)
		c2, _ := net.Pipe()
		hub.Register(c2)
		assert.Equal(t, 2, len(hub.us))

		hub.RemoveAllUser()
		assert.Equal(t, 0, len(hub.us))
	})
}

func TestUser_OnlineMapMatch(t *testing.T) {
	mockMMS := new(MockMapMatcherService)
	hub := NewHub(nil, mockMMS)

	t.Run("Success", func(t *testing.T) {
		c1, c2 := net.Pipe()
		user := hub.Register(c1)
		requestTime := time.Date(2026, 4, 29, 10, 0, 0, 0, time.UTC)

		request := mapMatchRequest{
			Gps: gps{
				Lat:  yogyakartaOriginLat,
				Lon:  yogyakartaOriginLon,
				Time: requestTime,
			},
			K: 1,
		}

		go func() {
			body, _ := json.Marshal(request)
			_ = wsutil.WriteClientText(c2, body)
		}()

		dummyGPS := da.NewGPSPoint(yogyakartaOriginLat, yogyakartaOriginLon, requestTime, 0, 0, false)
		dummyMatched := da.NewMatchedGPSPoint(dummyGPS, 1, da.Coordinate{Lat: yogyakartaOriginLat, Lon: yogyakartaOriginLon}, 0)
		mockMMS.On("OnlineMapMatch", context.Background(), dummyGPS, 1, []*ma.Candidate{}, 0.0, 0.0, 0.0).
			Return(dummyMatched, []*ma.Candidate{}, 0.0, 0.0, nil)

		respChan := make(chan []byte, 1)
		errChan := make(chan error, 1)
		go func() {
			msg, _, err := wsutil.ReadServerData(c2)
			if err != nil {
				errChan <- err
				return
			}
			respChan <- msg
		}()

		err := user.OnlineMapMatch()
		assert.NoError(t, err)

		select {
		case msg := <-respChan:
			assert.Contains(t, string(msg), "data")
		case err := <-errChan:
			t.Fatalf("failed to read response: %v", err)
		case <-time.After(2 * time.Second):
			t.Fatal("timed out waiting for response")
		}
	})

	t.Run("Invalid JSON", func(t *testing.T) {
		c1, c2 := net.Pipe()
		user := hub.Register(c1)

		go func() {
			_ = wsutil.WriteClientText(c2, []byte("invalid"))
		}()

		err := user.OnlineMapMatch()
		assert.Error(t, err)
	})

	t.Run("Validation Error", func(t *testing.T) {
		c1, c2 := net.Pipe()
		user := hub.Register(c1)

		request := mapMatchRequest{
			Gps: gps{
				Lat: 100.0, // Invalid
			},
		}

		go func() {
			body, _ := json.Marshal(request)
			_ = wsutil.WriteClientText(c2, body)
		}()

		respChan := make(chan []byte, 1)
		go func() {
			msg, _, _ := wsutil.ReadServerData(c2)
			respChan <- msg
		}()

		err := user.OnlineMapMatch()
		assert.NoError(t, err) // write() returns nil if successful

		select {
		case msg := <-respChan:
			assert.Contains(t, string(msg), "validation error")
		case <-time.After(2 * time.Second):
			t.Fatal("timed out waiting for response")
		}
	})

	t.Run("Service Error", func(t *testing.T) {
		c1, c2 := net.Pipe()
		user := hub.Register(c1)
		requestTime := time.Date(2026, 4, 29, 10, 0, 0, 0, time.UTC)

		request := mapMatchRequest{
			Gps: gps{
				Lat:  yogyakartaOriginLat,
				Lon:  yogyakartaOriginLon,
				Time: requestTime,
			},
			K: 1,
		}

		go func() {
			body, _ := json.Marshal(request)
			_ = wsutil.WriteClientText(c2, body)
		}()

		mockMMS.ExpectedCalls = nil
		gpsPoint := da.NewGPSPoint(yogyakartaOriginLat, yogyakartaOriginLon, requestTime, 0, 0, false)
		mockMMS.On("OnlineMapMatch", context.Background(), gpsPoint, 1, []*ma.Candidate{}, 0.0, 0.0, 0.0).
			Return(nil, nil, 0.0, 0.0, errors.New("service error"))

		respChan := make(chan []byte, 1)
		go func() {
			msg, _, _ := wsutil.ReadServerData(c2)
			respChan <- msg
		}()

		err := user.OnlineMapMatch()
		assert.NoError(t, err)

		select {
		case msg := <-respChan:
			assert.Contains(t, string(msg), "service error")
		case <-time.After(2 * time.Second):
			t.Fatal("timed out waiting for response")
		}
	})

	t.Run("Write Error", func(t *testing.T) {
		c1, _ := net.Pipe()
		user := hub.Register(c1)
		c1.Close()

		err := user.write("test")
		assert.Error(t, err)
	})
}
