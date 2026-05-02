package router

import (
	"context"
	"fmt"
	"net"
	"os"
	"strings"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	http_server "github.com/lintang-b-s/Navigatorx/pkg/http/server"
	"github.com/mailru/easygo/netpoll"
	"github.com/stretchr/testify/assert"
	"go.uber.org/zap"
)

type pipeAddr string

func (a pipeAddr) Network() string { return "pipe" }
func (a pipeAddr) String() string  { return string(a) }

type filePipeConn struct {
	r *os.File
	w *os.File
}

func newFilePipeConnPair(t *testing.T) (net.Conn, net.Conn) {
	t.Helper()
	serverRead, clientWrite, err := os.Pipe()
	assert.NoError(t, err)
	clientRead, serverWrite, err := os.Pipe()
	assert.NoError(t, err)
	return &filePipeConn{r: serverRead, w: serverWrite}, &filePipeConn{r: clientRead, w: clientWrite}
}

func (c *filePipeConn) Read(b []byte) (int, error)       { return c.r.Read(b) }
func (c *filePipeConn) Write(b []byte) (int, error)      { return c.w.Write(b) }
func (c *filePipeConn) LocalAddr() net.Addr              { return pipeAddr("local") }
func (c *filePipeConn) RemoteAddr() net.Addr             { return pipeAddr("remote") }
func (c *filePipeConn) SetDeadline(time.Time) error      { return nil }
func (c *filePipeConn) SetReadDeadline(time.Time) error  { return nil }
func (c *filePipeConn) SetWriteDeadline(time.Time) error { return nil }
func (c *filePipeConn) File() (*os.File, error)          { return c.r, nil }
func (c *filePipeConn) Close() error                     { _ = c.r.Close(); return c.w.Close() }

type callbackPoller struct {
	event netpoll.Event
}

func (p callbackPoller) Start(_ *netpoll.Desc, cb netpoll.CallbackFn) error {
	cb(p.event)
	return nil
}

func (p callbackPoller) Stop(_ *netpoll.Desc) error   { return nil }
func (p callbackPoller) Resume(_ *netpoll.Desc) error { return nil }

type fakeWebsocketListener struct{}

func (fakeWebsocketListener) Accept() (net.Conn, error) {
	return nil, fmt.Errorf("accept error")
}

func (fakeWebsocketListener) Close() error {
	return nil
}

func (fakeWebsocketListener) Addr() net.Addr {
	return pipeAddr("listener")
}

func skipSocketPermission(t *testing.T, err error) {
	t.Helper()
	if err != nil && strings.Contains(err.Error(), "operation not permitted") {
		t.Skipf("local sockets are not permitted in this sandbox: %v", err)
	}
}

func TestAPI_Handle(t *testing.T) {
	log := zap.NewNop()
	api := NewAPI(log)

	var err error
	api.poller, err = netpoll.New(nil)
	assert.NoError(t, err)
	api.pool = concurrent.NewWorkerPool[int, int](1, 1)
	api.hub = controllers.NewHub(api.pool, nil, api.log)

	t.Run("Upgrade Success", func(t *testing.T) {
		ln, err := net.Listen("tcp", ":0")
		skipSocketPermission(t, err)
		if err != nil {
			t.Fatal(err)
		}
		defer ln.Close()

		done := make(chan struct{})
		go func() {
			c2, err := net.Dial("tcp", ln.Addr().String())
			if err != nil {
				return
			}
			defer c2.Close()
			// Send websocket upgrade request on c2
			req := "GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n"
			_, _ = c2.Write([]byte(req))
			// Read upgrade response
			buf := make([]byte, 1024)
			_, _ = c2.Read(buf)
			close(done)
		}()

		c1, err := ln.Accept()
		if err != nil {
			t.Fatal(err)
		}
		defer c1.Close()

		err = api.handle(c1)
		assert.NoError(t, err)
		<-done
	})

	t.Run("Handle OnlineMapMatch Error", func(t *testing.T) {
		ln, err := net.Listen("tcp", ":0")
		skipSocketPermission(t, err)
		if err != nil {
			t.Fatal(err)
		}
		defer ln.Close()

		go func() {
			c2, err := net.Dial("tcp", ln.Addr().String())
			if err != nil {
				return
			}
			// Send websocket upgrade request
			req := "GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n"
			_, _ = c2.Write([]byte(req))
			// Read upgrade response
			buf := make([]byte, 1024)
			_, _ = c2.Read(buf)
			// Close c2 to trigger read error in OnlineMapMatch
			c2.Close()
		}()

		c1, err := ln.Accept()
		if err != nil {
			t.Fatal(err)
		}
		defer c1.Close()

		err = api.handle(c1)
		assert.NoError(t, err)
		// Give some time for netpoll to trigger
		time.Sleep(200 * time.Millisecond)
	})

	t.Run("Upgrade Failure", func(t *testing.T) {
		ln, err := net.Listen("tcp", ":0")
		skipSocketPermission(t, err)
		if err != nil {
			t.Fatal(err)
		}
		defer ln.Close()

		go func() {
			c2, err := net.Dial("tcp", ln.Addr().String())
			if err != nil {
				return
			}
			defer c2.Close()
			_, _ = c2.Write([]byte("INVALID REQUEST"))
		}()

		c1, err := ln.Accept()
		if err != nil {
			t.Fatal(err)
		}
		defer c1.Close()

		err = api.handle(c1)
		assert.Error(t, err)
	})
}

func TestAPI_Handle_NetPipe(t *testing.T) {
	api := NewAPI(zap.NewNop())
	api.pool = concurrent.NewWorkerPool[int, int](1, 1)
	api.hub = controllers.NewHub(api.pool, nil, api.log)

	t.Run("Upgrade Failure", func(t *testing.T) {
		serverConn, clientConn := net.Pipe()
		defer clientConn.Close()
		_ = serverConn.SetDeadline(time.Now().Add(200 * time.Millisecond))
		_ = clientConn.SetDeadline(time.Now().Add(200 * time.Millisecond))

		go func() {
			_, _ = clientConn.Write([]byte("INVALID REQUEST"))
		}()

		err := api.handle(serverConn)
		assert.Error(t, err)
	})

	t.Run("Upgrade Success And Disconnect Event", func(t *testing.T) {
		serverConn, clientConn := newFilePipeConnPair(t)
		defer clientConn.Close()
		api.poller = callbackPoller{event: netpoll.EventReadHup}

		done := make(chan struct{})
		go func() {
			defer close(done)
			req := "GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n"
			_, _ = clientConn.Write([]byte(req))
			buf := make([]byte, 1024)
			_, _ = clientConn.Read(buf)
		}()

		err := api.handle(serverConn)
		assert.NoError(t, err)
		<-done
	})

}

func TestAPI_HandleWebsocket(t *testing.T) {
	log := zap.NewNop()
	api := NewAPI(log)

	t.Run("Success", func(t *testing.T) {
		ctx, cancel := context.WithCancel(context.Background())
		config := http_server.Config{
			WebsocketPort: 0,
		}

		errChan := make(chan error, 1)
		go func() {
			api.handleWebsocket(ctx, config, nil, false, errChan)
		}()

		time.Sleep(100 * time.Millisecond)
		cancel()

		select {
		case err := <-errChan:
			skipSocketPermission(t, err)
			assert.NoError(t, err)
		case <-time.After(2 * time.Second):
		}
	})

	t.Run("Listen Error", func(t *testing.T) {
		ln, err := net.Listen("tcp", ":0")
		skipSocketPermission(t, err)
		if err != nil {
			t.Fatal(err)
		}
		defer ln.Close()

		_, portStr, _ := net.SplitHostPort(ln.Addr().String())
		var port int
		_, _ = fmt.Sscanf(portStr, "%d", &port)

		configErr := http_server.Config{
			WebsocketPort: port,
		}

		errChan := make(chan error, 1)
		api.handleWebsocket(context.Background(), configErr, nil, false, errChan)
		err = <-errChan
		assert.Error(t, err)
	})
}

func TestAPI_HandleWebsocket_WithInjectedListener(t *testing.T) {
	api := NewAPI(zap.NewNop())
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	origListen := listenWebsocket
	origHandleListener := handleListener
	origNewPoller := newPoller
	listenWebsocket = func(network, addr string) (net.Listener, error) {
		return fakeWebsocketListener{}, nil
	}
	handleListener = func(net.Listener, netpoll.Event) (*netpoll.Desc, error) {
		return netpoll.NewDesc(0, netpoll.EventRead), nil
	}
	newPoller = func(*netpoll.Config) (netpoll.Poller, error) {
		return callbackPoller{event: netpoll.EventRead}, nil
	}
	t.Cleanup(func() {
		listenWebsocket = origListen
		handleListener = origHandleListener
		newPoller = origNewPoller
	})

	errChan := make(chan error, 1)
	done := make(chan struct{})
	go func() {
		defer close(done)
		api.handleWebsocket(ctx, http_server.Config{WebsocketPort: 9090}, nil, true, errChan)
	}()

	time.Sleep(20 * time.Millisecond)
	cancel()

	select {
	case err := <-errChan:
		assert.NoError(t, err)
	case <-done:
	case <-time.After(time.Second):
		t.Fatal("timed out waiting for injected websocket server")
	}
}
