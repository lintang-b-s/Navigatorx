package router

import (
	"context"
	"errors"
	"fmt"
	"net"
	"net/http"
	"strings"
	"time"

	"github.com/gobwas/ws"
	"github.com/julienschmidt/httprouter"
	"github.com/justinas/alice"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	http_server "github.com/lintang-b-s/Navigatorx/pkg/http/server"
	"github.com/mailru/easygo/netpoll"
	"github.com/rs/cors"
	"go.uber.org/zap"
)

var (
	listenWebsocket = net.Listen
	handleListener  = netpoll.HandleListener
	newPoller       = netpoll.New
)

func (api *API) handleWebsocket(ctx context.Context, config http_server.Config,
	mapMatcherService controllers.MapMatcherService,
	useRateLimit bool, errChan chan error,
) {
	var err error

	wsRouter := httprouter.New()

	corsHandler := cors.New(cors.Options{ //nolint:gocritic // ignore
		AllowedOrigins:   []string{"*"},
		AllowedMethods:   []string{"GET", "POST", "DELETE", "OPTIONS"},
		AllowedHeaders:   []string{"Accept", "Authorization", "Content-Type", "X-CSRF-Token"},
		ExposedHeaders:   []string{"Link"},
		AllowCredentials: true,
		MaxAge:           300, //nolint:mnd // ignore

	})

	wsRouter.GET("/doc/*any", swaggerHandler)

	wsRouter.Handler(http.MethodGet, "/debug/pprof/*item", http.DefaultServeMux)

	var mwChain []alice.Constructor
	if useRateLimit {
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, api.Heartbeat("healthz"), Logger(api.log), Labels, api.Limit)
	} else {
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, api.Heartbeat("healthz"), Logger(api.log), Labels)
	}
	mainMwChain := alice.New(mwChain...).Then(wsRouter)
	srv := http_server.NewWithoutSet(ctx, mainMwChain, config, true)
	ln, err := listenWebsocket("tcp", srv.Addr)
	if err != nil {
		errChan <- err
		return
	}
	api.log.Info(fmt.Sprintf("online map-matcher websocket API run on port %d", config.WebsocketPort))

	// taken from: https://github.com/gobwas/ws-examples/tree/master/src

	// Create netpoll descriptor for the listener.
	// We use OneShot here to manually resume events stream when we want to.
	acceptDesc := netpoll.Must(handleListener(
		ln, netpoll.EventRead|netpoll.EventOneShot,
	))

	api.poller, err = newPoller(nil)
	if err != nil {
		errChan <- err
		return
	}

	api.pool = concurrent.NewWorkerPool[int, int](2, 15)

	api.hub = controllers.NewHub(api.pool, mapMatcherService, api.log)
	go api.hub.Start()

	api.pool.Spawn(1)
	// accept is a channel to signal about next incoming connection Accept()
	// results.
	accept := make(chan error, 1)

	err = api.poller.Start(acceptDesc, func(conn netpoll.Event) {
		/*
			add net listener (stream socket) file descriptor desc to epoll interest list. (netpoll run epoll_wait() in the background)

			The epoll_ctl() system call modifies the interest list of the epoll instance referred to
			by the file descriptor epfd.

			The epoll_wait() system call returns information about ready file descriptors from
			the epoll instance referred to by the file descriptor epfd. A single epoll_wait() call can
			return information about multiple ready file descriptors.
		*/
		defer func() {
			if err := api.poller.Resume(acceptDesc); err != nil {
				api.log.Sugar().Errorf("failed to resume poller: %v", err)
			}
		}()
		err := api.pool.ScheduleTimeout(1*time.Millisecond, func() {
			conn, err := ln.Accept()
			if err != nil {
				accept <- err
				return
			}

			accept <- api.handle(conn)
		})

		if err == nil {
			err = <-accept
		}

		if err != nil {
			/*
				if the goroutine pool is full for 1 ms and there are incoming connections,
				cooldown the server for 5 ms
			*/
			if err != concurrent.ErrScheduleTimeout {
				delay := 5 * time.Millisecond
				api.log.Sugar().Error("accept error: %v; retrying in %s", err, delay)
				time.Sleep(delay)
			} else if ne, ok := err.(net.Error); ok && ne.Timeout() {
				delay := 5 * time.Millisecond
				api.log.Sugar().Error("accept error: %v; retrying in %s", err, delay)
				time.Sleep(delay)
			} else {
				api.log.Sugar().Errorf("accept error: %v", err)
			}
		}
	})
	if err != nil {
		errChan <- err
		return
	}

	// Handle graceful shutdown
	<-ctx.Done()

	if err := ln.Close(); err != nil {
		api.log.Sugar().Errorf("failed to close listener: %v", err)
	}

	api.hub.RemoveAllUser()
	api.hub.Stop()
	if err := api.poller.Stop(acceptDesc); err != nil {
		api.log.Sugar().Errorf("failed to stop poller: %v", err)
	}

	api.pool.Close()

	api.log.Info("webocket server stopped")
}

/*
handle is a new incoming connection handler.
It upgrades TCP connection to WebSocket, registers netpoll listener on
it and stores it as a map matching user in Hub instance.

handle. handle online map matching request
ref: https://sergey.kamardin.org/articles/million-websocket-and-go/

NOTE: jujur aku masih gakpaham epoll epoll API, asal copas doang aowkaokw..
tapi ini works buat demo..
mungkin kalau ada waktu bakal belajar linux system programming buat pahamin ini semua wkwk...

the linux programming interface chapter 63:
the epoll API allows a process to monitor multiple
file descriptors to see if I/O is possible on any of them. Like signal-driven I/O,
the epoll API provides much better performance when monitoring large num-
bers of file descriptors.
*/
type deadliner struct {
	net.Conn
	t time.Duration
}

const (
	ioTimeout = time.Millisecond * 100
)

func (api *API) handle(conn net.Conn) error {

	safeConn := deadliner{conn, ioTimeout}

	// Capture client IP and User-Agent during the WebSocket upgrade handshake.
	// gobwas/ws OnHeader provides zero-copy access to HTTP headers before they are discarded.
	var clientIP, userAgent string
	u := ws.Upgrader{
		OnHeader: func(key, value []byte) error {
			k := string(key)
			switch {
			case strings.EqualFold(k, "X-Real-IP"):
				clientIP = string(value)
			case strings.EqualFold(k, "User-Agent"):
				userAgent = string(value)
			}
			return nil
		},
	}

	// Zero-copy upgrade to WebSocket connection.
	hs, err := u.Upgrade(safeConn)

	if err != nil {
		api.log.Error("upgrade error", zap.Error(err), zap.String("connnection name ", nameConn(conn)))
		conn.Close()
		return err
	}

	// Fallback: if X-Real-IP was not set (e.g. local dev without Caddy), use RemoteAddr
	if clientIP == "" {
		clientIP = conn.RemoteAddr().String()
	}

	api.log.Info("established websocket connection", zap.String("connnection name ", nameConn(conn)),
		zap.String("protocol", hs.Protocol), zap.String("client_ip", clientIP),
		zap.String("user_agent", userAgent))

	user := api.hub.Register(conn, clientIP, userAgent)

	desc, err := netpoll.HandleRead(conn)
	if err != nil {
		return err
	}

	err = api.poller.Start(desc, func(ev netpoll.Event) {
		/*
			add user connection (request) file descriptor desc to epoll interest list. (netpoll run epoll_wait() in the background)

			The epoll_ctl() system call modifies the interest list of the epoll instance referred to
			by the file descriptor epfd.

			The epoll_wait() system call returns information about ready file descriptors from
			the epoll instance referred to by the file descriptor epfd. A single epoll_wait() call can
			return information about multiple ready file descriptors.
		*/
		if ev&(netpoll.EventReadHup|netpoll.EventHup) != 0 {
			/*
				Hang up happened on the associated file descriptor.

				Note that when reading from a channel such as a pipe or a
				stream socket, this event merely indicates that the peer
				closed its end of the channel.  Subsequent reads from the
				channel will return 0 (end of file) only after all
				outstanding data in the channel has been consumed.
			*/

			if err := api.poller.Stop(desc); err != nil {
				api.log.Sugar().Errorf("failed to stop poller: %v", err)
			}
			api.hub.Remove(user)
			return
		}

		// spawn goroutine from goroutine pool to handle the request
		err := api.pool.Schedule(func() {
			// run online map matching & send the map matching result to user
			err := user.OnlineMapMatch()
			if err != nil {
				if errors.Is(err, controllers.ErrNormalClosure) {
					// Normal closure: user stopped navigation (ws.close(1000))
				} else {
					api.log.Error("error online map matching", zap.Error(err))
				}
				// In both cases: remove user conn file descriptor from epoll interest list & remove from hub
				if err := api.poller.Stop(desc); err != nil {
					api.log.Sugar().Errorf("failed to stop poller: %v", err)
				}
				api.hub.Remove(user)
			}
		})
		if err != nil {
			api.log.Error("error scheduling job", zap.Error(err))
			// error -> remove user conn file descriptor from epoll interest list & remove from hub
			if err := api.poller.Stop(desc); err != nil {
				api.log.Sugar().Errorf("failed to stop poller: %v", err)
			}
			api.hub.Remove(user)
		}
	})
	if err != nil {
		return err
	}

	return nil
}

func nameConn(conn net.Conn) string {
	return conn.LocalAddr().String() + " > " + conn.RemoteAddr().String()
}
