package router

import (
	"bufio"
	"context"
	"fmt"
	"io"
	"log"
	"net"
	"net/http"
	"os"
	"os/signal"
	"syscall"
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
			RealIP, Heartbeat("healthz"), Logger(api.log), Labels, Limit)
	} else {
		mwChain = append(mwChain, corsHandler.Handler, EnforceJSONHandler, api.recoverPanic,
			RealIP, Heartbeat("healthz"), Logger(api.log), Labels)
	}
	mainMwChain := alice.New(mwChain...).Then(wsRouter)
	srv := http_server.New(ctx, mainMwChain, config, true)
	ln, err := net.Listen("tcp", srv.Addr)
	if err != nil {
		errChan <- err
	}
	api.log.Info(fmt.Sprintf("online map-matcher websocket API run on port %d", config.WebsocketPort))

	acceptDesc := netpoll.Must(netpoll.HandleListener(
		ln, netpoll.EventRead|netpoll.EventOneShot,
	))

	api.poller, err = netpoll.New(nil)
	if err != nil {
		errChan <- err
	}

	api.pool = concurrent.NewWorkerPool[int, int](15, 10)

	api.hub = controllers.NewHub(api.pool, mapMatcherService)

	api.pool.Spawn(10)
	// accept is a channel to signal about next incoming connection Accept()
	// results.
	accept := make(chan error, 1)

	api.poller.Start(acceptDesc, func(conn netpoll.Event) {
		/*
			add net listener (stream socket) file descriptor desc to epoll interest list. (netpoll run epoll_wait() in the background)

			The epoll_ctl() system call modifies the interest list of the epoll instance referred to
			by the file descriptor epfd.

			The epoll_wait() system call returns information about ready file descriptors from
			the epoll instance referred to by the file descriptor epfd. A single epoll_wait() call can
			return information about multiple ready file descriptors.
		*/
		defer api.poller.Resume(acceptDesc)
		err := api.pool.ScheduleTimeout(1000*time.Millisecond, func() {
			conn, err := ln.Accept()
			if err != nil {
				accept <- err
				return
			}

			accept <- nil
			api.handle(conn)
			return
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
				api.log.Sugar().Infof("accept error: %v; retrying in %s", err, delay)
				time.Sleep(delay)
			} else if ne, ok := err.(net.Error); ok && ne.Timeout() {
				delay := 5 * time.Millisecond
				api.log.Sugar().Infof("accept error: %v; retrying in %s", err, delay)
				time.Sleep(delay)
			} else {
				api.log.Sugar().Fatalf("accept error: %v", err)
			}
		}

	})

	// Handle graceful shutdown
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM, syscall.SIGKILL)

	<-sig

	ln.Close()

	api.hub.RemoveAllUser()
	api.poller.Stop(acceptDesc)

	api.pool.Close()

	log.Println("webocket server stopped")

	return
}

/*
handle. handle online map matching request
use epoll api to reduce memory stack, ref: https://sergey.kamardin.org/articles/million-websocket-and-go/

the linux programming interface chapter 63:
the epoll API allows a process to monitor multiple
file descriptors to see if I/O is possible on any of them. Like signal-driven I/O,
the epoll API provides much better performance when monitoring large num-
bers of file descriptors.
*/

func (api *API) handle(conn net.Conn) {

	br := bufio.NewReader(conn)

	rw := struct {
		io.Reader
		io.Writer
	}{br, conn}

	hs, err := ws.Upgrade(rw)
	if err != nil {
		api.log.Info("upgrade error", zap.Error(err), zap.String("connnection name ", nameConn(conn)))
		conn.Close()
		return
	}

	api.log.Info("established websocket connection", zap.String("connnection name ", nameConn(conn)),
		zap.String("protocol", hs.Protocol))

	user := api.hub.Register(conn)

	desc := netpoll.Must(netpoll.HandleRead(conn))

	api.poller.Start(desc, func(ev netpoll.Event) {
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
			api.log.Error("user disconnected from websocket server")

			api.poller.Stop(desc)
			api.hub.Remove(user)
			return
		}

		// spawn goroutine from goroutine pool to handle the request
		api.pool.Schedule(func() {
			// run online map matching & send the map matching result to user
			err := user.OnlineMapMatch()
			if err != nil {
				api.log.Error("error online map matching", zap.Error(err))
				// error -> remove user conn file descriptor from epoll interest list & remove from hub
				api.poller.Stop(desc)
				api.hub.Remove(user)
			}
			return
		})

	})
}

func nameConn(conn net.Conn) string {
	return conn.LocalAddr().String() + " > " + conn.RemoteAddr().String()
}
