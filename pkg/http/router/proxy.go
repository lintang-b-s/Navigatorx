package router

import (
	"io"
	"net"
	"net/http"

	"go.uber.org/zap"
)

func (api *API) upstream(name, network, addr string) func(w http.ResponseWriter, r *http.Request) {

	return func(w http.ResponseWriter, r *http.Request) {

		peer, err := net.Dial(network, addr)
		if err != nil {
			api.log.Error("dial upstream error:", zap.Error(err))
			w.WriteHeader(502)
			return
		}
		if err := r.Write(peer); err != nil {
			api.log.Error("write request to upstream error: %v", zap.Error(err))
			w.WriteHeader(502)
			return
		}
		hj, ok := w.(http.Hijacker)
		if !ok {
			w.WriteHeader(500)
			return
		}
		conn, _, err := hj.Hijack() // get tcp socket
		if err != nil {
			w.WriteHeader(500)
			return
		}

		go func() {
			defer peer.Close()
			defer conn.Close()
			io.Copy(peer, conn)
		}()
		go func() {
			defer peer.Close()
			defer conn.Close()
			io.Copy(conn, peer)
		}()
	}
}
