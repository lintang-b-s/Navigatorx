package router

import (
	"fmt"
	"mime"
	"net"
	"net/http"
	"runtime/pprof"
	"strings"
	"sync"
	"time"

	"runtime/debug"

	"go.uber.org/zap"
	"golang.org/x/time/rate"
)

// recoverPanic is middleware that recovers from a panic by responding with a 500 Internal Server
// Error before closing the connection.
func (api *API) recoverPanic(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {

		defer func() {
			if err := recover(); err != nil {
				api.log.Error("panic recovered. err: ", zap.String("err", string(debug.Stack())))

				w.Header().Set("Connection:", "close")

				api.ServerErrorResponse(w, r, fmt.Errorf("%s", err))
			}
		}()
		next.ServeHTTP(w, r)
	})
}

var trueClientIP = http.CanonicalHeaderKey("True-Client-IP")
var xForwardedFor = http.CanonicalHeaderKey("X-Forwarded-For")
var xRealIP = http.CanonicalHeaderKey("X-Real-IP")

func RealIP(h http.Handler) http.Handler {
	fn := func(w http.ResponseWriter, r *http.Request) {
		if rip := realIP(r); rip != "" {
			r.RemoteAddr = rip
		}
		h.ServeHTTP(w, r)
	}

	return http.HandlerFunc(fn)
}

func realIP(r *http.Request) string {
	var ip string

	if tcip := r.Header.Get(trueClientIP); tcip != "" {
		ip = tcip
	} else if xrip := r.Header.Get(xRealIP); xrip != "" {
		ip = xrip
	} else if xff := r.Header.Get(xForwardedFor); xff != "" {
		i := strings.Index(xff, ",")
		if i == -1 {
			i = len(xff)
		}
		ip = xff[:i]
	}
	if ip == "" || net.ParseIP(ip) == nil {
		return ""
	}
	return ip
}

// heartbeat /healthz endpoint
func (api *API) Heartbeat(endpoint string) func(http.Handler) http.Handler {
	f := func(h http.Handler) http.Handler {
		fn := func(w http.ResponseWriter, r *http.Request) {
			if (r.Method == "GET" || r.Method == "HEAD") && strings.EqualFold(r.URL.Path, endpoint) {
				if isShuttingDown.Load() { // https://victoriametrics.com/blog/go-graceful-shutdown/
					http.Error(w, "Shutting down", http.StatusServiceUnavailable)
					return
				}
				w.Header().Set("Content-Type", "text/plain")
				w.WriteHeader(http.StatusOK)
				_, err := w.Write([]byte("."))
				if err != nil {
					api.log.Error("error writing heartbeat response", zap.Error(err))
				}
				return
			}
			h.ServeHTTP(w, r)
		}
		return http.HandlerFunc(fn)
	}
	return f
}

// EnforceJSONHandler make sure that the request has a Content-Type header of application/json
func EnforceJSONHandler(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		contentType := r.Header.Get("Content-Type")

		if contentType != "" {
			mt, _, err := mime.ParseMediaType(contentType)
			if err != nil {
				http.Error(w, "Malformed Content-Type header", http.StatusBadRequest)
				return
			}

			if mt != "application/json" {
				http.Error(w, "Content-Type header must be application/json", http.StatusUnsupportedMediaType)
				return
			}
		}

		next.ServeHTTP(w, r)
	})
}

// request logger
type responseWriter struct {
	http.ResponseWriter
	status      int
	wroteHeader bool
}

func wrapResponseWriter(w http.ResponseWriter) *responseWriter {
	return &responseWriter{ResponseWriter: w}
}

func (rw *responseWriter) Status() int {
	return rw.status
}

func (rw *responseWriter) WriteHeader(code int) {
	if rw.wroteHeader {
		return
	}

	rw.status = code
	rw.ResponseWriter.WriteHeader(code)
	rw.wroteHeader = true

}

type httprouterLogger struct {
	log *zap.Logger
}

func Logger(log *zap.Logger) func(next http.Handler) http.Handler {
	return httprouterLogger{log: log}.middleware
}

func (hl httprouterLogger) middleware(next http.Handler) http.Handler {
	fn := func(w http.ResponseWriter, r *http.Request) {
		start := time.Now()

		wrappped := wrapResponseWriter(w)
		defer func() {
			latency := time.Since(start)

			// Extract client IP: prefer X-Real-IP (set by Caddy), fallback to RemoteAddr
			clientIP := r.Header.Get("X-Real-IP")
			if clientIP == "" {
				clientIP = r.RemoteAddr
			}

			hl.log.Info("request completed", zap.String("status", http.StatusText(wrappped.status)),
				zap.Int64("took", latency.Milliseconds()), zap.String("remote", r.RemoteAddr),
				zap.String("client_ip", clientIP), zap.String("user_agent", r.UserAgent()),
				zap.String("request", r.RequestURI), zap.String("method", r.Method))
		}()

		next.ServeHTTP(wrappped, r)
	}

	return http.HandlerFunc(fn)
}

// Labels is a middleware function that adds pprof labels to the context of the incoming HTTP request.
func Labels(next http.Handler) http.Handler {
	fn := func(w http.ResponseWriter, r *http.Request) {
		ctx := pprof.WithLabels(r.Context(), pprof.Labels(
			"path", r.URL.Path,
			"method", r.Method,
		))

		pprof.SetGoroutineLabels(ctx)
		next.ServeHTTP(w, r.WithContext(ctx))
	}

	return http.HandlerFunc(fn)
}

type visitor struct {
	limiter  *rate.Limiter
	lastSeen time.Time
}

const (
	qps = 20
)

var visitors = make(map[string]*visitor)
var mu sync.Mutex

func init() {
	go cleanupVisitors()
}

func getVisitor(ip string) *rate.Limiter {
	mu.Lock()
	defer mu.Unlock()

	v, exists := visitors[ip]
	if !exists {
		limiter := rate.NewLimiter(rate.Limit(qps), qps)
		visitors[ip] = &visitor{
			limiter:  limiter,
			lastSeen: time.Now(),
		}
		return limiter
	}
	v.lastSeen = time.Now()

	return v.limiter
}

func cleanupVisitors() {
	for {
		time.Sleep(time.Minute)

		mu.Lock()
		for ip, v := range visitors {
			if time.Since(v.lastSeen) > 2*time.Minute {
				delete(visitors, ip)
			}
		}
		mu.Unlock()
	}
}

func (api *API) Limit(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		// Extract client IP: prefer X-Real-IP (set by Caddy), fallback to RemoteAddr
		clientIP := r.Header.Get("X-Real-IP")
		if clientIP == "" {
			clientIP = r.RemoteAddr
		}
		ip, _, err := net.SplitHostPort(clientIP)
		if err != nil {
			if strings.Contains(err.Error(), "missing port in address") {
				ip = clientIP
			} else {
				api.log.Error("api.Limit: error splitting host port", zap.Error(err), zap.String("client_ip", clientIP))
				http.Error(w, "Internal Server Error", http.StatusInternalServerError)
				return
			}
		}
		limiter := getVisitor(ip)
		if !limiter.Allow() {
			http.Error(w, http.StatusText(http.StatusTooManyRequests), http.StatusTooManyRequests)
			return
		}
		next.ServeHTTP(w, r)
	})
}
