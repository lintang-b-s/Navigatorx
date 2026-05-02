package controllers

import (
	"context"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"sort"
	"sync"
	"time"

	json "github.com/bytedance/sonic"

	"github.com/go-playground/locales/en"
	ut "github.com/go-playground/universal-translator"
	"github.com/go-playground/validator/v10"
	enTranslations "github.com/go-playground/validator/v10/translations/en"
	"github.com/gobwas/ws"
	"github.com/gobwas/ws/wsutil"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"go.uber.org/zap"
)

// ErrNormalClosure is returned when the client sends a normal WebSocket close frame (status 1000).
// This is expected behavior when a user stops navigation, not a real error.
var ErrNormalClosure = errors.New("ws normal closure")

type User struct {
	io   sync.Mutex
	conn io.ReadWriteCloser

	id        uint
	hub       *Hub
	log       *zap.Logger
	clientIP  string
	userAgent string
}

func (u *User) readRequest() (*mapMatchRequest, error) {
	u.io.Lock()
	defer u.io.Unlock()

	h, r, err := wsutil.NextReader(u.conn, ws.StateServerSide)
	if err != nil {
		return nil, err
	}
	if h.OpCode.IsControl() {
		if h.OpCode == ws.OpClose {
			// Normal close frame from client (e.g. user stopped navigation).
			// Handle the frame to send close response, then return sentinel error.
			_ = wsutil.ControlFrameHandler(u.conn, ws.StateServerSide)(h, r)
			return nil, ErrNormalClosure
		}
		// handle OpPing, OpPong
		return nil, wsutil.ControlFrameHandler(u.conn, ws.StateServerSide)(h, r)
	}

	req := &mapMatchRequest{}
	decoder := json.ConfigDefault.NewDecoder(r)
	if err := decoder.Decode(req); err != nil {
		return nil, err
	}
	return req, nil
}

func (u *User) OnlineMapMatch() error {
	start := time.Now()

	req, err := u.readRequest()
	if err != nil {
		u.conn.Close()
		u.hub.Remove(u)
		return err
	}

	if req == nil {
		return nil
	}

	if err := u.hub.validate.Struct(req); err != nil {
		vv := translateError(err, u.hub.trans)
		vvString := []string{}
		for _, v := range vv {
			vvString = append(vvString, v.Error())
		}
		errResp := envelope{"error": map[string]string{
			"code":    http.StatusText(http.StatusBadRequest),
			"message": fmt.Sprintf("validation error: %v", vvString),
		}}
		return u.write(errResp)
	}

	mgpsPoint, cands, speedMeanK, speedStdK, err := u.hub.mapmatchingService.OnlineMapMatch(
		context.Background(),
		req.Gps.ToDataGPS(),
		req.K, ToOnlineCandidates(req.Candidates),
		req.SpeedMeanK, req.SpeedStdK, req.LastBearing)
	if err != nil {
		errResp := envelope{"error": map[string]string{
			"code":    http.StatusText(http.StatusInternalServerError),
			"message": err.Error(),
		}}
		return u.write(errResp)
	}

	resp := envelope{"data": NewMapmatchingResponse(mgpsPoint, cands, speedMeanK,
		speedStdK, mgpsPoint.GetBearing())}

	// log ws map matching request completion
	u.log.Info("ws online map matching completed",
		zap.Int64("took", time.Since(start).Milliseconds()),
		zap.Uint("user_id", u.id),
		zap.String("client_ip", u.clientIP),
		zap.String("user_agent", u.userAgent),
		zap.Float64("lat", req.Gps.Lat),
		zap.Float64("lon", req.Gps.Lon),
		zap.String("gps_time", req.Gps.Time.Local().String()),
		zap.Float64("speed_ms", req.Gps.Speed), // meter/seconds
		zap.Int("k", req.K),
		zap.Float64("delta_time_s", req.Gps.DeltaTime),
		zap.Bool("dead_reckoning", req.Gps.DeadReckoning),
		zap.Float64("last_bearing", req.LastBearing),
	)

	return u.write(resp)
}

func (u *User) write(x interface{}) error {
	w := wsutil.NewWriter(u.conn, ws.StateServerSide, ws.OpText)
	encoder := json.ConfigDefault.NewEncoder(w)

	u.io.Lock()
	defer u.io.Unlock()

	if err := encoder.Encode(x); err != nil {
		return err
	}

	return w.Flush()
}

func (u *User) Ping() error {
	u.io.Lock()
	defer u.io.Unlock()

	return wsutil.WriteServerMessage(u.conn, ws.OpPing, nil)
}

type Hub struct {
	mu                 sync.RWMutex
	seq                uint
	us                 []*User
	ns                 map[uint]*User
	mapmatchingService MapMatcherService

	pool *concurrent.WorkerPool[int, int]
	log  *zap.Logger

	validate *validator.Validate
	trans    ut.Translator

	ctx    context.Context
	cancel context.CancelFunc
}

func NewHub(pool *concurrent.WorkerPool[int, int], mmService MapMatcherService, log *zap.Logger) *Hub {
	ctx, cancel := context.WithCancel(context.Background())
	hub := &Hub{
		pool:               pool,
		ns:                 make(map[uint]*User),
		us:                 make([]*User, 0),
		mapmatchingService: mmService,
		log:                log,
		ctx:                ctx,
		cancel:             cancel,
	}
	hub.validate = validator.New()

	english := en.New()
	uni := ut.New(english, english)
	hub.trans, _ = uni.GetTranslator("en")
	_ = enTranslations.RegisterDefaultTranslations(hub.validate, hub.trans)

	return hub
}

func (h *Hub) Start() {
	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			h.mu.RLock()
			for _, u := range h.us {
				if err := u.Ping(); err != nil {
					h.log.Debug("failed to ping user", zap.Uint("user_id", u.id), zap.Error(err))
				}
			}
			h.mu.RUnlock()
		case <-h.ctx.Done():
			return
		}
	}
}

func (h *Hub) Stop() {
	h.cancel()
}

func (h *Hub) Register(conn net.Conn, clientIP, userAgent string) *User {

	user := &User{
		hub:       h,
		conn:      conn,
		log:       h.log,
		clientIP:  clientIP,
		userAgent: userAgent,
	}

	h.mu.Lock()
	user.id = h.seq
	h.ns[user.id] = user
	h.us = append(h.us, user)

	h.seq++
	h.mu.Unlock()

	return user
}

func (h *Hub) Remove(user *User) {

	h.mu.Lock()
	defer h.mu.Unlock()
	if _, oki := h.ns[user.id]; !oki {
		return
	}
	h.log.Info("user disconnected from websocket server",
		zap.Uint("user_id", user.id),
		zap.String("client_ip", user.clientIP),
		zap.String("user_agent", user.userAgent),
	)
	delete(h.ns, user.id)

	i := sort.Search(len(h.us), func(i int) bool {
		return h.us[i].id >= user.id
	})

	newUs := make([]*User, len(h.us)-1)
	copy(newUs[:i], h.us[:i])
	copy(newUs[i:], h.us[i+1:])
	h.us = newUs

}

func (h *Hub) RemoveAllUser() {
	for _, user := range h.us {
		h.Remove(user)
	}
}
