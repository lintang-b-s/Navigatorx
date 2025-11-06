package controllers

import (
	"encoding/json"
	"fmt"
	"io"
	"net"
	"net/http"
	"sort"
	"sync"

	"github.com/go-playground/locales/en"
	ut "github.com/go-playground/universal-translator"
	"github.com/go-playground/validator/v10"
	enTranslations "github.com/go-playground/validator/v10/translations/en"
	"github.com/gobwas/ws"
	"github.com/gobwas/ws/wsutil"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
)

type User struct {
	io   sync.Mutex
	conn io.ReadWriteCloser

	id  uint
	hub *Hub
}

func (u *User) readRequest() (*mapMatchRequest, error) {
	u.io.Lock()
	defer u.io.Unlock()

	h, r, err := wsutil.NextReader(u.conn, ws.StateServerSide)
	if err != nil {
		return nil, err
	}
	if h.OpCode.IsControl() {
		return nil, wsutil.ControlFrameHandler(u.conn, ws.StateServerSide)(h, r)
	}

	req := &mapMatchRequest{}
	decoder := json.NewDecoder(r)
	if err := decoder.Decode(req); err != nil {
		return nil, err
	}
	return req, nil
}

func (u *User) OnlineMapMatch() error {
	req, err := u.readRequest()
	if err != nil {
		u.conn.Close()
		return err
	}

	if req == nil {
		return nil
	}

	validate := validator.New()

	if err := validate.Struct(req); err != nil {
		english := en.New()
		uni := ut.New(english, english)
		trans, _ := uni.GetTranslator("en")
		_ = enTranslations.RegisterDefaultTranslations(validate, trans)
		vv := translateError(err, trans)
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

	mgpsPoint, cands, speedMeanK, speedStdK := u.hub.mapmatchingService.OnlineMapMatch(
		req.Gps.ToDataGPS(),
		req.K, ToOnlineCandidates(req.Candidates),
		req.SpeedMeanK, req.SpeedStdK, req.LastBearing)

	resp := envelope{"data": NewMapmatchingResponse(mgpsPoint, cands, speedMeanK,
		speedStdK, mgpsPoint.GetBearing())}
	return u.write(resp)
}

func (u *User) write(x interface{}) error {
	w := wsutil.NewWriter(u.conn, ws.StateServerSide, ws.OpText)
	encoder := json.NewEncoder(w)

	u.io.Lock()
	defer u.io.Unlock()

	if err := encoder.Encode(x); err != nil {
		return err
	}

	return w.Flush()
}

type Hub struct {
	mu                 sync.RWMutex
	seq                uint
	us                 []*User
	ns                 map[uint]*User
	mapmatchingService MapMatcherService

	pool *concurrent.WorkerPool[int, int]
}

func NewHub(pool *concurrent.WorkerPool[int, int], mmService MapMatcherService) *Hub {
	hub := &Hub{
		pool:               pool,
		ns:                 make(map[uint]*User),
		us:                 make([]*User, 0),
		mapmatchingService: mmService,
	}

	return hub
}

func (h *Hub) Register(conn net.Conn) *User {
	user := &User{
		hub:  h,
		conn: conn,
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
	if _, oki := h.ns[user.id]; !oki {
		return
	}
	delete(h.ns, user.id)

	i := sort.Search(len(h.us), func(i int) bool {
		return h.us[i].id >= user.id
	})

	newUs := make([]*User, len(h.us)-1)
	copy(newUs[:i], h.us[:i])
	copy(newUs[i:], h.us[i+1:])
	h.us = newUs

	h.mu.Unlock()
}

func (h *Hub) RemoveAllUser() {
	for _, user := range h.us {
		h.Remove(user)
	}
}
