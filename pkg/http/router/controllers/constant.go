// Package controllers defines the HTTP controllers for the routing engine.
package controllers

import (
	"bytes"
	"fmt"
	"sync"

	json "github.com/bytedance/sonic"

	"github.com/lintang-b-s/Navigatorx/pkg"
)

const (
	// JSONBufPoolSize is the jsonbuf pool size value.
	JSONBufPoolSize = 1 << 14
	// MbToBytes is the mb to bytes value.
	MbToBytes = 1000000
)

// GetMapMatchingTransitionFile returns map matching transition file.
func GetMapMatchingTransitionFile() string {
	return fmt.Sprintf("./data/profiles/%s/%s_transition_matrix.ntm", pkg.ProfileName, pkg.RegionName)
}

// JSONEncoderPool represents jsonencoder pool.
type JSONEncoderPool struct {
	pool sync.Pool
}

type encoderWrapper struct {
	buf     *bytes.Buffer
	encoder json.Encoder
}

// NewJSONEncoderPool creates and returns jsonencoder pool.
func NewJSONEncoderPool(size int) *JSONEncoderPool {
	return &JSONEncoderPool{
		pool: sync.Pool{
			New: func() any {
				buf := bytes.NewBuffer(make([]byte, 0, size))
				return &encoderWrapper{
					buf:     buf,
					encoder: json.ConfigFastest.NewEncoder(buf),
				}
			},
		},
	}
}

var (
	jencPool = NewJSONEncoderPool(JSONBufPoolSize)
)
