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

// var (
// inspired by http/2 sync.pool example in: https://wundergraph.com/blog/golang-sync-pool#3.-complexity-trade-offs
// jsonEncSizeClasses = [9]int{
// 		1 << 8,
// 		1 << 9,
// 		1 << 10,
// 		1 << 11,
// 		1 << 12,
// 		1 << 13,
// 		1 << 14,
// 		1 << 15,
// 		1 << 16,
// 	}
// 	jencPools [9]sync.Pool = [9]sync.Pool{
// 		{New: func() interface{} {
// 			buf := bytes.NewBuffer(make([]byte, 0, 1<<8))
// 			return &encoderWrapper{
// 				buf:     buf,
// 				encoder: json.ConfigDefault.NewEncoder(buf),
// 			}
// 		}},
// 		{New: func() interface{} {
// 			buf := bytes.NewBuffer(make([]byte, 0, 1<<9))
// 			return &encoderWrapper{
// 				buf:     buf,
// 				encoder: json.ConfigDefault.NewEncoder(buf),
// 			}
// 		}},
// 		{New: func() interface{} {
// 			buf := bytes.NewBuffer(make([]byte, 0, 1<<10))
// 			return &encoderWrapper{
// 				buf:     buf,
// 				encoder: json.ConfigDefault.NewEncoder(buf),
// 			}
// 		}},
// 		{New: func() interface{} {
// 			buf := bytes.NewBuffer(make([]byte, 0, 1<<11))
// 			return &encoderWrapper{
// 				buf:     buf,
// 				encoder: json.ConfigDefault.NewEncoder(buf),
// 			}
// 		}},
// 		{New: func() interface{} {
// 			buf := bytes.NewBuffer(make([]byte, 0, 1<<12))
// 			return &encoderWrapper{
// 				buf:     buf,
// 				encoder: json.ConfigDefault.NewEncoder(buf),
// 			}
// 		}},
// 		{New: func() interface{} {
// 			buf := bytes.NewBuffer(make([]byte, 0, 1<<13))
// 			return &encoderWrapper{
// 				buf:     buf,
// 				encoder: json.ConfigDefault.NewEncoder(buf),
// 			}
// 		}},
// 		{New: func() interface{} {
// 			buf := bytes.NewBuffer(make([]byte, 0, 1<<14))
// 			return &encoderWrapper{
// 				buf:     buf,
// 				encoder: json.ConfigDefault.NewEncoder(buf),
// 			}
// 		}},
// 		{New: func() interface{} {
// 			buf := bytes.NewBuffer(make([]byte, 0, 1<<15))
// 			return &encoderWrapper{
// 				buf:     buf,
// 				encoder: json.ConfigDefault.NewEncoder(buf),
// 			}
// 		}},

// 		{New: func() interface{} {
// 			buf := bytes.NewBuffer(make([]byte, 0, 1<<16))
// 			return &encoderWrapper{
// 				buf:     buf,
// 				encoder: json.ConfigDefault.NewEncoder(buf),
// 			}
// 		}},
// 	}
// )

// func getJsonEnc(size int) []uint32 {
// 	i := 0
// 	for ; i < len(jsonEncSizeClasses)-1; i++ {
// 		if size <= int(jsonEncSizeClasses[i]) {
// 			break
// 		}
// 	}
// 	s := jencPools[i].Get().([]uint32)
// 	return s
// }

// func putJsonEnc(s []uint32) {
// 	size := len((s))
// 	for i := range len(jsonEncSizeClasses) {
// 		if size <= int(jsonEncSizeClasses[i]) {
// 			jencPools[i].Put(s)
// 			return
// 		}
// 	}
// }
