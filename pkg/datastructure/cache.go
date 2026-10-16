package datastructure

import (
	"github.com/maypok86/otter/v2"
	"github.com/spf13/viper"
)

const (
	pathUnpackEntrySize    = 24 + 9 + 4*4
	initialPuCacheCapacity = 400_000
)

const (
	defaultPathUnpackCacheMaxBytes = 1 << 28 // 256 MB default
)

type PUCacheKey struct {
	source, target Index
	level          uint8
}

func NewPUCacheKey(source, target Index, level uint8) PUCacheKey {
	return PUCacheKey{source: source, target: target, level: level}
}

func NewPuCache() *otter.Cache[PUCacheKey, []Index] {
	maxBytes := defaultPathUnpackCacheMaxBytes
	if viper.IsSet("cache.path_unpacking_size") {
		sizeMB := viper.GetInt("cache.path_unpacking_size")
		maxBytes = sizeMB * 1024 * 1024
	}
	maxSize := maxBytes / pathUnpackEntrySize
	initialCap := initialPuCacheCapacity

	// customizable route planning in road networks section 7.2 (path retrieval)
	puCache := otter.Must(&otter.Options[PUCacheKey, []Index]{
		MaximumSize:     maxSize,
		InitialCapacity: initialCap,
	})
	return puCache
}

const (
	turnSignCacheMaxBytes        uint64 = 1 << 24
	turnSignEntryBytes           uint64 = 16
	turnSigncacheMaximumSize            = int(turnSignCacheMaxBytes / turnSignEntryBytes)
	initialTurnSignCacheCapacity        = 10_000
)

func NewTurnSignCache() *otter.Cache[uint64, uint64] {
	c := otter.Must(&otter.Options[uint64, uint64]{
		MaximumSize:     turnSigncacheMaximumSize,
		InitialCapacity: initialTurnSignCacheCapacity,
	})
	return c
}
