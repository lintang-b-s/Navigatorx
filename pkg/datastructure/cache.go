package datastructure

import (
	"github.com/maypok86/otter/v2"
	"github.com/spf13/viper"
)

const (
	pathUnpackEntrySize    = 24 + 8 + 4*4
	initialPuCacheCapacity = 700_000
)

const (
	defaultPathUnpackCacheMaxBytes = 1 << 28 // 256 MB default
	defaultMaxSize                 = defaultPathUnpackCacheMaxBytes / pathUnpackEntrySize
)

// NewPUCacheKey creates and returns pucache key.
func NewPUCacheKey(source, target Index, level uint8) uint64 {
	// bit packed [level, target, source] pos=[61-64, 31-60, 1-30]
	// i assume max number boundary/overlay vertices 2^30 - 1... (around 1 billion)
	k := uint64(source) | (uint64(target) << 30) | (uint64(level) << 60)
	return k
}

// NewPuCache creates and returns pu cache.
func NewPuCache() *otter.Cache[uint64, []Index] {
	maxBytes := defaultPathUnpackCacheMaxBytes
	if viper.IsSet("cache.path_unpacking_size") {
		sizeMB := viper.GetInt("cache.path_unpacking_size")
		maxBytes = sizeMB * 1024 * 1024
	}
	maxSize := maxBytes / pathUnpackEntrySize
	initialCap := initialPuCacheCapacity

	// customizable route planning in road networks section 7.2 (path retrieval)
	puCache := otter.Must(&otter.Options[uint64, []Index]{
		MaximumSize:     maxSize,
		InitialCapacity: initialCap,
	})
	return puCache
}

const (
	turnSignCacheMaxBytes        uint64 = 1 << 25
	turnSignEntryBytes           uint64 = 16
	turnSigncacheMaximumSize            = int(turnSignCacheMaxBytes / turnSignEntryBytes)
	initialTurnSignCacheCapacity        = 10_000
)

// NewTurnSignCache creates and returns turn sign cache.
func NewTurnSignCache() *otter.Cache[uint64, uint64] {
	c := otter.Must(&otter.Options[uint64, uint64]{
		MaximumSize:     turnSigncacheMaximumSize,
		InitialCapacity: initialTurnSignCacheCapacity,
	})
	return c
}
