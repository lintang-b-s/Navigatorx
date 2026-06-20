package datastructure

import "github.com/maypok86/otter/v2"

const (
	pathUnpackCacheMaxBytes uint64 = 1 << 28
	pathUnpackEntryOverhead        = 24 + 9 + 4*4
	pathUnpackerMaxSize            = int(pathUnpackCacheMaxBytes / pathUnpackEntryOverhead)
	initialPuCacheCapacity         = 400_000
)

type PUCacheKey struct {
	source, target Index
	level          uint8
}

func NewPUCacheKey(source, target Index, level uint8) PUCacheKey {
	return PUCacheKey{source: source, target: target, level: level}
}

func NewPuCache() *otter.Cache[PUCacheKey, []Index] {
	puCache := otter.Must(&otter.Options[PUCacheKey, []Index]{
		MaximumSize:     pathUnpackerMaxSize,
		InitialCapacity: initialPuCacheCapacity,
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
