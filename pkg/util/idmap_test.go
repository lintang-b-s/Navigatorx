package util

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestNewIdMap(t *testing.T) {
	idMap := NewIdMap()
	assert.NotNil(t, idMap.strToId)
	assert.NotNil(t, idMap.idToStr)
	assert.Empty(t, idMap.strToId)
	assert.Empty(t, idMap.idToStr)
}

func TestIDMap_GetID(t *testing.T) {
	idMap := NewIdMap()

	tests := []struct {
		name     string
		input    string
		expected uint32
	}{
		{"new string 1", "foo", 0},
		{"new string 2", "bar", 1},
		{"existing string 1", "foo", 0},
		{"existing string 2", "bar", 1},
		{"new string 3", "baz", 2},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			id := idMap.GetID(tt.input)
			assert.Equal(t, tt.expected, id)
		})
	}
}

func TestIDMap_SetID(t *testing.T) {
	idMap := NewIdMap()

	idMap.SetID(10, "manual")
	assert.Equal(t, uint32(10), idMap.GetID("manual"))
	assert.Equal(t, "manual", idMap.GetStr(10))
}

func TestIDMap_GetStr(t *testing.T) {
	idMap := NewIdMap()
	idMap.GetID("foo") // ID 0
	idMap.GetID("bar") // ID 1

	tests := []struct {
		name     string
		input    uint32
		expected string
	}{
		{"existing ID 0", 0, "foo"},
		{"existing ID 1", 1, "bar"},
		{"non-existent ID", 99, ""},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			str := idMap.GetStr(tt.input)
			assert.Equal(t, tt.expected, str)
		})
	}
}

func TestIDMap_GetIdToStr(t *testing.T) {
	idMap := NewIdMap()
	idMap.GetID("foo")
	idMap.GetID("bar")

	mapping := idMap.GetIdToStr()
	assert.Len(t, mapping, 2)
	assert.Equal(t, "foo", mapping[0])
	assert.Equal(t, "bar", mapping[1])
}
