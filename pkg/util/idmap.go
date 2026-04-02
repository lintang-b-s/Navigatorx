package util

import (
	"sort"
)

type IDMap struct {
	strToId map[string]uint32
	idToStr map[uint32]string
	strs    []string
}

func NewIdMap() IDMap {
	return IDMap{
		strToId: make(map[string]uint32),
		idToStr: make(map[uint32]string),
	}
}

func (idMap *IDMap) GetID(str string) uint32 {
	if id, ok := idMap.strToId[str]; ok {
		return id
	}
	id := uint32(len(idMap.strToId))
	idMap.strToId[str] = id
	idMap.idToStr[id] = str
	return id
}

func (idMap *IDMap) SetID(id uint32, str string) {
	idMap.strToId[str] = id
	idMap.idToStr[id] = str
}

func (idMap *IDMap) GetStr(id uint32) string {
	if str, ok := idMap.idToStr[id]; ok {
		return str
	}
	return ""
}

func (idMap *IDMap) GetIdToStr() map[uint32]string {
	return idMap.idToStr
}

func (idMap *IDMap) ToStringArray(idToStr map[uint32]string) {
	keys := make([]uint32, 0, len(idToStr))
	for key, _ := range idToStr {
		keys = append(keys, key)
	}

	sort.Slice(keys, func(i, j int) bool {
		return i < j
	})

	idMap.strs = make([]string, len(keys))
	for i := 0; i < len(keys); i++ {
		key := keys[i]
		idMap.strs[key] = idToStr[key]
	}
}

func (idMap *IDMap) GetStrFast(id uint32) string {
	return idMap.strs[id]
}
