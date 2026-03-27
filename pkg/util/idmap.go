package util

import "sort"

type IDMap struct {
	strToId map[string]int
	idToStr map[int]string
	strs    []string
}

func NewIdMap() IDMap {
	return IDMap{
		strToId: make(map[string]int),
		idToStr: make(map[int]string),
	}
}

func (idMap *IDMap) GetID(str string) int {
	if id, ok := idMap.strToId[str]; ok {
		return id
	}
	id := len(idMap.strToId)
	idMap.strToId[str] = id
	idMap.idToStr[id] = str
	return id
}

func (idMap *IDMap) SetID(id int, str string) {
	idMap.strToId[str] = id
	idMap.idToStr[id] = str
}

func (idMap *IDMap) GetStr(id int) string {
	if str, ok := idMap.idToStr[id]; ok {
		return str
	}
	return ""
}

func (idMap *IDMap) GetIdToStr() map[int]string {
	return idMap.idToStr
}

func (idMap *IDMap) ToStringArray() {
	keys := make([]int, 0, len(idMap.idToStr))
	for key, _ := range idMap.idToStr {
		keys = append(keys, key)
	}

	sort.Slice(keys, func(i, j int) bool {
		return i < j
	})

	idMap.strs = make([]string, len(keys))
	for key := range keys {
		idMap.strs[key] = idMap.GetStr(key)
	}
}

func (idMap *IDMap) GetStrFast(id int) string {
	return idMap.strs[id]
}
