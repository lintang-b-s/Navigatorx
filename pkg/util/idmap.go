package util

type IDMap struct {
	strToId map[string]uint32
	idToStr map[uint32]string
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
