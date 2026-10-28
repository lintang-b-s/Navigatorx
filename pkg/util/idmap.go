package util

// IDMap provides bidirectional mapping between strings and uint32 Ids.
type IDMap struct {
	strToId map[string]uint32
	idToStr map[uint32]string
}

// NewIdMap creates an initialized IDMap.
func NewIdMap() IDMap {
	return IDMap{
		strToId: make(map[string]uint32),
		idToStr: make(map[uint32]string),
	}
}

// GetID returns the ID for the given string, registering it if not present.
func (idMap *IDMap) GetID(str string) uint32 {
	if id, ok := idMap.strToId[str]; ok {
		return id
	}
	id := uint32(len(idMap.strToId))
	idMap.strToId[str] = id
	idMap.idToStr[id] = str
	return id
}

// SetID registers an explicit ID for the given string.
func (idMap *IDMap) SetID(id uint32, str string) {
	idMap.strToId[str] = id
	idMap.idToStr[id] = str
}

// GetStr returns the string corresponding to the given ID, or empty string if not found.
func (idMap *IDMap) GetStr(id uint32) string {
	if str, ok := idMap.idToStr[id]; ok {
		return str
	}
	return ""
}

// GetIdToStr returns the map of Ids to strings.
func (idMap *IDMap) GetIdToStr() map[uint32]string {
	return idMap.idToStr
}
