package datastructure

type IndexStorage interface {
	Get(id Index) uint32
	Set(id Index, info uint32)
	Clear()
	Clone() IndexStorage
	ForAllItems(handle func(offsetedVId Index, vertexIndex uint32))
}

type ExploredSetStorage interface {
	Test(vertexIndex uint32) bool
	Set(vertexIndex uint32)
	Clear(maxEdgesInCell uint32)
}
