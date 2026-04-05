package datastructure

type QueryInfoStorage interface {
	Get(id Index) uint32
	Set(id Index, info uint32)
	Clear()
	Clone() QueryInfoStorage
	ForAllItems(handle func(offsetedVId Index, queryInfoId uint32))
}

type ScannedSetStorage interface {
	Test(queryInfoId uint32) bool
	Set(queryInfoId uint32)
}
