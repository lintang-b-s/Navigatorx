package datastructure

type IndexStorage interface {
	Get(id Index) Index
	Set(id Index, info Index)
	Clear()
}
