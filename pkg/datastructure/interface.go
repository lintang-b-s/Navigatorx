package datastructure

type QueryInfoStorage interface {
	Get(id Index) int
	Set(id Index, info int)
	Clear()
	Clone() QueryInfoStorage
}
