package datastructure

// IndexStorage represents a storage interface for indexing node/vertex data.
type IndexStorage interface {
	Get(id Index) Index
	Set(id Index, info Index)
	Clear()
}
