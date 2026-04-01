package datastructure

type Set[T comparable] map[T]struct{}

func NewSet[T comparable](length int) *Set[T] {
	set := make(map[T]struct{}, length)
	return (*Set[T])(&set)
}

// https://go.dev/doc/effective_go#pointers_vs_values
func (s *Set[T]) Set(key T) {
	set := *s
	set[key] = struct{}{}
	*s = set
}

// https://go.dev/doc/effective_go#pointers_vs_values
func (s *Set[T]) Test(key T) bool {
	set := *s
	_, ok := set[key]
	return ok
}
