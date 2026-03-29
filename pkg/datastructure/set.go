package datastructure

type Set[T comparable] struct {
	set map[T]struct{}
}

func NewSet[T comparable](length int) *Set[T] {
	return &Set[T]{
		set: make(map[T]struct{}, length),
	}
}

func (s *Set[T]) Set(key T) {
	s.set[key] = struct{}{}
}

func (s *Set[T]) Test(key T) bool {
	_, ok := s.set[key]
	return ok
}
