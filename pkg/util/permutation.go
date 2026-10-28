package util

const (
	permErrorMsg = "length of permutation slice & slice not same"
)

// ApplyPermutation. apply permutation perm to slice
// perm maps new element id to old element id
func ApplyPermutation[T any](slice []T, perm []int) []T {
	AssertFatal(len(slice) == len(perm), permErrorMsg)
	n := len(slice)
	applied := make([]T, n)
	// O(n)
	for i := 0; i < n; i++ {
		applied[i] = slice[perm[i]]
	}
	return applied
}

// ApplyInversePermutation
// perm maps old element id to new element id
func ApplyInversePermutation[T any](slice []T, perm []int) []T {
	AssertFatal(len(slice) == len(perm), permErrorMsg)

	n := len(slice)
	applied := make([]T, n)
	for i := 0; i < n; i++ {
		applied[perm[i]] = slice[i]
	}
	return applied
}
