package util

const (
	permErrorMsg = "length of permutation slice & slice not same"
)

// ApplyPermutation applies the permutation perm to slice, mapping new element ID to old element ID.
func ApplyPermutation[T any](slice []T, perm []int) []T {
	AssertPanic(len(slice) == len(perm), permErrorMsg)
	n := len(slice)
	applied := make([]T, n)
	// O(n)
	for i := 0; i < n; i++ {
		applied[i] = slice[perm[i]]
	}
	return applied
}

// ApplyInversePermutation applies the inverse permutation perm to slice, mapping old element ID to new element ID.
func ApplyInversePermutation[T any](slice []T, perm []int) []T {
	AssertPanic(len(slice) == len(perm), permErrorMsg)

	n := len(slice)
	applied := make([]T, n)
	for i := 0; i < n; i++ {
		applied[perm[i]] = slice[i]
	}
	return applied
}
