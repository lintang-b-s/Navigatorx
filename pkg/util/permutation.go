package util

// ApplyPermutation. apply permutation perm to slice
// perm maps new element id to old element id
func ApplyPermutation[T any](slice []T, perm []int) []T {
	AssertPanic(len(slice) == len(perm), "length of permutation slice & slice not same")
	n := len(slice)
	applied := make([]T, n)
	// O(n)
	for i := 0; i < n; i++ {
		applied[i] = slice[perm[i]]
	}
	return applied
}

func ApplyInversePermutation[T any](slice []T, perm []int) []T {
	AssertPanic(len(slice) == len(perm), "length of permutation slice & slice not same")

	n := len(slice)
	applied := make([]T, n)
	for i := 0; i < n; i++ {
		applied[perm[i]] = slice[i]
	}
	return applied
}
