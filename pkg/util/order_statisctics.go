package util

import "math/rand/v2"

// todo: implement MedianOfNinthers k-th order statistics algorithm: https://drops.dagstuhl.de/entities/document/10.4230/LIPIcs.SEA.2017.24

// RandomizedSelect return the i-th (1-indexed) smallest element (or largest depends on comp) of the array arr[p...r]
// & partition the arr such that all elements (arr[p,..q]) left of i-th smallest element  are smaller (or largest depends on comp) than  the pivot element arr[q] & all elements (arr[q+1,...,r]) in the right of i-th smallest element
// expected runtime O(n), n=len(arr). worst case O(n^2)
// read chapter 9.2 CLRS (introduction to algorithm by Cormen, et al. 3rd edition) for the time complexity analysis
// or https://ocw.mit.edu/courses/6-046j-design-and-analysis-of-algorithms-spring-2012/cc13d40706471ea882c39f315861ce64_MIT6_046JS12_lec08.pdf
func RandomizedSelect[T comparable](arr []T, p, r, i int, comp func(left, right int) bool) int {
	if p == r {
		return p
	}

	q := randomizedPartition(arr, p, r, comp)
	k := q - p + 1 // size of arr[p,...,q] (include pivot element arr[q])
	if i == k {
		return q
	} else if i < k {
		return RandomizedSelect(arr, p, q-1, i, comp)
	}
	return RandomizedSelect(arr, q+1, r, i-k, comp) // i-k th smallest/largest element di arr[q+1,...,r] karena di next recursion kita operate di arr[q+1,...,r]
}

// randomizedPartition random pivot selection & partition function of randomized quicksort
func randomizedPartition[T comparable](arr []T, p, r int, comp func(left, right int) bool) int {
	i := p - 1

	pivotId := p + rand.IntN(r-p+1)
	arr[pivotId], arr[r] = arr[r], arr[pivotId]
	for j := p; j < r; j++ {
		if comp(j, r) {
			i++
			arr[i], arr[j] = arr[j], arr[i]
		}
	}

	arr[i+1], arr[r] = arr[r], arr[i+1]

	return i + 1
}
