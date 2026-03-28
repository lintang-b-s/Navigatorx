package usecases

import "errors"

var (
	ERRPATHNOTFOND = errors.New("no path found from origin to destination")
)

const (
	UPPERBOUND_SHORTEST_PATH = 1.0
	MAX_SEARCH_RADIUS        = 4.0 // 4km
	SEARCH_RADIUS_MULTIPLIER = 2.0 // 0.06 -> 0.12 -> 0.24 -> 0.48 -> 0.96 -> 1.92 -> 3.84 -> 7.68 
)
