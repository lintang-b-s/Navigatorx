package usecases

import "errors"

var (
	ERRPATHNOTFOND = errors.New("no path found from origin to destination")
)

const (
	UPPERBOUND_SHORTEST_PATH = 1.0
	MAX_SEARCH_RADIUS        = 4.0 // 4km
	SEARCH_RADIUS_MULTIPLIER = 2.0
)
