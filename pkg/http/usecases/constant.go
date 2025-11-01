package usecases

import "errors"

var (
	ERRPATHNOTFOND = errors.New("no path found from origin to destination")
)

const (
	UPPERBOUND_SHORTEST_PATH = 1.0
)
