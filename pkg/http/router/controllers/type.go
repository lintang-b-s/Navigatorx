package controllers

type RoutingService interface {
	ShortestPath(origLat, origLon, dstLat, dstLon float64) (float64, float64, string, bool, error)
}
