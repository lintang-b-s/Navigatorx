package controllers

type shortestPathRequest struct {
	OriginLat      float64 `json:"origin_lat" validate:"required,min=-90,max=90"`
	OriginLon      float64 `json:"origin_lon" validate:"required,min=-180,max=180"`
	DestinationLat float64 `json:"destination_lat" validate:"required,min=-90,max=90"`
	DestinationLon float64 `json:"destination_lon" validate:"required,min=-180,max=180"`
}

type shortestPathResponse struct {
	Eta  float64 `json:"eta"`
	Path string  `json:"path"`
	Dist float64 `json:"distance"`
}

func NewShortestPathResponse(eta, dist float64, path string) shortestPathResponse {
	return shortestPathResponse{
		Eta:  eta,
		Path: path,
		Dist: dist,
	}
}

type errorResponse struct {
	Error struct {
		Code    string `json:"code"`
		Message string `json:"message"`
	} `json:"error"`
}
