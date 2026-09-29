package routing

import da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

type AlternativeRoute struct {
	path              *da.Coordinates
	segmentPath       []da.Index
	objectiveValue    float64
	drivingDirections []da.DrivingDirection
	polylinePath      string

	travelTime  float64
	dist        float64
	distSharing float64
	viaNode     da.Index
	viaVertex   ViaVertex
}

func NewAlternativeRoute(objectiveValue, dist, travelTime, distSharing float64,
	viaNode da.Index, path *da.Coordinates, segmentPath []da.Index,
	viaVertex ViaVertex) AlternativeRoute {
	return AlternativeRoute{
		objectiveValue: objectiveValue,
		viaNode:        viaNode,
		path:           path,
		dist:           dist,
		travelTime:     travelTime,
		viaVertex:      viaVertex,
		distSharing:    distSharing,
		segmentPath:    segmentPath,
	}
}

func NewAEmptyAlternativeroute() AlternativeRoute {
	return AlternativeRoute{viaNode: da.INVALID_VERTEX_ID}
}

func isEmptyAlternativeRoute(ar AlternativeRoute) bool {
	return ar.viaNode == da.INVALID_VERTEX_ID
}

func (ar *AlternativeRoute) GetCoords() *da.Coordinates {
	return ar.path
}

func (ar *AlternativeRoute) GetPolylinePath() string {
	return ar.polylinePath
}

func (ar *AlternativeRoute) SetPolylinePath(pp string) {
	ar.polylinePath = pp
}

func (ar *AlternativeRoute) GetDrivingDirections() []da.DrivingDirection {
	return ar.drivingDirections
}
func (ar *AlternativeRoute) SetDrivingDirections(dds []da.DrivingDirection) {
	ddsCopy := make([]da.DrivingDirection, len(dds))
	copy(ddsCopy, dds)
	ar.drivingDirections = ddsCopy
}

func (ar *AlternativeRoute) GetTravelTime() float64 {
	return ar.travelTime
}

func (ar *AlternativeRoute) SetTravelTime(travelTime float64) {
	ar.travelTime = travelTime
}

func (ar *AlternativeRoute) GetDist() float64 {
	return ar.dist
}

func (ar *AlternativeRoute) GetSegmentPath() []da.Index {
	return ar.segmentPath
}

func (ar *AlternativeRoute) SetSegmentPath(segmentPath []da.Index) {
	ar.segmentPath = segmentPath
}

func (ar *AlternativeRoute) SetDist(dist float64) {
	ar.dist = dist
}

type ViaVertex struct {
	v                         da.Index
	overlay                   bool
	plv, lv, approxSharedDist float64
}

func NewViaVertex(v da.Index, overlay bool) ViaVertex {
	return ViaVertex{v: v, overlay: overlay}
}

func NewEmptyViaVertex() ViaVertex {
	return ViaVertex{v: da.INVALID_VERTEX_ID}
}

func isEmptyViaVertex(v ViaVertex) bool {
	return v.v == da.INVALID_VERTEX_ID
}

func (v *ViaVertex) GetVId() da.Index {
	return v.v
}

func (v *ViaVertex) IsOverlay() bool {
	return v.overlay
}

func (v *ViaVertex) SetPlateau(plv float64) {
	v.plv = plv
}

func (v *ViaVertex) SetCost(lv float64) {
	v.lv = lv
}

func (v *ViaVertex) SetApproxSharedDist(approxSigma float64) {
	v.approxSharedDist = approxSigma
}

func (v *ViaVertex) GetPlateau() float64 {
	return v.plv
}

func (v *ViaVertex) GetCost() float64 {
	return v.lv
}

func (v *ViaVertex) GetApproxSharedDist() float64 {
	return v.approxSharedDist
}

func (v *ViaVertex) GetApproxObjectiveValue() float64 {
	return 2*v.GetCost() + v.GetApproxSharedDist() - v.GetPlateau()
}
