package routing

import (
	"sort"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// AlternativeRoute represents alternative route.
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
	viaVertex   *ViaVertex
}

// NewAlternativeRoute creates and returns alternative route.
func NewAlternativeRoute(objectiveValue, dist, travelTime, distSharing float64,
	viaNode da.Index, path *da.Coordinates, segmentPath []da.Index,
	viaVertex *ViaVertex) AlternativeRoute {
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

var (
	emptyAlternativeRoute = AlternativeRoute{viaNode: da.InvalidVertexId}
)

// NewAEmptyAlternativeroute creates and returns aempty alternativeroute.
func NewAEmptyAlternativeroute() AlternativeRoute {
	return emptyAlternativeRoute
}

func isEmptyAlternativeRoute(ar AlternativeRoute) bool {
	return ar.viaNode == da.InvalidVertexId
}

// GetCoords returns coords.
func (ar AlternativeRoute) GetCoords() *da.Coordinates {
	return ar.path
}

// GetPolylinePath returns polyline path.
func (ar AlternativeRoute) GetPolylinePath() string {
	return ar.polylinePath
}

// GetDrivingDirections returns driving directions.
func (ar AlternativeRoute) GetDrivingDirections() []da.DrivingDirection {
	return ar.drivingDirections
}

// GetTravelTime returns travel time.
func (ar AlternativeRoute) GetTravelTime() float64 {
	return ar.travelTime
}

// GetDist returns dist.
func (ar AlternativeRoute) GetDist() float64 {
	return ar.dist
}

// GetSegmentPath returns segment path.
func (ar AlternativeRoute) GetSegmentPath() []da.Index {
	return ar.segmentPath
}

// ViaVertex represents via vertex.
type ViaVertex struct {
	v                                 da.Index
	overlay                           bool
	plv, lv, approxSharedDist, sigmav float64 // lv  is cost in seconds. pleatau is the plateau cost in seconds. approxSharedDist is approximate cost sharing with the optimal path in seconds.
	passLimitedSharing, passTTest     bool
	path                              []da.Index
	packedPath                        []da.Index
}

// NewViaVertex creates and returns via vertex.
func NewViaVertex(v da.Index, overlay bool) *ViaVertex {
	return &ViaVertex{v: v, overlay: overlay}
}

// GetVId get vertex id
func (v *ViaVertex) GetVId() da.Index {
	return v.v
}

// IsOverlay is boundary/overlay vertex
func (v *ViaVertex) IsOverlay() bool {
	return v.overlay
}

// GetPlateau get plateau value
func (v *ViaVertex) GetPlateau() float64 {
	return v.plv
}

// Cost get cost of this alternative route s-via-t
func (v *ViaVertex) Cost() float64 {
	return v.lv
}

// GetApproxSharedDist returns approx shared dist.
func (v *ViaVertex) GetApproxSharedDist() float64 {
	return v.approxSharedDist
}

// GetApproxObjectiveValue returns approx objective value.
func (v *ViaVertex) GetApproxObjectiveValue() float64 {
	return 2*v.Cost() + v.GetApproxSharedDist() - v.GetPlateau()
}

// ParamInterval represents param interval.
type ParamInterval[T any] struct {
	threshold float64
	val       T
}

// ParamLookup represents param lookup.
type ParamLookup[T any] struct {
	intervals  []ParamInterval[T]
	defaultVal T
}

func newParamLookup[T any](paramMap map[float64]T, defaultVal T) ParamLookup[T] {
	keys := make([]float64, 0, len(paramMap))
	for k := range paramMap {
		keys = append(keys, k)
	}
	sort.Float64s(keys)
	intervals := make([]ParamInterval[T], len(keys))
	for i, k := range keys {
		intervals[i] = ParamInterval[T]{threshold: k, val: paramMap[k]}
	}
	return ParamLookup[T]{intervals: intervals, defaultVal: defaultVal}
}

// Pick handles pick.
func (pl ParamLookup[T]) Pick(dist float64) T {
	for i := range pl.intervals {
		if util.Le(dist, pl.intervals[i].threshold) {
			return pl.intervals[i].val
		}
	}
	return pl.defaultVal
}
