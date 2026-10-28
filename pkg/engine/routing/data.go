package routing

import (
	"sort"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

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
	emptyAlternativeRoute = AlternativeRoute{viaNode: da.INVALID_VERTEX_ID}
)

func NewAEmptyAlternativeroute() AlternativeRoute {
	return emptyAlternativeRoute
}

func isEmptyAlternativeRoute(ar AlternativeRoute) bool {
	return ar.viaNode == da.INVALID_VERTEX_ID
}

func (ar AlternativeRoute) GetCoords() *da.Coordinates {
	return ar.path
}

func (ar AlternativeRoute) GetPolylinePath() string {
	return ar.polylinePath
}

func (ar AlternativeRoute) GetDrivingDirections() []da.DrivingDirection {
	return ar.drivingDirections
}

func (ar AlternativeRoute) GetTravelTime() float64 {
	return ar.travelTime
}

func (ar AlternativeRoute) GetDist() float64 {
	return ar.dist
}

func (ar AlternativeRoute) GetSegmentPath() []da.Index {
	return ar.segmentPath
}

type ViaVertex struct {
	v                                 da.Index
	overlay                           bool
	plv, lv, approxSharedDist, sigmav float64 // lv  is cost in seconds. pleatau is the plateau cost in seconds. approxSharedDist is approximate cost sharing with the optimal path in seconds.
	passLimitedSharing, passTTest     bool
	path                              []da.Index
	packedPath                        []da.Index
}

func NewViaVertex(v da.Index, overlay bool) *ViaVertex {
	return &ViaVertex{v: v, overlay: overlay}
}

func (v *ViaVertex) GetVId() da.Index {
	return v.v
}

func (v *ViaVertex) IsOverlay() bool {
	return v.overlay
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

type ParamInterval[T any] struct {
	threshold float64
	val       T
}

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

func (pl ParamLookup[T]) Pick(dist float64) T {
	for i := range pl.intervals {
		if util.Le(dist, pl.intervals[i].threshold) {
			return pl.intervals[i].val
		}
	}
	return pl.defaultVal
}
