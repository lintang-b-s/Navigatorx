package mapattributes

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (ma *MapAttributesEngine[W]) GetPrevPoint(segmentId da.Index, tailCoord da.Coordinate, atLeastDist float64) da.Coordinate {

	curved := ma.rn.IsCurved(segmentId)
	n := ma.rn.GetSegmentGeometryLength(segmentId)
	if !curved || n == 2 {
		return ma.rn.GetSegmentGeometryPoint(segmentId, 0)
	}

	v := ma.rn.GetSegmentGeometryPoint(segmentId, int(n-2))
	for i := int(n - 2); int(i) >= 0; i-- {
		p := ma.rn.GetSegmentGeometryPoint(segmentId, int(i))
		dist := geo.CalculateEuclideanDistMercatorProj(tailCoord.GetLat(), tailCoord.GetLon(), p.GetLat(), p.GetLon())
		if util.Ge(dist, atLeastDist) {
			v = p
			break
		}
	}

	return v
}

// GetHeadPoint. ini buat get headPoint, point setelah tail vertex/intersection vertex.
// mirip kaya GetPrevPoint, tapi pakai geometry dari currentEdge.
func (ma *MapAttributesEngine[W]) GetHeadPoint(eId da.Index, tailCoord da.Coordinate, atLeastDist float64) da.Coordinate {

	curved := ma.rn.IsCurved(eId)
	n := ma.rn.GetSegmentGeometryLength(eId)
	if !curved || n == 2 {
		return ma.rn.GetSegmentGeometryPoint(eId, 1)
	}

	v := ma.rn.GetSegmentGeometryPoint(eId, 1)
	for i := 1; i < int(n); i++ {
		p := ma.rn.GetSegmentGeometryPoint(eId, i)
		dist := geo.CalculateEuclideanDistMercatorProj(tailCoord.GetLat(), tailCoord.GetLon(), p.GetLat(), p.GetLon())
		if util.Ge(dist, atLeastDist) {
			v = p
			break
		}
	}

	return v
}
