package geo

import (
	"math"
	"math/rand"
	"testing"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func BenchmarkMercatorExact(b *testing.B) {
	boundingBox := da.NewBoundingBox(-8.2618, 110.132, -6.888, 110.9221)
	rd := rand.New(rand.NewSource((time.Now().UnixNano())))

	for b.Loop() {
		coord := RandomCoordinate(boundingBox, rd)
		CalcLatToY(coord.GetLat())
	}
}

func BenchmarkMercatorApprox(b *testing.B) {
	boundingBox := da.NewBoundingBox(-8.2618, 110.132, -6.888, 110.9221)
	rd := rand.New(rand.NewSource((time.Now().UnixNano())))

	for b.Loop() {
		coord := RandomCoordinate(boundingBox, rd)
		CalcLatToYApprox(coord.GetLat())
	}
}

func TestMercatorApprox(t *testing.T) {
	numItems := 1e6
	boundingBox := da.NewBoundingBox(-8.2618, 110.132, -6.888, 110.9221)
	rd := rand.New(rand.NewSource((time.Now().UnixNano())))

	for i := 0; i < int(numItems); i++ {
		coord := RandomCoordinate(boundingBox, rd)
		yExact := CalcLatToY(coord.GetLat())
		yApprox := CalcLatToYApprox(coord.GetLat())
		remainder := math.Abs(yExact - yApprox)
		if util.Ge(remainder, maxError) {
			t.Errorf("want error less than: %v, got: %v", maxError, remainder)
		}
	}
}

func RandomCoordinate(bb *da.BoundingBox, rd *rand.Rand) da.Coordinate {

	lat := bb.GetMinLat() + rd.Float64()*(bb.GetMaxLat()-bb.GetMinLat())
	lon := bb.GetMinLon() + rd.Float64()*(bb.GetMaxLon()-bb.GetMinLon())
	return da.NewCoordinate(lat, lon)
}
