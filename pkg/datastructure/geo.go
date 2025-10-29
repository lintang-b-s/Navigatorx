package datastructure

type BoundingBox struct {
	minLat, minLon float64
	maxLat, maxLon float64
}

func NewBoundingBox(minLat, minLon, maxLat, maxLon float64) *BoundingBox {
	return &BoundingBox{minLat: minLat,
		minLon: minLon,
		maxLat: maxLat,
		maxLon: maxLon}
}

func (b *BoundingBox) GetMinCoord() (float64, float64) {
	return b.minLat, b.minLon
}

func (b *BoundingBox) GetMaxCoord() (float64, float64) {
	return b.maxLat, b.maxLon
}

func (b *BoundingBox) GetMinLat() float64 {
	return b.minLat
}

func (b *BoundingBox) GetMinLon() float64 {
	return b.minLon
}

func (b *BoundingBox) GetMaxLat() float64 {
	return b.maxLat
}

func (b *BoundingBox) GetMaxLon() float64 {
	return b.maxLon
}
