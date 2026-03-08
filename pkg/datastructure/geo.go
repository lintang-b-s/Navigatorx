package datastructure

import "math"

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

func NewBoundingBoxEmpty() *BoundingBox {
	return &BoundingBox{
		minLat: math.MaxFloat64,
		minLon: math.MaxFloat64,
		maxLat: math.Inf(-1),
		maxLon: math.Inf(-1),
	}
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

func (b *BoundingBox) SetMinLat(minLat float64) {
	b.minLat = minLat
}

func (b *BoundingBox) SetMinLon(minLon float64) {
	b.minLon = minLon
}

func (b *BoundingBox) SetMaxLat(maxLat float64) {
	b.maxLat = maxLat
}

func (b *BoundingBox) SetMaxLon(maxLon float64) {
	b.maxLon = maxLon
}
