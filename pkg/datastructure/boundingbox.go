package datastructure

import "math"

// BoundingBox represents a 2D geographic bounding box defined by min/max latitude and longitude.
type BoundingBox struct {
	minLat, minLon float64
	maxLat, maxLon float64
}

// NewBoundingBox creates a new BoundingBox with the given coordinate bounds.
func NewBoundingBox(minLat, minLon, maxLat, maxLon float64) *BoundingBox {
	return &BoundingBox{minLat: minLat,
		minLon: minLon,
		maxLat: maxLat,
		maxLon: maxLon}
}

// NewBoundingBoxEmpty creates an empty BoundingBox initialized to infinite/inverted bounds.
func NewBoundingBoxEmpty() *BoundingBox {
	return &BoundingBox{
		minLat: math.MaxFloat64,
		minLon: math.MaxFloat64,
		maxLat: math.Inf(-1),
		maxLon: math.Inf(-1),
	}
}

// GetMinCoord returns the minimum latitude and longitude coordinates.
func (b *BoundingBox) GetMinCoord() (float64, float64) {
	return b.minLat, b.minLon
}

// GetMaxCoord returns the maximum latitude and longitude coordinates.
func (b *BoundingBox) GetMaxCoord() (float64, float64) {
	return b.maxLat, b.maxLon
}

// GetMinLat returns the minimum latitude.
func (b *BoundingBox) GetMinLat() float64 {
	return b.minLat
}

// GetMinLon returns the minimum longitude.
func (b *BoundingBox) GetMinLon() float64 {
	return b.minLon
}

// GetMaxLat returns the maximum latitude.
func (b *BoundingBox) GetMaxLat() float64 {
	return b.maxLat
}

// GetMaxLon returns the maximum longitude.
func (b *BoundingBox) GetMaxLon() float64 {
	return b.maxLon
}

// SetMinLat sets the minimum latitude.
func (b *BoundingBox) SetMinLat(minLat float64) {
	b.minLat = minLat
}

// SetMinLon sets the minimum longitude.
func (b *BoundingBox) SetMinLon(minLon float64) {
	b.minLon = minLon
}

// SetMaxLat sets the maximum latitude.
func (b *BoundingBox) SetMaxLat(maxLat float64) {
	b.maxLat = maxLat
}

// SetMaxLon sets the maximum longitude.
func (b *BoundingBox) SetMaxLon(maxLon float64) {
	b.maxLon = maxLon
}
