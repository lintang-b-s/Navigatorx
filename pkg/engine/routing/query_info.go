package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type pathUnpackingParam struct {
	sourceOverlayId da.Index
	targetOverlayId da.Index
	level           uint8
	index           int
}

func NewPathUnpackingParam(sourceOverlayId, targetOverlayId da.Index, level uint8, index int) pathUnpackingParam {
	return pathUnpackingParam{sourceOverlayId: sourceOverlayId, targetOverlayId: targetOverlayId, level: level, index: index}
}

func (p *pathUnpackingParam) getSourceOverlayId() da.Index {
	return p.sourceOverlayId
}

func (p *pathUnpackingParam) getTargetOverlayId() da.Index {
	return p.targetOverlayId
}

func (p *pathUnpackingParam) getLevel() uint8 {
	return p.level
}

func (p *pathUnpackingParam) getIndex() int {
	return p.index
}

type pathUnpackingResult struct {
	index       int
	edgeIdsPath []da.Index
}

func newPathUnpackingResult(index int, edgeIdsPath []da.Index) pathUnpackingResult {
	return pathUnpackingResult{index: index, edgeIdsPath: edgeIdsPath}
}

func (p *pathUnpackingResult) getIndex() int {
	return p.index
}

func (p *pathUnpackingResult) getEdgeIdsPath() []da.Index {
	return p.edgeIdsPath
}
