package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type pathUnpackingParam struct {
	sourceOverlayId  da.Index
	targetOverlayId  da.Index
	level            uint8
	unpackedEdgePath *[]da.OutEdge
}

func NewPathUnpackingParam(sourceOverlayId, targetOverlayId da.Index, level uint8, unpackedEdgePath *[]da.OutEdge) pathUnpackingParam {
	return pathUnpackingParam{sourceOverlayId: sourceOverlayId, targetOverlayId: targetOverlayId, level: level, unpackedEdgePath: unpackedEdgePath}
}

func (p pathUnpackingParam) getSourceOverlayId() da.Index {
	return p.sourceOverlayId
}

func (p pathUnpackingParam) getTargetOverlayId() da.Index {
	return p.targetOverlayId
}

func (p pathUnpackingParam) getLevel() uint8 {
	return p.level
}

func (p pathUnpackingParam) getUnpackedEdgePath() *[]da.OutEdge {
	return p.unpackedEdgePath
}
