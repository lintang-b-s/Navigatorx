package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

func isBitOn(u da.Index, i int) bool {
	return u&da.Index(uint32(1)<<i) != 0
}

func offBit(u da.Index, i int) da.Index {
	return u & ^da.Index(uint32(1)<<i)
}

func onBit(u da.Index, i int) da.Index {
	return u | da.Index(uint32(1)<<i)
}

func (crp *CRPRoutingEngine[W]) offsetOverlay(v da.Index) da.Index {
	return v + da.Index(crp.graph.NumberOfVertices())
}

func (crp *CRPRoutingEngine[W]) adjustoffsetOverlay(v da.Index) da.Index {
	return v - da.Index(crp.graph.NumberOfVertices())
}
