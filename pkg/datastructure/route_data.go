package datastructure

type DirtyCell struct {
	cellId    Pv
	usedLevel int
}

func NewDirtyCell(cellId Pv, usedLevel int) DirtyCell {
	return DirtyCell{cellId: cellId, usedLevel: usedLevel}
}

func (d *DirtyCell) GetCellId() Pv {
	return d.cellId
}

func (d *DirtyCell) GetUsedLevel() int {
	return d.usedLevel
}

type PenaltiedEdge struct {
	edgeId Index
	out    bool
}

func NewPenaltiedEdge(id Index, out bool) PenaltiedEdge {
	return PenaltiedEdge{id, out}
}
