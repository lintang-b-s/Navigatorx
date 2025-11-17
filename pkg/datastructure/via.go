package datastructure


type ViaVertex struct {
	v           Index
	originalVId Index
	entryId     Index
	exitId      Index
}

func NewViaVertex(v, entryId, exitId, originalVId Index) ViaVertex {
	return ViaVertex{v: v, entryId: entryId, exitId: exitId, originalVId: originalVId}
}

func (v ViaVertex) GetEntryId() Index {
	return v.entryId
}

func (v ViaVertex) GetExitId() Index {
	return v.exitId
}

func (v ViaVertex) GetVId() Index {
	return v.v
}

func (v ViaVertex) GetOriginalVId() Index {
	return v.originalVId
}
