package datastructure

type ViaVertex struct {
	v           Index
	originalVId Index
	entryId     Index
	exitId      Index
	overlay     bool
}

func NewViaVertex(v, entryId, exitId, originalVId Index, overlay bool) ViaVertex {
	return ViaVertex{v: v, entryId: entryId, exitId: exitId, originalVId: originalVId, overlay: overlay}
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

func (v ViaVertex) IsOverlay()bool {
	return v.overlay
}