package datastructure

type ViaVertex struct {
	v                         Index
	originalVId               Index
	entryId                   Index
	exitId                    Index
	overlay                   bool
	plv, lv, approxSharedDist float64
}

func NewViaVertex(v, entryId, exitId, originalVId Index, overlay bool) ViaVertex {
	return ViaVertex{v: v, entryId: entryId, exitId: exitId, originalVId: originalVId, overlay: overlay}
}

func NewEmptyViaVertex() ViaVertex {
	return ViaVertex{v: INVALID_VERTEX_ID}
}

func IsEmptyViaVertex(v ViaVertex) bool {
	return v.v == INVALID_VERTEX_ID
}

func (v *ViaVertex) GetEntryId() Index {
	return v.entryId
}

func (v *ViaVertex) GetExitId() Index {
	return v.exitId
}

func (v *ViaVertex) GetVId() Index {
	return v.v
}

func (v *ViaVertex) GetOriginalVId() Index {
	return v.originalVId
}

func (v *ViaVertex) IsOverlay() bool {
	return v.overlay
}

func (v *ViaVertex) SetPlateau(plv float64) {
	v.plv = plv
}

func (v *ViaVertex) SetCost(lv float64) {
	v.lv = lv
}

func (v *ViaVertex) SetApproxSharedDist(approxSigma float64) {
	v.approxSharedDist = approxSigma
}

func (v *ViaVertex) GetPlateau() float64 {
	return v.plv
}

func (v *ViaVertex) GetCost() float64 {
	return v.lv
}

func (v *ViaVertex) GetApproxSharedDist() float64 {
	return v.approxSharedDist
}

func (v *ViaVertex) GetApproxObjectiveValue() float64 {
	return 2*v.GetCost() + v.GetApproxSharedDist() - v.GetPlateau()
}


