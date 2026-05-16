package guidance

import (
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/stretchr/testify/assert"
)

type mockGraph struct {
	edgeGeometries map[da.Index][]da.Coordinate
}

func (m *mockGraph) GetTailOfOutedge(e da.Index) da.Index                  { return 0 }
func (m *mockGraph) GetVertex(u da.Index) da.Vertex                        { return da.Vertex{} }
func (m *mockGraph) GetVertexCoordinate(u da.Index) da.Coordinate          { return da.Coordinate{} }
func (m *mockGraph) GetOutEdge(eId da.Index) *da.OutEdge                   { return nil }
func (m *mockGraph) IsRoundabout(edgeId da.Index) bool                     { return false }
func (m *mockGraph) GetStreetName(edgeId da.Index) string                  { return "" }
func (m *mockGraph) GetEdgeGeometry(edgeID da.Index) []da.Coordinate       { return m.edgeGeometries[edgeID] }
func (m *mockGraph) ForOutEdgeIdsOf(u da.Index, handle func(eId da.Index)) {}
func (m *mockGraph) GetRoadClass(edgeId da.Index) string                   { return "" }
func (m *mockGraph) GetRoadClassLink(edgeId da.Index) string               { return "" }
func (m *mockGraph) GetStreetDirection(edgeId da.Index) [2]bool            { return [2]bool{} }
func (m *mockGraph) GetRoadLanes(edgeId da.Index) uint8                    { return 0 }
func (m *mockGraph) ForInEdgeIdsOf(v da.Index, handle func(eId da.Index))  {}
func (m *mockGraph) GetHeadFromInEdge(entryId da.Index) da.Index           { return 0 }
func (m *mockGraph) IsTrafficLight(vertexId da.Index) bool                 { return false }
func (m *mockGraph) IsDummyOutEdge(eId da.Index) bool                      { return false }
func (m *mockGraph) GetHeadOfOutEdge(e da.Index) da.Index                  { return 0 }
func (m *mockGraph) GetTailOfInedge(e da.Index) da.Index                   { return 0 }
func (m *mockGraph) GetOutEdgeLength(e da.Index) float64                   { return 0 }
func (m *mockGraph) GetOutEdgeWeight(e da.Index) float64                   { return 0 }
func (m *mockGraph) IsStreetBidirectional(edgeId da.Index) bool            { return false }
func (m *mockGraph) GetExitIdOfInEdge(e da.Index) da.Index                 { return 0 }
func (m *mockGraph) GetOsmWayId(edgeId da.Index) int64                     { return 0 }
func (m *mockGraph) GetStrFromId(stNameId uint32) string                   { return "" }
func (m *mockGraph) GetStreetNameId(edgeId da.Index) uint32                { return 0 }
func (m *mockGraph) IsCurved(edgeId da.Index) bool                         { return false }

type mockEngine struct {
	speeds map[da.Index]float64
}

func (m *mockEngine) GetGraph() *da.Graph                                { return nil }
func (m *mockEngine) PathExists(u, v da.Index) bool                      { return false }
func (m *mockEngine) GetWeight(eId da.Index, outEdge bool) float64       { return 0 }
func (m *mockEngine) GetSegmentSpeed(eId da.Index, outEdge bool) float64 { return m.speeds[eId] }
func (m *mockEngine) GetWeightFromLength(eId da.Index, eLength float64, outEdge bool) float64 {
	return 0
}
func (m *mockEngine) IsDummyOutEdge(eId da.Index) bool { return false }
func (m *mockEngine) IsDummyInEdge(eId da.Index) bool  { return false }

func TestBuildEdgeGeomOffsetFromSimplifiedGeometry(t *testing.T) {
	p0 := da.NewCoordinate(0, 0)
	p1 := da.NewCoordinate(0, 1)
	p2 := da.NewCoordinate(0, 2)
	p3 := da.NewCoordinate(0, 3)
	p4 := da.NewCoordinate(0, 4)
	simplifiedGeometry := da.Coordinates{p0, p1, p2, p3, p4}

	tests := []struct {
		name           string
		edgeIDs        []da.Index
		edgeGeometries map[da.Index][]da.Coordinate
		want           []da.Index
	}{
		{
			name:    "exact matches",
			edgeIDs: []da.Index{0, 1, 2},
			edgeGeometries: map[da.Index][]da.Coordinate{
				0: {p0},
				1: {p2},
				2: {p4},
			},
			want: []da.Index{0, 0, 0},
		},
		{
			name:    "uses edge length minus one for next offset",
			edgeIDs: []da.Index{10, 11, 12},
			edgeGeometries: map[da.Index][]da.Coordinate{
				10: {p0, p1},
				11: {p2},
				12: {p2, p3, p4},
			},
			want: []da.Index{0, 1, 1},
		},
		{
			name:           "empty inputs",
			edgeIDs:        []da.Index{},
			edgeGeometries: map[da.Index][]da.Coordinate{},
			want:           []da.Index{},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			db := &DirectionBuilder{
				graph:  &mockGraph{edgeGeometries: tt.edgeGeometries},
				engine: &mockEngine{speeds: map[da.Index]float64{}},
			}
			got := db.buildEdgeGeomOffsetFromGeometry(tt.edgeIDs, simplifiedGeometry)
			assert.Equal(t, tt.want, got)
		})
	}
}

func TestBuildSimplifiedAnnotation(t *testing.T) {
	// Mirrors updateState() behavior: each edge contributes eGeom[:len(eGeom)-1].
	geomWithoutLastPoint := da.Coordinates{
		da.NewCoordinate(-7.00000, 110.00000),
		da.NewCoordinate(-7.00000, 110.00010),
		da.NewCoordinate(-7.00000, 110.00020),
		da.NewCoordinate(-7.00000, 110.00030),
	}
	lastEdgeLastPoint := da.NewCoordinate(-7.00000, 110.00040)

	graph := &mockGraph{
		edgeGeometries: map[da.Index][]da.Coordinate{
			1: {geomWithoutLastPoint[0], geomWithoutLastPoint[1], geomWithoutLastPoint[2]},
			2: {geomWithoutLastPoint[2], geomWithoutLastPoint[3], lastEdgeLastPoint},
		},
	}
	engine := &mockEngine{speeds: map[da.Index]float64{1: 10, 2: 20}}
	db := &DirectionBuilder{graph: graph, engine: engine}

	t.Run("empty geometry", func(t *testing.T) {
		ann := db.buildSimplifiedAnnotation([]da.Index{}, da.Coordinates{})
		assert.Equal(t, da.Coordinates{}, ann.GetGeometry())
		assert.Equal(t, []float64{}, ann.GetDuration())
		assert.Equal(t, []float64{}, ann.GetDistance())
		assert.Equal(t, []da.Index{}, ann.GetEdgeGeomOffset())
	})

	t.Run("single point geometry", func(t *testing.T) {
		single := da.Coordinates{da.NewCoordinate(-7.0, 110.0)}
		ann := db.buildSimplifiedAnnotation([]da.Index{}, single)
		assert.Equal(t, single, ann.GetGeometry())
		assert.Equal(t, []float64{}, ann.GetDuration())
		assert.Equal(t, []float64{}, ann.GetDistance())
		assert.Equal(t, []da.Index{}, ann.GetEdgeGeomOffset())
	})

	t.Run("appends last edge final point and builds metrics", func(t *testing.T) {
		ann := db.buildSimplifiedAnnotation([]da.Index{1, 2}, geomWithoutLastPoint)
		wantGeometry := append(da.Coordinates{}, geomWithoutLastPoint...)
		wantGeometry = append(wantGeometry, lastEdgeLastPoint)
		assert.Equal(t, wantGeometry, ann.GetGeometry())
		assert.Len(t, ann.GetDistance(), len(wantGeometry)-1)
		assert.Len(t, ann.GetDuration(), len(wantGeometry)-1)
		assert.Greater(t, ann.GetDistance()[0], 0.0)
		assert.Greater(t, ann.GetDuration()[0], 0.0)
		assert.Equal(t, []da.Index{0, 2}, ann.GetEdgeGeomOffset())
	})

	t.Run("does not panic when last edge geometry is empty", func(t *testing.T) {
		graph.edgeGeometries[3] = []da.Coordinate{}
		ann := db.buildSimplifiedAnnotation([]da.Index{3}, geomWithoutLastPoint)
		assert.Equal(t, geomWithoutLastPoint, ann.GetGeometry())
		assert.Len(t, ann.GetDistance(), len(geomWithoutLastPoint)-1)
		assert.Len(t, ann.GetDuration(), len(geomWithoutLastPoint)-1)
		assert.Equal(t, []da.Index{0}, ann.GetEdgeGeomOffset())
	})
}
