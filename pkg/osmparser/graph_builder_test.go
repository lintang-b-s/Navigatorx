package osmparser

// NOTE: kebanyakan unit tests pada package osmparser digenerate oleh gemini 3 flash

import (
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

func TestBuildGraphSimple(t *testing.T) {
	p := NewOSMParserV2()

	// Create some mocked edges
	scannedEdges := []Edge{
		NewEdge(0, 1, 10.0, 100.0, 1),
		NewEdge(1, 2, 15.0, 150.0, 2),
	}

	// Mock wayNodeMap and nodeToOsmId
	p.wayNodeMap[1] = nodeWithCoord{tipe: JUNCTION_NODE, coord: NewNodeCoord(0, 0)}
	p.wayNodeMap[2] = nodeWithCoord{tipe: JUNCTION_NODE, coord: NewNodeCoord(0.1, 0.1)}
	p.wayNodeMap[3] = nodeWithCoord{tipe: JUNCTION_NODE, coord: NewNodeCoord(0.2, 0.2)}

	p.nodeToOsmId[0] = 1
	p.nodeToOsmId[1] = 2
	p.nodeToOsmId[2] = 3

	numV := uint32(3)
	graphStorage := da.NewGraphStorage(da.DEFAULT_BIT_SIZE_OSM_WAY_ID)

	graph, edgeInfoIds := p.BuildGraph(scannedEdges, graphStorage, numV, true)

	if graph == nil {
		t.Fatal("BuildGraph returned nil")
	}

	if graph.NumberOfVertices() != int(numV) {
		t.Errorf("expected %d vertices, got %d", numV, graph.NumberOfVertices())
	}

	// Check if edges are added correctly
	if graph.NumberOfEdges() < 2 {
		t.Errorf("expected at least 2 edges, got %d", graph.NumberOfEdges())
	}

	if len(edgeInfoIds) != int(numV) {
		t.Errorf("expected edgeInfoIds length %d, got %d", numV, len(edgeInfoIds))
	}
}

func TestBuildGraphWithTurnRestrictions(t *testing.T) {
	p := NewOSMParserV2()

	// Way 1: 1 -> 2
	// Way 2: 2 -> 3
	// Restriction: No left turn from Way 1 to Way 2 via node 2

	way1Id := int64(100)
	way2Id := int64(200)

	p.ways[way1Id] = osmWay{nodes: []int64{1, 2}, graphNodes: []da.Index{0, 1}, oneWay: true}
	p.ways[way2Id] = osmWay{nodes: []int64{2, 3}, graphNodes: []da.Index{1, 2}, oneWay: true}

	p.restrictions[way1Id] = []restriction{
		{
			via:             1, // graph node index for node 2
			to:              way2Id,
			turnRestriction: NO_LEFT_TURN,
			isWay:           false,
		},
	}

	scannedEdges := []Edge{
		NewEdge(0, 1, 10.0, 100.0, 1),
		NewEdge(1, 2, 15.0, 150.0, 2),
	}

	p.wayNodeMap[1] = nodeWithCoord{tipe: JUNCTION_NODE, coord: NewNodeCoord(-7.795870, 110.365442)}
	p.wayNodeMap[2] = nodeWithCoord{tipe: JUNCTION_NODE, coord: NewNodeCoord(-7.796351, 110.365372)}
	p.wayNodeMap[3] = nodeWithCoord{tipe: JUNCTION_NODE, coord: NewNodeCoord(-7.796497, 110.366863)} // left turn

	p.nodeToOsmId[0] = 1
	p.nodeToOsmId[1] = 2
	p.nodeToOsmId[2] = 3

	numV := uint32(3)
	graphStorage := da.NewGraphStorage(da.DEFAULT_BIT_SIZE_OSM_WAY_ID)

	graph, _ := p.BuildGraph(scannedEdges, graphStorage, numV, true)

	if graph == nil {
		t.Fatal("BuildGraph returned nil")
	}

	// Verify turn matrix at node 1 (node 2)
	// entryPoint from node 0 (Way 1), exitPoint to node 2 (Way 2)
	// The turn matrix should have NO_ENTRY for this turn

	// We need to find the entry and exit points
	turnResEntryPoint := -1
	turnResExitPoint := -1

	entryPointIdx := 0
	graph.ForInEdgeIdsOf(1, func(id da.Index) {
		tail := graph.GetTailOfInedge(id)
		head := graph.GetHeadOfInedge(id)

		if head == 1 && tail == 0 && !graph.IsParallelOutEdge(id) {
			turnResEntryPoint = int(entryPointIdx)
		}
		entryPointIdx++
	})

	graph.ForOutEdgesOf(1, da.Index(turnResEntryPoint), func(eId, head da.Index, weight, length float64, exitPoint, entryPoint da.Index, turnType pkg.TurnType, hwType pkg.OsmHighwayType) {
		if head == 2 {
			turnResExitPoint = int(exitPoint)
		}
	})

	if turnResEntryPoint == -1 || turnResExitPoint == -1 {
		t.Fatalf("could not find entry or exit point: entry=%d, exit=%d", turnResEntryPoint, turnResExitPoint)
	}

	graph.ForOutEdgesOf(1, da.Index(turnResEntryPoint), func(eId, head da.Index, weight, length float64, exitPoint, entryPoint da.Index, turnType pkg.TurnType, hwType pkg.OsmHighwayType) {
		if exitPoint == da.Index(turnResExitPoint) {
			if turnType != pkg.NO_ENTRY {
				t.Errorf("expected NO_ENTRY turn type, got %v", turnType)
			}
		}
	})
}
