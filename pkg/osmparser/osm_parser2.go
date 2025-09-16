package osmparser

import (
	"context"
	"io"
	"log"
	"math"
	"os"
	"sort"
	"strconv"
	"strings"

	"github.com/lintang-b-s/navigatorx-crp/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx-crp/pkg/geo"
	"github.com/lintang-b-s/navigatorx-crp/pkg/util"
	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
)

type node struct {
	id    int64
	coord nodeCoord
}

type nodeCoord struct {
	lat float64
	lon float64
}

type restriction struct {
	via             uint32
	to              int64
	turnRestriction TurnRestriction
}

type osmWay struct {
	nodes  []uint32
	oneWay bool
}

type OsmParser struct {
	wayNodeMap        map[int64]NodeType
	relationMemberMap map[int64]struct{}
	acceptedNodeMap   map[int64]nodeCoord
	barrierNodes      map[int64]bool
	nodeTag           map[int64]map[int]int
	tagStringIdMap    util.IDMap
	nodeIDMap         map[int64]uint32
	maxNodeID         int64
	restrictions      map[int64][]restriction // wayId -> list of restrictions
	ways              map[int64]osmWay
}

type edge struct {
	from          uint32
	to            uint32
	weight        float64
	distance      float64
	edgeID        uint32
	bidirectional bool
}

func NewOSMParserV2() *OsmParser {
	return &OsmParser{
		wayNodeMap:        make(map[int64]NodeType),
		relationMemberMap: make(map[int64]struct{}),
		acceptedNodeMap:   make(map[int64]nodeCoord),
		barrierNodes:      make(map[int64]bool),
		nodeTag:           make(map[int64]map[int]int),
		tagStringIdMap:    util.NewIdMap(),
		nodeIDMap:         make(map[int64]uint32),
	}
}
func (o *OsmParser) GetTagStringIdMap() util.IDMap {
	return o.tagStringIdMap
}

var (
	skipHighway = map[string]struct{}{
		"footway":                struct{}{},
		"construction":           struct{}{},
		"cycleway":               struct{}{},
		"path":                   struct{}{},
		"pedestrian":             struct{}{},
		"busway":                 struct{}{},
		"steps":                  struct{}{},
		"bridleway":              struct{}{},
		"corridor":               struct{}{},
		"street_lamp":            struct{}{},
		"bus_stop":               struct{}{},
		"crossing":               struct{}{},
		"cyclist_waiting_aid":    struct{}{},
		"elevator":               struct{}{},
		"emergency_bay":          struct{}{},
		"emergency_access_point": struct{}{},
		"give_way":               struct{}{},
		"phone":                  struct{}{},
		"ladder":                 struct{}{},
		"milestone":              struct{}{},
		"passing_place":          struct{}{},
		"platform":               struct{}{},
		"speed_camera":           struct{}{},
		"track":                  struct{}{},
		"bus_guideway":           struct{}{},
		"speed_display":          struct{}{},
		"stop":                   struct{}{},
		"toll_gantry":            struct{}{},
		"traffic_mirror":         struct{}{},
		"traffic_signals":        struct{}{},
		"trailhead":              struct{}{},
	}

	// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Telenav
	acceptedHighway = map[string]struct{}{
		"motorway":         struct{}{},
		"motorway_link":    struct{}{},
		"trunk":            struct{}{},
		"trunk_link":       struct{}{},
		"primary":          struct{}{},
		"primary_link":     struct{}{},
		"secondary":        struct{}{},
		"secondary_link":   struct{}{},
		"residential":      struct{}{},
		"residential_link": struct{}{},
		"service":          struct{}{},
		"tertiary":         struct{}{},
		"tertiary_link":    struct{}{},
		"road":             struct{}{},
		"track":            struct{}{},
		"unclassified":     struct{}{},
		"undefined":        struct{}{},
		"unknown":          struct{}{},
		"living_street":    struct{}{},
		"private":          struct{}{},
		"motorroad":        struct{}{},
	}

	//https://wiki.openstreetmap.org/wiki/Key:barrier
	// for splitting street segment to 2 disconnected graph edge
	// if the access tag of the barrier node is != "no" , we dont split the segment
	// for example, at the barrier at the entrance to FMIPA UGM, where entry is only allowed after 16.00 WIB or before 8.00 wib. (https://www.openstreetmap.org/node/8837559088#map=19/-7.767125/110.375436&layers=N)

	acceptedBarrierType = map[string]struct{}{
		"bollard":    struct{}{},
		"swing_gate": struct{}{},

		"jersey_barrier": struct{}{},
		"lift_gate":      struct{}{},
		"block":          struct{}{},
		"gate":           struct{}{},
	}
)

func (p *OsmParser) Parse(mapFile string) *datastructure.Graph {

	f, err := os.Open(mapFile)

	if err != nil {
		log.Fatal(err)
	}
	defer f.Close()

	restrictions := make(map[int64][]struct {
		via             int64
		turnRestriction TurnRestriction
		to              int64
	})
	scanner := osmpbf.New(context.Background(), f, 0)
	// must not be parallel
	countWays := 0
	for scanner.Scan() {
		o := scanner.Object()

		tipe := o.ObjectID().Type()

		switch tipe {
		case osm.TypeWay:
			{
				way := o.(*osm.Way)
				if len(way.Nodes) < 2 {
					continue
				}

				if !acceptOsmWay(way) {
					continue
				}
				if (countWays+1)%50000 == 0 {
					log.Printf("reading openstreetmap ways: %d...", countWays+1)
				}
				countWays++

				for i, node := range way.Nodes {
					if _, ok := p.wayNodeMap[int64(node.ID)]; !ok {
						if i == 0 || i == len(way.Nodes)-1 {
							p.wayNodeMap[int64(node.ID)] = END_NODE
						} else {
							p.wayNodeMap[int64(node.ID)] = BETWEEN_NODE
						}
					} else {
						p.wayNodeMap[int64(node.ID)] = JUNCTION_NODE
					}
				}
			}
		case osm.TypeNode:
			{
			}
		case osm.TypeRelation:
			{
				relation := o.(*osm.Relation)
				for _, member := range relation.Members {
					if member.Type == osm.TypeRelation {
						continue
					}
				}

				if relation.Tags.Find("type") == "route" {
					for _, member := range relation.Members {
						p.relationMemberMap[member.Ref] = struct{}{}
					}
				}

				tagVal := relation.Tags.Find("restriction")
				if tagVal != "" {
					from := int64(0)
					via := int64(0)
					to := int64(0)
					// https://www.openstreetmap.org/api/0.6/relation/5710500
					for _, member := range relation.Members {
						if member.Role == "from" {
							from = member.Ref
						} else if member.Role == "to" {
							to = member.Ref
						} else {
							via = int64(member.Ref)
						}
					}
					rest := struct {
						via             int64
						turnRestriction TurnRestriction
						to              int64
					}{
						via:             via,
						to:              to,
						turnRestriction: parseTurnRestriction(tagVal),
					}
					restrictions[from] = append(restrictions[from], rest)
				}
			}
		}
	}
	scanner.Close()

	graphStorage := datastructure.NewGraphStorage()

	edgeSet := make(map[uint32]map[uint32]struct{})

	f.Seek(0, io.SeekStart)
	if err != nil {
		log.Fatal(err)
	}
	scanner = osmpbf.New(context.Background(), f, 0)
	//must not be parallel
	defer scanner.Close()

	scannedEdges := make([]edge, 0)
	p.ways = make(map[int64]osmWay, countWays)

	streetDirection := make(map[string][2]bool)
	countWays = 0
	countNodes := 0
	for scanner.Scan() {
		o := scanner.Object()

		tipe := o.ObjectID().Type()

		switch tipe {
		case osm.TypeWay:
			{
				way := o.(*osm.Way)
				if len(way.Nodes) < 2 {
					continue
				}

				if !acceptOsmWay(way) {
					continue
				}
				if (countWays+1)%50000 == 0 {
					log.Printf("processing openstreetmap ways: %d...", countWays+1)
				}
				countWays++

				p.processWay(way, graphStorage, streetDirection, edgeSet, &scannedEdges)

				wayExtraInfoData := wayExtraInfo{}
				okvf, okmvf, okvb, okmvb := getReversedOneWay(way)
				if val := way.Tags.Find("oneway"); val == "yes" || val == "-1" || okvf || okmvf || okvb || okmvb {
					wayExtraInfoData.oneWay = true
				}

				if way.Tags.Find("oneway") == "-1" || okvf || okmvf {
					// okvf / omvf = restricted/not allowed forward.
					wayExtraInfoData.forward = false

				} else {
					wayExtraInfoData.forward = true
				}
				wNodes := make([]uint32, 0, len(way.Nodes))
				for _, node := range way.Nodes {
					nodeID := p.nodeIDMap[int64(node.ID)]
					wNodes = append(wNodes, nodeID)
				}
				p.ways[int64(way.ID)] = osmWay{
					nodes:  wNodes,
					oneWay: wayExtraInfoData.oneWay,
				}
			}
		case osm.TypeNode:
			{

				if (countNodes+1)%50000 == 0 {
					log.Printf("processing openstreetmap nodes: %d...", countNodes+1)
				}
				countNodes++
				node := o.(*osm.Node)

				p.maxNodeID = max(p.maxNodeID, int64(node.ID))

				if _, ok := p.wayNodeMap[int64(node.ID)]; ok {
					p.acceptedNodeMap[int64(node.ID)] = nodeCoord{
						lat: node.Lat,
						lon: node.Lon,
					}
				}
				accessType := node.Tags.Find("access")
				barrierType := node.Tags.Find("barrier")

				if _, ok := acceptedBarrierType[barrierType]; ok && accessType == "no" && barrierType != "" {
					p.barrierNodes[int64(node.ID)] = true
				}

				for _, tag := range node.Tags {
					if strings.Contains(tag.Key, "created_by") ||
						strings.Contains(tag.Key, "source") ||
						strings.Contains(tag.Key, "note") ||
						strings.Contains(tag.Key, "fixme") {
						continue
					}
					tagID := p.tagStringIdMap.GetID(tag.Key)
					if _, ok := p.nodeTag[int64(node.ID)]; !ok {
						p.nodeTag[int64(node.ID)] = make(map[int]int)
					}
					p.nodeTag[int64(node.ID)][tagID] = p.tagStringIdMap.GetID(tag.Value)
					if strings.Contains(tag.Value, "traffic_signals") {
						p.nodeTag[int64(node.ID)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] = 1
					}
				}

			}
		case osm.TypeRelation:
			{

			}
		}
	}

	p.restrictions = make(map[int64][]restriction, len(restrictions))
	for key, val := range restrictions {
		savedRest := make([]restriction, len(val))
		for i := range val {
			savedRest[i] = restriction{
				via:             p.nodeIDMap[val[i].via],
				to:              val[i].to,
				turnRestriction: val[i].turnRestriction,
			}
		}
		p.restrictions[key] = savedRest
	}

	graph := p.BuildGraph(scannedEdges)
	return graph
}

func (p *OsmParser) BuildGraph(scannedEdges []edge) *datastructure.Graph {
	var (
		outEdges  [][]datastructure.OutEdge = make([][]datastructure.OutEdge, len(p.nodeIDMap))
		inEdges   [][]datastructure.InEdge  = make([][]datastructure.InEdge, len(p.nodeIDMap))
		inDegree  []uint8                   = make([]uint8, len(p.nodeIDMap))
		outDegree []uint8                   = make([]uint8, len(p.nodeIDMap))
		vertices  []datastructure.Vertex    = make([]datastructure.Vertex, len(p.nodeIDMap)+1)
	)

	for _, e := range scannedEdges {
		u := datastructure.Index(e.from)
		v := datastructure.Index(e.to)
		edgeID := datastructure.Index(e.edgeID)
		outEdges[u] = append(outEdges[u], datastructure.NewOutEdge(edgeID,
			v, e.weight, e.distance, uint8(len(inEdges[v]))))
		outDegree[u]++
		inEdges[v] = append(inEdges[v], datastructure.NewInEdge(edgeID,
			u, e.weight, e.distance, uint8(len(outEdges[u])-1)))
		inDegree[v]++

		uData := p.acceptedNodeMap[int64(u)]
		vertices[u] = datastructure.NewVertex(uData.lat, uData.lon, u)

		vData := p.acceptedNodeMap[int64(v)]
		vertices[v] = datastructure.NewVertex(vData.lat, vData.lon, v)

		if e.bidirectional {
			outEdges[v] = append(outEdges[v], datastructure.NewOutEdge(edgeID,
				u, e.weight, e.distance, uint8(len(inEdges[u]))))
			outDegree[v]++
			inEdges[u] = append(inEdges[u], datastructure.NewInEdge(edgeID,
				v, e.weight, e.distance, uint8(len(outEdges[v])-1)))
			inDegree[u]++
		}
	}

	turnMatrices := make([][]datastructure.TurnType, len(vertices)-1)
	// T_u[i*outDegree[u]+j] = turn type from inEdge i to outEdge j at vertex u

	// init turn matrices
	for i := 0; i < len(turnMatrices); i++ {
		turnMatrices[i] = make([]datastructure.TurnType, outDegree[i]*inDegree[i])

		for j := 0; j < len(turnMatrices[i]); j++ {
			turnMatrices[i][j] = datastructure.NONE
		}
	}

	for _, e := range scannedEdges {
		// dont allow u_turns at (u,v) -> (v,u)
		if !e.bidirectional {
			via := datastructure.Index(e.from)
			if inDegree[via] != 1 || outDegree[via] != 1 {
				to := datastructure.Index(e.to)

				entryId := -1
				exitId := -1
				for k := 0; k < len(outEdges[via]); k++ {
					if outEdges[via][k].GetHead() == to {
						exitId = k
						break
					}
				}

				for k := 0; k < len(inEdges[via]); k++ {
					if inEdges[via][k].GetTail() == to {
						entryId = k
						break
					}
				}
				if entryId == -1 || exitId == -1 {
					continue
				}
				turnMatrices[via][entryId*int(outDegree[via])+exitId] = datastructure.U_TURN
			}

			// to
			via = datastructure.Index(e.to)
			if inDegree[via] != 1 || outDegree[via] != 1 {
				to := datastructure.Index(e.from)

				entryId := -1
				exitId := -1
				for k := 0; k < len(outEdges[via]); k++ {
					if outEdges[via][k].GetHead() == to {
						exitId = k
						break
					}
				}

				for k := 0; k < len(inEdges[via]); k++ {
					if inEdges[via][k].GetTail() == to {
						entryId = k
						break
					}
				}

				if entryId == -1 || exitId == -1 {
					continue
				}
				turnMatrices[via][entryId*int(outDegree[via])+exitId] = datastructure.U_TURN
			}
		}

	}

	for wayID, way := range p.ways {
		fromNodes := way.nodes
		fromRestrictions := p.restrictions[wayID]
		for _, restriction := range fromRestrictions {
			_, isAcceptedNode := p.nodeIDMap[int64(restriction.via)]
			if wayID == int64(restriction.to) || !isAcceptedNode {
				continue
			}

			_, acceptedWay := p.ways[int64(restriction.to)]
			if !acceptedWay {
				continue
			}

			for i := 0; i < len(fromNodes); i++ {
				if fromNodes[i] == restriction.via {
					if i == 0 && way.oneWay {
						// no predecessor
						continue
					}

					var predecessor uint32
					if i == 0 {
						predecessor = fromNodes[i+1]
					} else {
						predecessor = fromNodes[i-1]
					}

					if predecessor == restriction.via {
						continue
					}
					successor := uint32(math.MaxUint32)
					toNodes := p.ways[int64(restriction.to)].nodes
					for j := 0; j < len(toNodes)-1; j++ {
						if toNodes[j] == restriction.via {
							if j == len(toNodes)-1 {
								successor = toNodes[j-1]
							} else {
								successor = toNodes[j+1]
							}
							break
						}
					}

					if successor != uint32(math.MaxUint32) && successor != restriction.via {

						from := datastructure.Index(predecessor)
						via := datastructure.Index(restriction.via)
						to := datastructure.Index(successor)

						entryID := datastructure.Index(math.MaxUint32)
						exitID := datastructure.Index(math.MaxUint32)

						for k := 0; k < len(inEdges[via]); k++ {
							if inEdges[via][k].GetTail() == from {
								entryID = datastructure.Index(k)
								break
							}
						}

						if entryID == datastructure.Index(math.MaxUint32) {
							continue
						}

						rowOffset := entryID * datastructure.Index(outDegree[via])
						for k := 0; k < len(outEdges[via]); k++ {
							if outEdges[via][k].GetHead() == to {
								exitID = datastructure.Index(k)
							}

							if restriction.turnRestriction == ONLY_LEFT_TURN || restriction.turnRestriction == ONLY_RIGHT_TURN ||
								restriction.turnRestriction == ONLY_STRAIGHT_ON {
								turnMatrices[via][rowOffset+datastructure.Index(k)] = datastructure.NO_ENTRY
							}
						}

						if exitID == datastructure.Index(math.MaxUint32) {
							continue
						}

						if rowOffset+exitID >= datastructure.Index(len(turnMatrices[via])) {
							continue
						}

						switch restriction.turnRestriction {
						case NO_LEFT_TURN:
							turnMatrices[via][rowOffset+exitID] = datastructure.NO_ENTRY
							break
						case NO_RIGHT_TURN:
							turnMatrices[via][rowOffset+exitID] = datastructure.NO_ENTRY
							break
						case NO_STRAIGHT_ON:
							turnMatrices[via][rowOffset+exitID] = datastructure.NO_ENTRY
							break
						case NO_U_TURN:
							turnMatrices[via][rowOffset+exitID] = datastructure.NO_ENTRY
							break
						case NO_ENTRY:
							turnMatrices[via][rowOffset+exitID] = datastructure.NO_ENTRY
							break
						case ONLY_LEFT_TURN:
							turnMatrices[via][rowOffset+exitID] = datastructure.LEFT_TURN
							break
						case ONLY_RIGHT_TURN:
							turnMatrices[via][rowOffset+exitID] = datastructure.RIGHT_TURN
							break
						case ONLY_STRAIGHT_ON:
							turnMatrices[via][rowOffset+exitID] = datastructure.STRAIGHT_ON
							break
						default:
							turnMatrices[via][rowOffset+exitID] = datastructure.NONE
							break
						}

					}
					break
				}
			}
		}
	}

	matrices := make([]datastructure.TurnType, 0)
	matrixOffset := 0

	for v := 0; v < len(vertices)-1; v++ {
		// sort inEdges
		rowScore := make([]datastructure.Index, inDegree[v])
		rowHist := make([][]uint32, inDegree[v])
		for k := 0; k < len(rowHist); k++ {
			rowHist[k] = make([]uint32, NUM_TURN_TYPES)
		}

		for i := uint8(0); i < inDegree[v]; i++ {
			for j := i * outDegree[v]; j < (i+1)*outDegree[v]; j++ {
				if datastructure.Index(turnMatrices[v][j]) != datastructure.Index(NONE) {
					rowScore[i] += 1
				}

				t := turnMatrices[v][j]
				rowHist[i][t]++

			}
		}

		sortOrder := make([]datastructure.Index, inDegree[v])
		for i := uint8(0); i < inDegree[v]; i++ {
			sortOrder[i] = datastructure.Index(i)
		}

		// sort sortOrder by rowScore descending, then rowHist ascending
		sort.Slice(sortOrder, func(i, j int) bool {
			if rowScore[sortOrder[i]] > rowScore[sortOrder[j]] {
				return true
			}

			for tt := NUM_TURN_TYPES - 1; tt >= 0; tt -= 1 {
				if rowHist[i][tt] != rowHist[j][tt] {
					return rowHist[i][tt] < rowHist[j][tt]
				}
			}
			return i < j
		})

		sortedMatrix := make([]datastructure.TurnType, inDegree[v]*outDegree[v])
		sortedInEdge := make([]datastructure.InEdge, len(inEdges[v]))
		for i := uint8(0); i < inDegree[v]; i++ {
			k := i * outDegree[v]
			for j := sortOrder[i] * datastructure.Index(outDegree[v]); j < (sortOrder[i]+1)*datastructure.Index(outDegree[v]); j++ {
				sortedMatrix[k] = turnMatrices[v][j]
				k += 1
			}
			sortedInEdge[i] = inEdges[v][sortOrder[i]]
			for _, outEdge := range outEdges[inEdges[v][sortOrder[i]].GetTail()] {
				if outEdge.GetHead() == datastructure.Index(v) {
					outEdge.SetEntryPoint(i)
					break
				}
			}
		}

		turnMatrices[v] = sortedMatrix
		inEdges[v] = sortedInEdge

		// sort OutEdges
		columnScore := make([]datastructure.Index, outDegree[v])
		colHist := make([][]uint32, outDegree[v])
		for k := 0; k < len(colHist); k++ {
			colHist[k] = make([]uint32, NUM_TURN_TYPES)
		}

		for j := uint8(0); j < outDegree[v]; j++ {
			for i := uint8(0); i < inDegree[v]; i++ {
				offset := i*outDegree[v] + j
				t := turnMatrices[v][offset]
				if t != datastructure.NONE {
					columnScore[j] |= 1 << i
				}

				colHist[j][t]++
			}
		}

		sortOrder = make([]datastructure.Index, outDegree[v])
		for j := uint8(0); j < outDegree[v]; j++ {
			sortOrder[j] = datastructure.Index(j)
		}

		sort.Slice(sortOrder, func(i, j int) bool {
			firstDiff := columnScore[i] ^ columnScore[j]
			for k := uint8(0); k < inDegree[v]; k++ {
				if firstDiff&(1<<k) != 0 {
					return columnScore[i]&(1<<k) != 0
				}
			}

			for tt := NUM_TURN_TYPES - 1; tt >= 0; tt -= 1 {
				if colHist[i][tt] != colHist[j][tt] {
					return colHist[i][tt] < colHist[j][tt]
				}
			}
			return i < j
		})

		sortedOutEdges := make([]datastructure.OutEdge, len(outEdges[v]))
		finalMatrix := make([]datastructure.TurnType, inDegree[v]*outDegree[v])
		for j := uint8(0); j < outDegree[v]; j++ {
			for i := uint8(0); i < inDegree[v]; i++ {
				finalMatrix[j+i*outDegree[v]] = turnMatrices[v][sortOrder[j]+
					datastructure.Index(i)*datastructure.Index(outDegree[v])]
			}

			sortedOutEdges[j] = outEdges[v][sortOrder[j]]
			for _, inEdge := range inEdges[outEdges[v][sortOrder[j]].GetHead()] {
				if inEdge.GetTail() == datastructure.Index(v) {
					inEdge.SetExitPoint(j)
					break
				}

			}
		}

		turnMatrices[v] = finalMatrix
		outEdges[v] = sortedOutEdges

		vertices[v].SetTurnTablePtr(datastructure.Index(matrixOffset))
		for i := 0; i < len(turnMatrices[v]); i++ {
			matrices = append(matrices, turnMatrices[v][i])
		}

		matrixOffset += len(turnMatrices[v])
	}

	outEdgeOffset := datastructure.Index(0)
	inEdgeOffset := datastructure.Index(0)

	for i := 0; i < len(vertices)-1; i++ {
		vertices[i].SetTurnTablePtr(vertices[i].GetTurnTablePtr())
		vertices[i].SetFirstOut(outEdgeOffset)
		vertices[i].SetFirstIn(inEdgeOffset)
		outEdgeOffset += datastructure.Index(len(outEdges[i]))
		inEdgeOffset += datastructure.Index(len(inEdges[i]))
	}

	vertices[len(vertices)-1] = datastructure.NewVertex(0, 0, datastructure.Index(len(vertices)-1))
	vertices[len(vertices)-1].SetFirstOut(outEdgeOffset)
	vertices[len(vertices)-1].SetFirstIn(inEdgeOffset)

	flattenOutEdges := flatten(outEdges)
	flattenInEdges := flatten(inEdges)
	graph := datastructure.NewGraph(vertices, flattenOutEdges, flattenInEdges, matrices)

	return graph
}

func flatten[T any](container [][]T) []T {
	finalSize := 0
	for _, part := range container {
		finalSize += len(part)
	}

	result := make([]T, finalSize)
	idx := 0
	for _, part := range container {
		for _, elem := range part {
			result[idx] = elem
			idx++
		}
	}
	return result
}

type wayExtraInfo struct {
	oneWay  bool
	forward bool
}

func (p *OsmParser) processWay(way *osm.Way, graphStorage *datastructure.GraphStorage,
	streetDirection map[string][2]bool, edgeSet map[uint32]map[uint32]struct{}, scannedEdges *[]edge) error {
	tempMap := make(map[string]string)
	name := way.Tags.Find("name")

	tempMap[STREET_NAME] = name

	refName := way.Tags.Find("ref")
	tempMap[STREET_REF] = refName

	maxSpeed := 0.0
	highwayTypeSpeed := 0.0

	wayExtraInfoData := wayExtraInfo{}
	okvf, okmvf, okvb, okmvb := getReversedOneWay(way)
	if val := way.Tags.Find("oneway"); val == "yes" || val == "-1" || okvf || okmvf || okvb || okmvb {
		wayExtraInfoData.oneWay = true
	}

	if way.Tags.Find("oneway") == "-1" || okvf || okmvf {
		// okvf / omvf = restricted/not allowed forward.
		wayExtraInfoData.forward = false

	} else {
		wayExtraInfoData.forward = true
	}

	if wayExtraInfoData.oneWay {
		if wayExtraInfoData.forward {
			streetDirection[name] = [2]bool{true, false} // {forward, backward}
		} else {
			streetDirection[name] = [2]bool{false, true}
		}
	} else {
		streetDirection[name] = [2]bool{true, true}
	}

	for _, tag := range way.Tags {
		switch tag.Key {
		case "junction":
			{
				tempMap[JUNCTION] = tag.Value
			}
		case "highway":
			{
				highwayTypeSpeed = roadTypeMaxSpeed2(tag.Value)

				if strings.Contains(tag.Value, "link") {
					tempMap[ROAD_CLASS_LINK] = tag.Value
				} else {
					tempMap[ROAD_CLASS] = tag.Value
				}
			}
		case "lanes":
			{
				tempMap[LANES] = tag.Value
			}
		case "maxspeed":
			{
				if strings.Contains(tag.Value, "mph") {

					currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " mph", "", -1), 64)
					if err != nil {
						return err
					}
					maxSpeed = currSpeed * 1.60934
				} else if strings.Contains(tag.Value, "km/h") {
					currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " km/h", "", -1), 64)
					if err != nil {
						return err
					}
					maxSpeed = currSpeed
				} else if strings.Contains(tag.Value, "knots") {
					currSpeed, err := strconv.ParseFloat(strings.Replace(tag.Value, " knots", "", -1), 64)
					if err != nil {
						return err
					}
					maxSpeed = currSpeed * 1.852
				} else {
					// without unit
					// dont use this
				}
			}

		}

	}

	if maxSpeed == 0 {
		maxSpeed = highwayTypeSpeed
	}
	if maxSpeed == 0 {
		maxSpeed = 30
	}

	waySegment := []node{}
	for _, wayNode := range way.Nodes {
		nodeCoord := p.acceptedNodeMap[int64(wayNode.ID)]
		nodeData := node{
			id:    int64(wayNode.ID),
			coord: nodeCoord,
		}
		if p.isJunctionNode(int64(nodeData.id)) {

			waySegment = append(waySegment, nodeData)
			p.processSegment(waySegment, tempMap, maxSpeed, graphStorage, wayExtraInfoData,
				edgeSet, scannedEdges)
			waySegment = []node{}

			waySegment = append(waySegment, nodeData)

		} else {
			waySegment = append(waySegment, nodeData)
		}

	}
	if len(waySegment) > 1 {
		p.processSegment(waySegment, tempMap, maxSpeed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges)
	}

	return nil
}

func isRestricted(value string) bool {
	if value == "no" || value == "restricted" {
		return true
	}
	return false
}

func getReversedOneWay(way *osm.Way) (bool, bool, bool, bool) {
	vehicleForward := way.Tags.Find("vehicle:forward")
	motorVehicleForward := way.Tags.Find("motor_vehicle:forward")
	vehicleBackward := way.Tags.Find("vehicle:backward")
	motorVehicleBackward := way.Tags.Find("motor_vehicle:backward")
	return isRestricted(vehicleForward), isRestricted(motorVehicleForward), isRestricted(vehicleBackward), isRestricted(motorVehicleBackward)
}

func (p *OsmParser) processSegment(segment []node, tempMap map[string]string, speed float64, graphStorage *datastructure.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[uint32]map[uint32]struct{}, scannedEdges *[]edge) {

	if len(segment) == 2 && segment[0].id == segment[1].id {
		// skip
		return
	} else if len(segment) > 2 && segment[0].id == segment[len(segment)-1].id {
		// loop
		p.processSegment2(segment[0:len(segment)-1], tempMap, speed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges)
		p.processSegment2(segment[len(segment)-2:], tempMap, speed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges)
	} else {
		p.processSegment2(segment, tempMap, speed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges)
	}
}

func (p *OsmParser) processSegment2(segment []node, tempMap map[string]string, speed float64, graphStorage *datastructure.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[uint32]map[uint32]struct{}, scannedEdges *[]edge) {
	waySegment := []node{}
	for i := 0; i < len(segment); i++ {
		nodeData := segment[i]
		if _, ok := p.barrierNodes[int64(nodeData.id)]; ok {

			if len(waySegment) != 0 {
				// if current node is a barrier
				// add the barrier node and process the segment (add edge)
				waySegment = append(waySegment, nodeData)
				p.addEdge(waySegment, tempMap, speed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges)
				waySegment = []node{}
			}
			// copy the barrier node but with different id so that previous edge (with barrier) not connected with the new edge

			nodeData = p.copyNode(nodeData)
			waySegment = append(waySegment, nodeData)

		} else {
			waySegment = append(waySegment, nodeData)
		}
	}
	if len(waySegment) > 1 {
		p.addEdge(waySegment, tempMap, speed, graphStorage, wayExtraInfoData, edgeSet, scannedEdges)
	}
}

func (p *OsmParser) copyNode(nodeData node) node {
	// use the same coordinate but different id & and the newID is not used
	newMaxID := p.maxNodeID + 1
	p.acceptedNodeMap[newMaxID] = nodeCoord{
		lat: nodeData.coord.lat,
		lon: nodeData.coord.lon,
	}
	p.maxNodeID++
	return node{
		id: newMaxID,
		coord: nodeCoord{
			lat: nodeData.coord.lat,
			lon: nodeData.coord.lon,
		},
	}
}

func (p *OsmParser) addEdge(segment []node, tempMap map[string]string, speed float64, graphStorage *datastructure.GraphStorage,
	wayExtraInfoData wayExtraInfo, edgeSet map[uint32]map[uint32]struct{}, scannedEdges *[]edge) {
	from := segment[0]

	to := segment[len(segment)-1]

	if from == to {
		return
	}

	if _, ok := p.nodeIDMap[from.id]; !ok {
		p.nodeIDMap[from.id] = uint32(len(p.nodeIDMap))
	}
	if _, ok := p.nodeIDMap[to.id]; !ok {
		p.nodeIDMap[to.id] = uint32(len(p.nodeIDMap))
	}

	edgePoints := []datastructure.Coordinate{}
	distance := 0.0
	for i := 0; i < len(segment); i++ {
		if i != 0 && i != len(segment)-1 && p.nodeTag[int64(segment[i].id)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] == 1 {

			distToFromNode := geo.CalculateHaversineDistance(from.coord.lat, from.coord.lon, segment[i].coord.lat, segment[i].coord.lon)
			distToToNode := geo.CalculateHaversineDistance(to.coord.lat, to.coord.lon, segment[i].coord.lat, segment[i].coord.lon)
			if distToFromNode < distToToNode {
				if _, ok := p.nodeTag[int64(segment[0].id)]; !ok {
					p.nodeTag[int64(segment[0].id)] = make(map[int]int)
				}
				p.nodeTag[int64(segment[0].id)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] = 1
			} else {
				if _, ok := p.nodeTag[int64(segment[len(segment)-1].id)]; !ok {
					p.nodeTag[int64(segment[len(segment)-1].id)] = make(map[int]int)
				}
				p.nodeTag[int64(segment[len(segment)-1].id)][p.tagStringIdMap.GetID(TRAFFIC_LIGHT)] = 1
			}
		}
		edgePoints = append(edgePoints, datastructure.Coordinate{
			Lat: segment[i].coord.lat,
			Lon: segment[i].coord.lon,
		})
		if i > 0 {
			distance += geo.CalculateHaversineDistance(segment[i-1].coord.lat, segment[i-1].coord.lon, segment[i].coord.lat, segment[i].coord.lon)
		}
	}

	edgePoints = geo.RamerDouglasPeucker(edgePoints) // simplify edge geometry

	isRoundabout := false
	if val, ok := tempMap[JUNCTION]; ok {
		if val == "roundabout" {
			isRoundabout = true
		}
		if val == "circular" {
			isRoundabout = true
		}
	}

	distanceInMeter := distance * 1000
	etaWeight := distanceInMeter / (speed * 1000 / 60) // in minutes

	lanes, err := strconv.Atoi(tempMap[LANES])
	if err != nil {
		lanes = 1 // assume
	}

	if _, ok := edgeSet[p.nodeIDMap[from.id]]; !ok {
		edgeSet[p.nodeIDMap[from.id]] = make(map[uint32]struct{})
	}
	if _, ok := edgeSet[p.nodeIDMap[to.id]]; !ok {
		edgeSet[p.nodeIDMap[to.id]] = make(map[uint32]struct{})
	}

	if wayExtraInfoData.oneWay {
		if wayExtraInfoData.forward {

			if _, ok := edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]]; ok {
				return
			}

			edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]] = struct{}{}

			startPointsIndex := len(graphStorage.GlobalPoints)

			graphStorage.AppendGlobalPoints(edgePoints)
			endPointsIndex := len(graphStorage.GlobalPoints)

			graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
				uint8(lanes),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
				uint32(startPointsIndex), uint32(endPointsIndex),
			))

			graphStorage.SetRoundabout(uint32(len(*scannedEdges)), isRoundabout)

			*scannedEdges = append(*scannedEdges, edge{
				from:          uint32(p.nodeIDMap[from.id]),
				to:            uint32(p.nodeIDMap[to.id]),
				weight:        etaWeight,
				distance:      distanceInMeter,
				edgeID:        uint32(len(*scannedEdges)),
				bidirectional: false,
			})

		} else {

			if _, ok := edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]]; ok {
				return
			}
			edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]] = struct{}{}

			// graphStorage.SetRoundabout(int32(len(graphStorage.EdgeStorage)), isRoundabout)

			edgePoints = util.ReverseG(edgePoints)

			startPointsIndex := len(graphStorage.GlobalPoints)

			graphStorage.AppendGlobalPoints(edgePoints)
			endPointsIndex := len(graphStorage.GlobalPoints)

			graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
				uint8(lanes),
				uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
				uint32(startPointsIndex), uint32(endPointsIndex),
			),
			)

			graphStorage.SetRoundabout(uint32(len(*scannedEdges)), isRoundabout)

			*scannedEdges = append(*scannedEdges, edge{
				from:          uint32(p.nodeIDMap[to.id]),
				to:            uint32(p.nodeIDMap[from.id]),
				weight:        etaWeight,
				distance:      distanceInMeter,
				edgeID:        uint32(len(*scannedEdges)),
				bidirectional: false,
			})
		}
	} else {
		if _, ok := edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]]; ok {
			return
		}
		edgeSet[p.nodeIDMap[from.id]][p.nodeIDMap[to.id]] = struct{}{}
		edgeSet[p.nodeIDMap[to.id]][p.nodeIDMap[from.id]] = struct{}{}

		startPointsIndex := len(graphStorage.GlobalPoints)

		graphStorage.AppendGlobalPoints(edgePoints)
		endPointsIndex := len(graphStorage.GlobalPoints)

		graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
			uint8(lanes),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
			uint32(startPointsIndex), uint32(endPointsIndex),
		),
		)

		graphStorage.SetRoundabout(uint32(len(*scannedEdges)), isRoundabout)

		*scannedEdges = append(*scannedEdges, edge{
			from:          uint32(p.nodeIDMap[from.id]),
			to:            uint32(p.nodeIDMap[to.id]),
			weight:        etaWeight,
			distance:      distanceInMeter,
			edgeID:        uint32(len(*scannedEdges)),
			bidirectional: false,
		})

		graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
			p.tagStringIdMap.GetID(tempMap[STREET_NAME]),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS])),
			uint8(lanes),
			uint8(p.tagStringIdMap.GetID(tempMap[ROAD_CLASS_LINK])),
			uint32(endPointsIndex), uint32(startPointsIndex),
		))

		*scannedEdges = append(*scannedEdges, edge{
			from:          uint32(p.nodeIDMap[to.id]),
			to:            uint32(p.nodeIDMap[from.id]),
			weight:        etaWeight,
			distance:      distanceInMeter,
			edgeID:        uint32(len(*scannedEdges)),
			bidirectional: false,
		})

	}
}

func roadTypeMaxSpeed2(roadType string) float64 {
	switch roadType {
	case "motorway":
		return 100
	case "trunk":
		return 70
	case "primary":
		return 65
	case "secondary":
		return 60
	case "tertiary":
		return 50
	case "unclassified":
		return 40
	case "residential":
		return 30
	case "service":
		return 20
	case "motorway_link":
		return 70
	case "trunk_link":
		return 65
	case "primary_link":
		return 60
	case "secondary_link":
		return 50
	case "tertiary_link":
		return 40
	case "living_street":
		return 5
	case "road":
		return 20
	case "track":
		return 15
	case "motorroad":
		return 90
	default:
		return 30
	}
}

func (p *OsmParser) isJunctionNode(nodeID int64) bool {
	return p.wayNodeMap[int64(nodeID)] == JUNCTION_NODE
}

func acceptOsmWay(way *osm.Way) bool {
	highway := way.Tags.Find("highway")
	junction := way.Tags.Find("junction")
	if highway != "" {
		if _, ok := acceptedHighway[highway]; ok {
			return true
		}
	} else if junction != "" {
		return true
	}
	return false
}

func max(a, b int64) int64 {
	if a > b {
		return a
	}
	return b
}
