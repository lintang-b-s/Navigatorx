package osmparser

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
)

// BuildGraph. build graph data structure from list of edges.
// roadNetwork = flag if the graph is a road network graph.
// test shortestpath ada beberapa yang gak pakai road network graph, diambil dari test cases soal-soal kontes pemrograman.
// jika roadNetwork=false, kita harus tambahkan dummy edge (v,v) untuk setiap vertex v di graph.
// karena Customizable Route Planning (CRP) Query phase (support turn costs) mengasumsikan setiap vertices memiliki setidaknya satu edge.
func (p *OsmParser) BuildGraph(scannedEdges []Edge, graphStorage *da.GraphStorage, numV uint32, roadNetwork bool) (*da.Graph, [][]da.Index) {
	var (
		outEdges    [][]da.OutEdge = make([][]da.OutEdge, numV)
		inEdges     [][]da.InEdge  = make([][]da.InEdge, numV)
		edgeInfoIds [][]da.Index   = make([][]da.Index, numV)
		inDegree    []int          = make([]int, numV)
		outDegree   []int          = make([]int, numV)
		vertices    []da.Vertex    = make([]da.Vertex, numV+1)
	)

	for v := 0; v < int(numV)+1; v++ {
		vertices[v] = da.NewVertex(0, 0, da.Index(v))
	}

	vertexOsmIds := make([]uint64, numV)

	// O(E)
	for eId, e := range scannedEdges {
		u := da.Index(e.from)
		v := da.Index(e.to)
		uOsmId := e.GetFromOsmId()
		vOsmId := e.GetToOsmId()

		vEntryPoint := da.Index(len(inEdges[v]))
		outEdge := da.NewOutEdge(0,
			v, e.GetWeight(), e.GetDistance(), vEntryPoint, e.GetHighwayType())

		if e.IsJunctionHead() {
			outEdge.SetJunctionHead()
		}

		if e.IsJunctionTail() {
			outEdge.SetJunctionTail()
		}

		outEdges[u] = append(outEdges[u], outEdge)

		edgeInfoIds[u] = append(edgeInfoIds[u], da.Index(eId))

		outDegree[u]++

		uExitPoint := da.Index(len(outEdges[u]) - 1)
		inEdge := da.NewInEdge(0,
			u, e.GetWeight(), e.GetDistance(), uExitPoint, e.GetHighwayType())

		if e.IsJunctionHead() {
			inEdge.SetJunctionHead()
		}

		if e.IsJunctionTail() {
			inEdge.SetJunctionTail()
		}

		inEdges[v] = append(inEdges[v], inEdge)
		inDegree[v]++

		uData := p.wayNodeMap[p.nodeToOsmId[da.Index(u)]].coord
		vertices[u] = da.NewVertex(uData.lat, uData.lon, u)

		vData := p.wayNodeMap[p.nodeToOsmId[da.Index(v)]].coord
		vertices[v] = da.NewVertex(vData.lat, vData.lon, v)

		vertexOsmIds[u] = uOsmId
		vertexOsmIds[v] = vOsmId
	}

	newEInfoId := len(scannedEdges)

	// tambahin parallel edges dulu buat via-way turn restrictions
	for wayId, way := range p.ways {
		fromRestrictions := p.restrictions[wayId]
		for fromResId, restriction := range fromRestrictions {
			if restriction.isWay {
				/*
									turn restriction berbentuk: {from-way, via-way, to-way}
									di kode ini:
									wayId/way: from-way
									restriction.to: to-way
									restriction.viaWay: via-way


									contoh: https://www.openstreetmap.org/relation/15268026
									saat ini kita cuma support viaway yang cuma punya 2 nodes (biasanya yang tipe restriction nya u-turn kaya contoh diatas).

									masalahnya:
									Turn Table nya Customizable Route Planning (CRP): https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
									cuma suppport turn cost dari entryPoint i dari vertex u ke exitPoint j, atau
									T[u][i,j] = turn cost dari via vertex u dari entryPoint i (inEdge yang head nya u) ke exitPoint j (outEdge yang tailnya u).

									kalau dari contoh diatas, misal kita add NO_ENTRY dari https://www.openstreetmap.org/way/1131069658 ke https://www.openstreetmap.org/way/1131069655 ..
									nanti dari jalan Subali Raya ke https://www.openstreetmap.org/way/1131069655 juga not allowed, padahal harusnya yang u-turn dari jalan siliwangi ke timur ke jalan siliwangi ke barat yang gaboleh...


									contoh route gmaps dari contoh diatas:
									dari subali raya: https://www.google.com/maps/dir/-6.9873908,110.3664021/-6.9881226,110.3659578/@-6.9878269,110.3658884,19.47z/data=!4m2!4m1!3e0?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D
									dari jl. siliwangi ke arah timur: https://www.google.com/maps/dir/-6.9876072,110.3661405/-6.9881226,110.3659578/@-6.9878269,110.3658884,19z/data=!4m2!4m1!3e0?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D


									contoh2: https://www.openstreetmap.org/relation/12570723#map=19/-7.729927/110.547100
									gmaps boleh u-turn: https://www.google.com/maps/dir/1st+State+Vocational+High+School,+Jogonalan,+Jl.+Raya+Solo+-+Yogyakarta+Jl.+Raya+Jogjakarta+Solo+No.313,+Tegalmas,+Prawatan,+Jogonalan,+Klaten+Regency,+Central+Java+57452/-7.7296566,110.5468658/@-7.7298901,110.5466694,19.54z/data=!4m9!4m8!1m5!1m1!1s0x2e7a41027753afb7:0x93394c25131d12eb!2m2!1d110.547994!2d-7.729615!1m0!3e0?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D


									contoh3: https://www.openstreetmap.org/relation/12845704#map=18/-7.702438/110.350628
									gmaps gaboleh u-turn di sini: https://www.google.com/maps/dir/-7.7035756,110.3503142/-7.7037607,110.3501331/@-7.7039124,110.3500935,19.14z/data=!4m2!4m1!3e0?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D


									idk mungkin solusi sementara kita bikin U-turn Table:
									T[e_u][e_i, e_j] = NO_ENTRY : gaboleh u-turn dari edge e_i -> e_u -> e_j...
									bisa pakai array 3 dimensi... T[e_u][e_i][e_j] tapi size nya kegedean kalau banyak graph edges nya....
									jangan pakai hashmap karena probingnya bikin routing lemot...

									solusi:
									T[e_u][t_i, h_j] = NO_ENTRY: gaboleh u-turn dari entryPoint i dari tail nya e_u -> e_u -> exitPoint j dari headnya e_u... (kita cuma support restrictions yang via way yang cuma punya 2 nodes)..
									bisa pakai array 3 dimensi lagi tapi size dari dim 2 adlh jumlah entryPoint dari tailnya e_u dan size dari dim 3 adlh jumlah exitPoint dari headnya e_u....


									pertama kita harus cari node (tail) dari via-way yang jadi JUNCTION dengan from-way
									kedua kita cari node (head) dari via-way yang jadi JUNCTION dengan to-way

									tinggal append ke uTurn table nya


									updated SOLUTION:
									ternyata udah di mention di paper CRP: see page 7 polyvalent turn: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

									kita bikin parallel edge dari via-way dan from-way, contoh:
									https://www.openstreetmap.org/relation/15268026
									tambah 1 edge dari via-way: https://www.openstreetmap.org/way/1131069658
									jadi ada dua edge parallel dari via-way dan from-way diatas...
									yang satu khusus diakses oleh from-way dari u-turn restriction dan satunya bisa diakses oleh other edges kecuali edge dari from-way....

									ilustrasi:

										|
										| Jalan Subali Raya
										|
									    \/
									 _________   from-edge 2 (parallel dengan from-edge 1)
									/         \
									----------->		 Jalan Siliwangi ke arah timur (from-edge 1)
											   |\
								via-edge 1	   | \ via-edge 2 (parallel dengan via-edge 1)
											   |  |
											   | /
											   |/
											   \/
					      		   <------------ Jalan Siliwangi ke arah barat (to-edge)




									nah dari from-edge 2 (https://www.openstreetmap.org/way/1131069660) ke via-edge 1 dikasih turn cost INF dan ke via-edge 2 dikasih turn cost 0...
									dari from-edge 1 (selain from-way, misalnya Jalan Subali Raya)  ke via-edge 2 dikasih turn cost INF dan via-edge 1 dikasih turn cost 0...
									dari edge Jalan Subali Raya ke from-edge 2 dikasih turn cost INF, tapi ke from-edge 1 dikasih turn cost 0...
									dari via-edge 2 ke to-edge dikasih turn cost INF (karena OSM u-turn restriction diatas).. biar dari  Jalan Siliwangi ke arah timur (from-edge 2) -> via-edge 2 -> to-edge gabisa lewat...
									dari via-edge 1 ke to-edge dikasih turn cost 0... biar dari jalan subali raya  -> from-edge 1 -> via-edge 1 -> to-edge bisa lewat ...

									kayake gak perlu bikin parallel edge buat from-edge, cuma perlu parallel edge buat via-edge, dari rute osrm dibawah:
									https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-6.987043%2C110.366592%3B-6.988103%2C110.366536#map=19/-6.987536/110.367400
									https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-6.98786%2C110.366571%3B-6.988103%2C110.366536#map=19/-6.987839/110.367400


									inspired by how to handle polyvalent turn dari paper CRP dan https://github.com/Project-OSRM/osrm-backend/issues/2681
				*/ //  nolint: gofmt

				fromNodes := way.graphNodes
				viaWays := restriction.viaWays // bisa aja via-way turn restriction, via-way nya ada lebih dari 1: https://www.openstreetmap.org/relation/17842412
				for q, viaWay := range viaWays {
					var toNodes []da.Index

					if q == len(viaWays)-1 {
						toNodes = p.ways[restriction.to].graphNodes
					} else {
						toNodes = p.ways[viaWays[q+1]].graphNodes
					}

					viaWayNodes := p.ways[viaWay].graphNodes
					viaWayNodesSet := make(map[da.Index]struct{}, len(viaWayNodes))
					for i := 0; i < len(viaWayNodes); i++ {
						viaWayNodesSet[viaWayNodes[i]] = struct{}{}
					}

					tail := da.Index(math.MaxUint32)
					head := da.Index(math.MaxUint32)

					for i := 0; i < len(fromNodes); i++ {
						if _, ok := viaWayNodesSet[fromNodes[i]]; ok {
							tail = fromNodes[i]
							break
						}
					}

					for i := 0; i < len(toNodes); i++ {
						if _, ok := viaWayNodesSet[toNodes[i]]; ok {
							head = toNodes[i]
							break
						}
					}

					if tail == da.Index(math.MaxUint32) || head == math.MaxUint32 {
						continue
					}

					viaWayEdge := da.NewEmptyOutEdge()

					viaWayEInfoId := da.Index(da.INVALID_EDGE_ID)
					for tailExitPoint, outEdge := range outEdges[tail] {
						eInfoId := edgeInfoIds[tail][tailExitPoint]
						eHead := outEdge.GetHead()
						if graphStorage.GetOsmWayId(eInfoId) == uint64(viaWay) && eHead == head {
							viaWayEInfoId = eInfoId
							viaWayEdge = outEdge
							break
						}
					}

					if viaWayEInfoId == da.INVALID_EDGE_ID {
						continue
					}

					isRoundabout := graphStorage.IsRoundabout(viaWayEInfoId)
					graphStorage.SetRoundabout(da.Index(newEInfoId), isRoundabout)
					viaHead := viaWayEdge.GetHead()
					viaHeadNewEdgeEntryPoint := da.Index(len(inEdges[viaHead]))
					viaParallelOutEdge := da.NewOutEdge(da.INVALID_PARALLEL_EDGE_ID+da.Index(fromResId)*da.Index(q), viaHead,
						viaWayEdge.GetWeight(), viaWayEdge.GetLength(), viaHeadNewEdgeEntryPoint, viaWayEdge.GetHighwayType())
					viaParallelOutEdge.SetParallelEdge()
					outEdges[tail] = append(outEdges[tail], viaParallelOutEdge)
					edgeInfoIds[tail] = append(edgeInfoIds[tail], da.Index(newEInfoId))
					outDegree[tail]++

					tailNewEdgeExitPoint := da.Index(len(outEdges[tail]) - 1)
					viaParallelInEdge := da.NewInEdge(da.INVALID_PARALLEL_EDGE_ID+da.Index(fromResId)*da.Index(q), tail,
						viaWayEdge.GetWeight(), viaWayEdge.GetLength(), tailNewEdgeExitPoint, viaWayEdge.GetHighwayType())
					viaParallelInEdge.SetParallelEdge()
					inEdges[viaHead] = append(inEdges[viaHead], viaParallelInEdge)
					inDegree[viaHead]++
					startPointId, endPointId := graphStorage.GetEdgeGeometryEndpoints(viaWayEInfoId)
					graphStorage.AppendEdgeMetadata(
						int64(graphStorage.GetOsmWayId(viaWayEInfoId)),
						startPointId, endPointId,
						graphStorage.GetStreetNameId(viaWayEInfoId),
						graphStorage.GetRoadClass(viaWayEInfoId),
						graphStorage.GetRoadClassLink(viaWayEInfoId),
						graphStorage.GetRoadLanes(viaWayEInfoId),
					)
					newEInfoId++
					fromNodes = viaWayNodes
				}
			}
		}
	}

	// O(V)
	for v := 0; v < len(vertices)-1; v++ {
		// we need to do this because Customizable Route Planning (with turn costs) query assume all vertex have at least one outEdge (at for target as source)
		if !roadNetwork || (roadNetwork && (outDegree[v] == 0 || inDegree[v] == 0)) {
			dummyOut := da.NewOutEdge(da.INVALID_EDGE_ID, da.Index(v),
				pkg.INF_WEIGHT, pkg.DUMMY_EDGE_LENGTH, da.Index(len(inEdges[v])), pkg.INVALID_HIGHWAY)
			dummyOut.SetDummyEdge()
			outEdges[v] = append(outEdges[v], dummyOut)
			edgeInfoIds[v] = append(edgeInfoIds[v], da.INVALID_EDGE_INFO_ID)
			outDegree[v]++

			dummyIn := da.NewInEdge(da.INVALID_EDGE_ID, da.Index(v),
				pkg.INF_WEIGHT, pkg.DUMMY_EDGE_LENGTH, da.Index(len(outEdges[v])-1), pkg.INVALID_HIGHWAY)
			dummyIn.SetDummyEdge()
			inEdges[v] = append(inEdges[v], dummyIn)
			inDegree[v]++

			graphStorage.AppendEdgeMetadata(
				-1,
				1, 1,
				p.tagStringIdMap.GetID(""),
				pkg.INVALID_HIGHWAY,
				pkg.INVALID_HIGHWAY,
				uint8(0),
			)
		}
	}

	// T[u][i*outDegree[u]+j] = turn type from entryPoint i (inEdge ke-i dari vertex u) to exitPoint j  (outEdge ke-j dari vertex u)  at vertex u.
	// buat via yang tipe nya osm node: https://wiki.openstreetmap.org/wiki/Relation:restriction .
	turnMatrices := make([][]pkg.TurnType, len(vertices)-1)

	// init turn matrices
	for via := 0; via < len(turnMatrices); via++ {
		turnMatrices[via] = make([]pkg.TurnType, outDegree[via]*inDegree[via])

		for j := 0; j < len(turnMatrices[via]); j++ {
			turnMatrices[via][j] = pkg.NONE
		}

		// tambahin turn type buat turn left/ turn right

		for entryPoint := 0; entryPoint < len(inEdges[via]); entryPoint++ {
			inEdge := inEdges[via][entryPoint]
			rowOffset := entryPoint * outDegree[via]

			for exitPoint := 0; exitPoint < len(outEdges[via]); exitPoint++ {
				outEdge := outEdges[via][exitPoint]

				prevPoint := vertices[inEdge.GetTail()].GetCoordinate()
				tail := vertices[via].GetCoordinate()
				headPoint := vertices[outEdge.GetHead()].GetCoordinate()

				prevInitialBearing := geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(),
					tail.GetLon())
				turn := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headPoint.GetLat(),
					headPoint.GetLon(), prevInitialBearing)
				switch turn {
				case da.TURN_SLIGHT_LEFT, da.TURN_LEFT, da.TURN_SHARP_LEFT:
					turnMatrices[via][rowOffset+exitPoint] = pkg.LEFT_TURN
				case da.TURN_SLIGHT_RIGHT, da.TURN_RIGHT, da.TURN_SHARP_RIGHT:
					turnMatrices[via][rowOffset+exitPoint] = pkg.RIGHT_TURN
				}
			}
		}
	}

	isParallelEdge := func(eId, fromResId, q da.Index) bool {
		if eId < da.INVALID_PARALLEL_EDGE_ID {
			return false
		}
		if eId == da.INVALID_PARALLEL_EDGE_ID+fromResId*q {
			return true
		}
		return false
	}

	// let m=number of ways , q = max number of nodes of any osm ways, r = max number of restrictions of any osm ways
	// O(m*r*q^2)
	for wayId, way := range p.ways {

		/*
			misal osm way (twoway):
				u1<->u2<->u3<->u4

				kita harus store semua u_turn dari edges yanng menyusun twoway osm way tsb:
				list u_turn:
				u1->u2->u1

				u2->u3->u2
				u2->u1->u2

				u3->u4->u3
				u3->u2->u3

				u4->u3->u4
		*/
		if !way.oneWay {

			for i, via := range way.graphNodes {

				if len(way.graphNodes) <= 1 {
					continue
				}

				if i == 0 {
					// store u_turn restrictions
					// dont allow u_turn at (to, via)->(via, to)
					to := way.graphNodes[1]
					if to == via {
						continue
					}

					entryPoint := -1
					exitPoint := -1
					for k := 0; k < len(outEdges[via]); k++ {
						if outEdges[via][k].GetHead() == to {

							exitPoint = k
							break
						}
					}

					for k := 0; k < len(inEdges[via]); k++ {
						if inEdges[via][k].GetTail() == to {

							entryPoint = k
							break
						}
					}

					if entryPoint == -1 || exitPoint == -1 {
						continue
					}

					turnMatrices[via][entryPoint*int(outDegree[via])+exitPoint] = pkg.U_TURN
				} else if i < len(way.graphNodes)-1 {

					// backward
					to := way.graphNodes[i-1]
					if to != via {
						entryPoint := -1
						exitPoint := -1
						for k := 0; k < len(outEdges[via]); k++ {
							if outEdges[via][k].GetHead() == to {

								exitPoint = k
								break
							}
						}

						for k := 0; k < len(inEdges[via]); k++ {
							if inEdges[via][k].GetTail() == to {

								entryPoint = k
								break
							}
						}

						if entryPoint != -1 && exitPoint != -1 {
							turnMatrices[via][entryPoint*int(outDegree[via])+exitPoint] = pkg.U_TURN
						}
					}

					// forward
					to = way.graphNodes[i+1]
					if to != via {
						entryPoint := -1
						exitPoint := -1
						for k := 0; k < len(outEdges[via]); k++ {
							if outEdges[via][k].GetHead() == to {

								exitPoint = k
								break
							}
						}

						for k := 0; k < len(inEdges[via]); k++ {
							if inEdges[via][k].GetTail() == to {

								entryPoint = k
								break
							}
						}

						if entryPoint != -1 && exitPoint != -1 {
							turnMatrices[via][entryPoint*int(outDegree[via])+exitPoint] = pkg.U_TURN
						}
					}

				} else {
					// last node in way.graphNodes
					to := way.graphNodes[i-1]

					if to == via {
						continue
					}

					entryPoint := -1
					exitPoint := -1
					for k := 0; k < len(outEdges[via]); k++ {
						if outEdges[via][k].GetHead() == to {

							exitPoint = k
							break
						}
					}

					for k := 0; k < len(inEdges[via]); k++ {
						if inEdges[via][k].GetTail() == to {

							entryPoint = k
							break
						}
					}

					if entryPoint == -1 || exitPoint == -1 {
						continue
					}

					turnMatrices[via][entryPoint*int(outDegree[via])+exitPoint] = pkg.U_TURN
				}
			}
		}

		// store turn restrictions https://wiki.openstreetmap.org/wiki/Relation:restriction

		fromNodes := way.graphNodes
		fromRestrictions := p.restrictions[wayId]
		for fromResId, restriction := range fromRestrictions {

			if wayId == int64(restriction.to) { // ignore restrictions from wayId == restriction.to
				continue
			}

			_, acceptedWay := p.ways[int64(restriction.to)]
			if !acceptedWay {
				continue
			}

			if !restriction.isWay {
				/*
						turn restriction berbentuk: {from-way, via-node, to-way}
						di kode ini:
						wayId/way: from-way
						restriction.to: to-way

						via-node berada di nodes nya from-way

						contoh: https://www.openstreetmap.org/relation/19474168#map=19/-7.782550/110.375438
						https://www.openstreetmap.org/api/0.6/relation/19474168
						https://www.openstreetmap.org/relation/5710500

					jadi kita pertama harus cari way.graphNodes yang jadi via-node

					note that from-way ke to-way bisa terhubung karena ada via-node yang jadi node di kedua way
					misal:
					u1 -> u2->via -> w1 ->w2
					from-way      to-way
					nah via ini jadi node di from-way.graphNodes dan to-way.graphNodes

					langkah kedua kita harus cari to-way.graphNodes yang == restriction.via

					kita store turnTable as:
					key (entryPoint, viaNode, exitPoint) -> tipe dari turn restrictionnya
					entryPoint adalah inEdge yang headnya ke viaNode
					exitPoint adalah outEdge yang tailnya dari viaNode

				*/

				for i := 0; i < len(fromNodes); i++ {
					if fromNodes[i] == restriction.via {
						if i == 0 && way.oneWay {
							// no predecessor
							continue
						}

						var predecessor da.Index // predecessor dari via nya turn restriction
						if i == 0 {
							// note that di osm_parser.go , way bisa two-way (dua arah) yang mana setiap pasang junction/end node dari osm way kita pecah jadi dua edges (kalau two-way)...
							// kalau via node dari turn restriction di first node dari from-way..
							// berarti predecessor nya ada di next node (i+1).. ke arah forward..
							// from-way nodes: via<->u2<->u3<->-.....<->un

							predecessor = fromNodes[i+1]
						} else {
							predecessor = fromNodes[i-1]
						}

						if predecessor == restriction.via {
							continue
						}

						successor := da.Index(math.MaxUint32) // successor dari via nya turn restriction
						toNodes := p.ways[int64(restriction.to)].graphNodes
						for j := 0; j < len(toNodes); j++ {
							if toNodes[j] == restriction.via {
								if j == len(toNodes)-1 {
									// note that di osm_parser.go , way bisa two-way (dua arah) yang mana setiap pasang junction/end node dari osm way kita pecah jadi dua edges (kalau two-way)...
									// kalau via node dari turn restriction di last node dari to-way..
									// berarti predecessor nya ada di next node (i-1).. ke arah backward
									// to-way nodes: u1<->u2<->u3<->-.....<->via

									successor = toNodes[j-1]
								} else {
									successor = toNodes[j+1]
								}
								break
							}
						}

						if successor != da.Index(math.MaxUint32) && successor != restriction.via {

							// (from, via, to) nodes dari turn restriction
							via := da.Index(restriction.via)

							entryPoint := da.Index(math.MaxUint32)
							exitPoint := da.Index(math.MaxUint32)

							inEdge := da.InEdge{}
							for k := 0; k < len(inEdges[via]); k++ {
								if inEdges[via][k].GetTail() == predecessor {
									entryPoint = da.Index(k)
									inEdge = inEdges[via][k]
									break
								}
							}

							if entryPoint == da.Index(math.MaxUint32) {
								continue
							}

							rowOffset := entryPoint * da.Index(outDegree[via])
							for k := 0; k < len(outEdges[via]); k++ {
								outEdge := outEdges[via][k]
								if outEdge.GetHead() == successor && !isParallelEdge(outEdge.GetEdgeId(), da.Index(fromResId), 0) {

									exitPoint = da.Index(k)

								}

								prevPoint := vertices[inEdge.GetTail()].GetCoordinate()
								tail := vertices[via].GetCoordinate()
								headPoint := vertices[outEdge.GetHead()].GetCoordinate()

								switch restriction.turnRestriction {
								case ONLY_LEFT_TURN:
									/*
											. = restriction.via node

										--------.--------- restriction.to way (two-way)
												|
												|
												|
												|
												|
												restriction.from  way

											misal ONLY_LEFT_TURN:
											berarti kita harus dissalow semua turn right...
											cara taunya cuma bisa dari relative bearing dari restriction.from ke restriction.to....

									*/

									prevInitialBearing := geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(),
										tail.GetLon())
									turn := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headPoint.GetLat(),
										headPoint.GetLon(), prevInitialBearing)
									if turn == da.TURN_SLIGHT_RIGHT || turn == da.TURN_RIGHT || turn == da.TURN_SHARP_RIGHT || turn == da.CONTINUE_ON_STREET {
										// dissallow semua turn right...
										// https://www.openstreetmap.org/relation/19516441#map=18/-7.774471/110.380569
										// https://www.openstreetmap.org/relation/19514924
										turnMatrices[via][rowOffset+da.Index(k)] = pkg.NO_ENTRY
									}

								case ONLY_RIGHT_TURN:

									prevInitialBearing := geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(),
										tail.GetLon())
									turn := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headPoint.GetLat(),
										headPoint.GetLon(), prevInitialBearing)
									if turn == da.TURN_SLIGHT_LEFT || turn == da.TURN_LEFT || turn == da.TURN_SHARP_LEFT || turn == da.CONTINUE_ON_STREET {
										// dissallow semua turn left...
										// https://www.openstreetmap.org/relation/19514925
										turnMatrices[via][rowOffset+da.Index(k)] = pkg.NO_ENTRY
									}

								case ONLY_STRAIGHT_ON:
									prevInitialBearing := geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(),
										tail.GetLon())
									turn := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headPoint.GetLat(),
										headPoint.GetLon(), prevInitialBearing)
									if turn == da.TURN_LEFT || turn == da.TURN_SHARP_LEFT ||
										turn == da.TURN_RIGHT || turn == da.TURN_SHARP_RIGHT {
										// contoh2: openstreetmap.org/relation/19516443 , (only_straight_on) ini kedetect nya TURN_SLIGHT_LEFT buat ke arah UNY...
										// padahal continue ..
										// how to fix?? gak usah include TURN_SLIGHT_LEFT buat NO_ENTRY nya only_straight_on
										// tapi yang lebih serem kalau ada case only_straight_on tapi ada belokan slight_left/slight_right yang diallow sama kode ini....
										// udah debugging and test pakai file osm yang include solo,diy,semarang,salatiga gak ada kasus gini sih

										// dissallow semua turn right dan turn left...
										// contoh: http://openstreetmap.org/relation/19474168
										turnMatrices[via][rowOffset+da.Index(k)] = pkg.NO_ENTRY
									}

								}
							}

							if exitPoint == da.Index(math.MaxUint32) {
								continue
							}

							if rowOffset+exitPoint >= da.Index(len(turnMatrices[via])) {
								continue
							}

							switch restriction.turnRestriction {
							case NO_LEFT_TURN:
								turnMatrices[via][rowOffset+exitPoint] = pkg.NO_ENTRY

							case NO_RIGHT_TURN: // contoh: https://www.openstreetmap.org/relation/5710505
								turnMatrices[via][rowOffset+exitPoint] = pkg.NO_ENTRY

							case NO_STRAIGHT_ON:
								turnMatrices[via][rowOffset+exitPoint] = pkg.NO_ENTRY

							case NO_U_TURN: // harus NO_ENTRY karena gak boleh u-turn: example: https://www.openstreetmap.org/relation/10732316#map=19/-7.566370/110.775455
								turnMatrices[via][rowOffset+exitPoint] = pkg.NO_ENTRY

							case NO_ENTRY:
								turnMatrices[via][rowOffset+exitPoint] = pkg.NO_ENTRY

							case ONLY_LEFT_TURN:
								turnMatrices[via][rowOffset+exitPoint] = pkg.LEFT_TURN

							case ONLY_RIGHT_TURN:
								turnMatrices[via][rowOffset+exitPoint] = pkg.RIGHT_TURN

							case ONLY_STRAIGHT_ON: // udah kita dissalow semua right & left turn di loc diatas
								turnMatrices[via][rowOffset+exitPoint] = pkg.NONE

							default:
								turnMatrices[via][rowOffset+exitPoint] = pkg.NONE

							}
						}
						break
					}
				}
			} else if restriction.isWay {
				/*
									turn restriction berbentuk: {from-way, via-way, to-way}
									di kode ini:
									wayId/way: from-way
									restriction.to: to-way
									restriction.viaWay: via-way


									contoh: https://www.openstreetmap.org/relation/15268026
									saat ini kita cuma support viaway yang cuma punya 2 nodes (biasanya yang tipe restriction nya u-turn kaya contoh diatas).

									masalahnya:
									Turn Table nya Customizable Route Planning (CRP): https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
									cuma suppport turn cost dari entryPoint i dari vertex u ke exitPoint j, atau
									T[u][i,j] = turn cost dari via vertex u dari entryPoint i (inEdge yang head nya u) ke exitPoint j (outEdge yang tailnya u).

									kalau dari contoh diatas, misal kita add NO_ENTRY dari https://www.openstreetmap.org/way/1131069658 ke https://www.openstreetmap.org/way/1131069655 ..
									nanti dari jalan Subali Raya ke https://www.openstreetmap.org/way/1131069655 juga not allowed, padahal harusnya yang u-turn dari jalan siliwangi ke timur ke jalan siliwangi ke barat yang gaboleh...


									contoh route gmaps dari contoh diatas:
									dari subali raya: https://www.google.com/maps/dir/-6.9873908,110.3664021/-6.9881226,110.3659578/@-6.9878269,110.3658884,19.47z/data=!4m2!4m1!3e0?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D
									dari jl. siliwangi ke arah timur: https://www.google.com/maps/dir/-6.9876072,110.3661405/-6.9881226,110.3659578/@-6.9878269,110.3658884,19z/data=!4m2!4m1!3e0?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D


									contoh2: https://www.openstreetmap.org/relation/12570723#map=19/-7.729927/110.547100
									gmaps boleh u-turn: https://www.google.com/maps/dir/1st+State+Vocational+High+School,+Jogonalan,+Jl.+Raya+Solo+-+Yogyakarta+Jl.+Raya+Jogjakarta+Solo+No.313,+Tegalmas,+Prawatan,+Jogonalan,+Klaten+Regency,+Central+Java+57452/-7.7296566,110.5468658/@-7.7298901,110.5466694,19.54z/data=!4m9!4m8!1m5!1m1!1s0x2e7a41027753afb7:0x93394c25131d12eb!2m2!1d110.547994!2d-7.729615!1m0!3e0?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D


									contoh3: https://www.openstreetmap.org/relation/12845704#map=18/-7.702438/110.350628
									gmaps gaboleh u-turn di sini: https://www.google.com/maps/dir/-7.7035756,110.3503142/-7.7037607,110.3501331/@-7.7039124,110.3500935,19.14z/data=!4m2!4m1!3e0?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D


									updated SOLUTION:
									ternyata udah di mention di paper CRP: see page 7 polyvalent turn: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

									kita bikin parallel edge dari via-way dan from-way, contoh:
									https://www.openstreetmap.org/relation/15268026
									tambah 1 edge dari via-way: https://www.openstreetmap.org/way/1131069658
									jadi ada dua edge parallel dari via-way dan from-way diatas...
									yang satu khusus diakses oleh from-way dari u-turn restriction dan satunya bisa diakses oleh other edges kecuali edge dari from-way....

									ilustrasi:

										|
										| Jalan Subali Raya
										|
									    \/
									 _________   from-edge 2 (parallel dengan from-edge 1)
									/         \
									----------->		 Jalan Siliwangi ke arah timur (from-edge 1)
											   |\
								via-edge 1	   | \ via-edge 2 (parallel dengan via-edge 1)
											   |  |
											   | /
											   |/
											   \/
					      		   <------------ Jalan Siliwangi ke arah barat (to-edge)




									nah dari from-edge 2 (https://www.openstreetmap.org/way/1131069660) ke via-edge 1 dikasih turn cost INF dan ke via-edge 2 dikasih turn cost 0...
									dari from-edge 1 (selain from-way, misalnya Jalan Subali Raya)  ke via-edge 2 dikasih turn cost INF dan via-edge 1 dikasih turn cost 0...
									dari edge Jalan Subali Raya ke from-edge 2 dikasih turn cost INF, tapi ke from-edge 1 dikasih turn cost 0...
									dari via-edge 2 ke to-edge dikasih turn cost INF (karena OSM u-turn restriction diatas).. biar dari  Jalan Siliwangi ke arah timur (from-edge 2) -> via-edge 2 -> to-edge gabisa lewat...
									dari via-edge 1 ke to-edge dikasih turn cost 0... biar dari jalan subali raya  -> from-edge 1 -> via-edge 1 -> to-edge bisa lewat ...

									kayake gak perlu bikin parallel edge buat from-edge, cuma perlu parallel edge buat via-edge, dari rute osrm dibawah:
									https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-6.987043%2C110.366592%3B-6.988103%2C110.366536#map=19/-6.987536/110.367400
									https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-6.98786%2C110.366571%3B-6.988103%2C110.366536#map=19/-6.987839/110.367400


									inspired by how to handle polyvalent turn dari paper CRP dan https://github.com/Project-OSRM/osrm-backend/issues/2681

									todo: add testCases turn restriction with via-way

				*/ //  nolint: gofmt

				// bisa aja via-way turn restriction, via-way nya ada lebih dari 1: https://www.openstreetmap.org/relation/17842412

				viaWays := restriction.viaWays
				moreThanOneViaWays := len(viaWays) > 1
				for q, viaWay := range viaWays {

					var (
						toNodes                   []da.Index
						lastViaway                bool
						tailInEdgeInParallelEdges bool
					)
					if q == len(viaWays)-1 {
						toNodes = p.ways[restriction.to].graphNodes
						lastViaway = true

						if moreThanOneViaWays {
							tailInEdgeInParallelEdges = true
						}
					} else {
						toNodes = p.ways[viaWays[q+1]].graphNodes

						if q > 0 {
							tailInEdgeInParallelEdges = true
						}

					}

					viaWayNodes := p.ways[viaWay].graphNodes

					viaWayNodesSet := make(map[da.Index]struct{}, len(viaWayNodes))
					for i := 0; i < len(viaWayNodes); i++ {
						viaWayNodesSet[viaWayNodes[i]] = struct{}{}
					}

					tail := da.Index(math.MaxUint32) // tail dari via-edge
					head := da.Index(math.MaxUint32) // head dari via-edge

					for i := 0; i < len(fromNodes); i++ {
						if _, ok := viaWayNodesSet[fromNodes[i]]; ok {
							tail = fromNodes[i]
							break
						}
					}

					for i := 0; i < len(toNodes); i++ {
						if _, ok := viaWayNodesSet[toNodes[i]]; ok {
							head = toNodes[i]
							break
						}
					}
					if tail == da.Index(math.MaxUint32) || head == math.MaxUint32 {
						continue
					}

					tailNewViaEdgeExitPoint := da.Index(0)

					viaWayEInfoId := da.Index(da.INVALID_EDGE_ID)
					for i := 0; i < len(outEdges[tail]); i++ {
						eInfoId := edgeInfoIds[tail][i]
						outEdge := outEdges[tail][i]
						if outEdge.GetHighwayType() == pkg.INVALID_HIGHWAY {
							// skip dummy edges
							continue
						}
						eHead := outEdge.GetHead()
						if graphStorage.GetOsmWayId(eInfoId) == uint64(viaWay) && eHead == head &&
							isParallelEdge(outEdge.GetEdgeId(), da.Index(fromResId), da.Index(q)) {
							tailNewViaEdgeExitPoint = da.Index(i)
							viaWayEInfoId = eInfoId
						}
					}

					if viaWayEInfoId == da.INVALID_EDGE_ID {
						continue
					}

					headNewViaEdgeEntryPoint := da.Index(math.MaxUint32)
					for i := 0; i < len(inEdges[head]); i++ {
						inEdge := inEdges[head][i]
						eTail := inEdge.GetTail()
						if eTail == tail && isParallelEdge(inEdge.GetEdgeId(), da.Index(fromResId), da.Index(q)) {
							headNewViaEdgeEntryPoint = da.Index(i)
						}
					}

					if headNewViaEdgeEntryPoint == math.MaxUint32 {
						continue
					}

					for i := 0; i < len(fromNodes); i++ {
						if fromNodes[i] == tail {
							if i == 0 && way.oneWay {
								// no predecessor
								continue
							}

							var predecessor da.Index // predecessor dari tail dari edge via nya turn restriction
							if i == 0 {
								// note that di osm_parser.go , way bisa two-way (dua arah) yang mana setiap pasang junction/end node dari osm way kita pecah jadi dua edges (kalau two-way)...
								// kalau via node dari turn restriction di first node dari from-way..
								// berarti predecessor nya ada di next node (i+1).. ke arah forward..
								// from-way nodes: via<->u2<->u3<->-.....<->un
								predecessor = fromNodes[i+1]
							} else {
								predecessor = fromNodes[i-1]
							}

							if predecessor == head {
								continue
							}

							successor := da.Index(math.MaxUint32) // successor dari head dari edge via nya turn restriction

							for j := 0; j < len(toNodes); j++ {
								if toNodes[j] == head {
									if j == len(toNodes)-1 {
										// note that di osm_parser.go , way bisa two-way (dua arah) yang mana setiap pasang junction/end node dari osm way kita pecah jadi dua edges (kalau two-way)...
										// kalau via node dari turn restriction di last node dari to-way..
										// berarti predecessor nya ada di next node (i-1).. ke arah backward
										// to-way nodes: u1<->u2<->u3<->-.....<->via

										successor = toNodes[j-1]
									} else {
										successor = toNodes[j+1]
									}
									break
								}
							}

							if successor != da.Index(math.MaxUint32) {

								tailEntryPoint := da.Index(math.MaxUint32)

								for k := da.Index(0); k < da.Index(len(inEdges[tail])); k++ {
									tailInEdge := inEdges[tail][k]
									validInEdge := ((tailInEdgeInParallelEdges && isParallelEdge(tailInEdge.GetEdgeId(), da.Index(fromResId), da.Index(q-1))) ||
										!tailInEdgeInParallelEdges && !isParallelEdge(tailInEdge.GetEdgeId(), da.Index(fromResId), da.Index(q-1)))
									if tailInEdge.GetTail() == predecessor && validInEdge {
										tailEntryPoint = k
									} else {
										// kasih turn cost INF buat transisi dari  all other inEdges dari tail (selain from-edge) -> tail -> via-edge 2
										turnMatrices[tail][k*da.Index(outDegree[tail])+tailNewViaEdgeExitPoint] = pkg.NO_ENTRY
									}
								}

								if tailEntryPoint == da.Index(math.MaxUint32) {
									continue
								}

								tailViaEdgeExitPoint := da.Index(math.MaxUint32)

								for k := 0; k < len(outEdges[tail]); k++ {
									outEdge := outEdges[tail][k]
									validOutEdge := !isParallelEdge(outEdge.GetEdgeId(), da.Index(fromResId), da.Index(q))
									if outEdge.GetHead() == head && validOutEdge {
										tailViaEdgeExitPoint = da.Index(k)
										break
									}
								}

								if tailViaEdgeExitPoint == da.Index(math.MaxUint32) {
									continue
								}

								rowOffset := tailEntryPoint * da.Index(outDegree[tail])

								// kasih turn cost 0 buat transisi dari from-edge -> tail -> via-edge 2
								turnMatrices[tail][rowOffset+tailNewViaEdgeExitPoint] = pkg.NONE

								// kasih turn cost INF buat transisi dari from-edge -> tail -> via-edge 1
								turnMatrices[tail][rowOffset+tailViaEdgeExitPoint] = pkg.NO_ENTRY

								if lastViaway {

									// dari via-edge 2 ke to-edge dikasih turn cost INF (karena OSM u-turn restriction diatas).. biar dari  Jalan Siliwangi ke arah timur (from-edge 2) -> via-edge 2 -> to-edge gabisa lewat...
									// dari via-edge 1 ke to-edge dikasih turn cost 0... biar dari jalan subali raya  -> from-edge 1 -> via-edge 1 -> to-edge bisa lewat ...

									headExitPoint := da.Index(math.MaxUint32)
									for k := 0; k < len(outEdges[head]); k++ {
										outEdge := outEdges[head][k]
										if outEdge.GetHead() == successor && !isParallelEdge(outEdge.GetEdgeId(), da.Index(fromResId), da.Index(q)) {
											headExitPoint = da.Index(k)

											break
										}
									}

									if headExitPoint == da.Index(math.MaxUint32) {
										continue
									}

									rowOffset = headNewViaEdgeEntryPoint * da.Index(outDegree[head])

									switch restriction.turnRestriction {
									case NO_LEFT_TURN:
										turnMatrices[head][rowOffset+headExitPoint] = pkg.NO_ENTRY

									case NO_RIGHT_TURN: // contoh: https://www.openstreetmap.org/relation/5710505
										turnMatrices[head][rowOffset+headExitPoint] = pkg.NO_ENTRY

									case NO_STRAIGHT_ON:
										turnMatrices[head][rowOffset+headExitPoint] = pkg.NO_ENTRY

									case NO_U_TURN: // harus NO_ENTRY karena gak boleh u-turn: example: https://www.openstreetmap.org/relation/10732316#map=19/-7.566370/110.775455
										turnMatrices[head][rowOffset+headExitPoint] = pkg.NO_ENTRY

									case NO_ENTRY:
										turnMatrices[head][rowOffset+headExitPoint] = pkg.NO_ENTRY

									default:
										turnMatrices[head][rowOffset+headExitPoint] = pkg.NONE

									}
								}

								break
							}
						}
					}

					fromNodes = viaWayNodes
				}
			}
		}
	}

	matrices := make([]pkg.TurnType, 0)
	matrixOffset := 0

	for v := 0; v < len(vertices)-1; v++ {

		// set the turnTablePtr of vertex v to the current matrixOffset
		// matrix offset is index of the first element of turnMatrices[v] in the flattened matrices array
		vertices[v].SetTurnTablePtr(da.Index(matrixOffset))
		// flatten the turnMatrices
		for i := 0; i < len(turnMatrices[v]); i++ {

			matrices = append(matrices, turnMatrices[v][i])
		}

		matrixOffset += len(turnMatrices[v])
	}

	outEdgeOffset := da.Index(0)
	inEdgeOffset := da.Index(0)

	for i := 0; i < len(vertices)-1; i++ {
		vertices[i].SetFirstOut(outEdgeOffset) // index of the first outEdge of vertex i in the flattened outEdges array
		vertices[i].SetFirstIn(inEdgeOffset)
		outEdgeOffset += da.Index(len(outEdges[i]))
		inEdgeOffset += da.Index(len(inEdges[i]))
	}

	// dummy vertex
	vertices[len(vertices)-1] = da.NewVertex(0, 0, da.Index(len(vertices)-1))
	vertices[len(vertices)-1].SetFirstOut(outEdgeOffset)
	vertices[len(vertices)-1].SetFirstIn(inEdgeOffset)

	flattenOutEdges := flatten(outEdges)
	for i := 0; i < len(flattenOutEdges); i++ {
		outEdgeId := da.Index(i)
		flattenOutEdges[i].SetEdgeId(outEdgeId)
	}

	flattenInEdges := flatten(inEdges)
	for i := 0; i < len(flattenInEdges); i++ {
		flattenInEdges[i].SetEdgeId(da.Index(i))
	}

	verticesOsmIdsPs := da.NewPackedSlice(da.BIT_SIZE_OSM_NODE_ID, uint64(numV)+1)

	for _, osmId := range vertexOsmIds {
		verticesOsmIdsPs.Append(osmId)
	}

	graph := da.NewGraph(vertices, flattenOutEdges, flattenInEdges, matrices, roadNetwork, verticesOsmIdsPs)
	graphStorage.BuildNameTable(p.tagStringIdMap.GetIdToStr())
	graph.SetGraphStorage(graphStorage)

	return graph, edgeInfoIds
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
