package extractor

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// kumpulan helper functions untuk process via-node turn restrictions & (multi)-via-way turn-restrictions dari OpenStreetMap
// addTwoWayTurnCost. add turn cost untuk u-turn di two-way osm way (road segment).
// didaptasi dari https://github.com/michaelwegner/CRP/blob/master/io/OSMParser.cpp
func addTwoWayTurnCost[W util.RoutingNumber](p *Extractor[W], wayId int64, way osmWay, outEdges [][]da.Index,
	inEdges [][]da.Index, turnMatrices [][]pkg.TurnType, outDegree []int) {
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
		uturnType := pkg.U_TURN
		if _, reversible := p.reversibleOsmWay[wayId]; reversible {
			// If a vehicle travels a oneway=reversible in any direction, routing engines should infer u-turn restriction
			uturnType = pkg.NO_ENTRY
		}

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
					if outEdges[via][k] == to {

						exitPoint = k
						break
					}
				}

				for k := 0; k < len(inEdges[via]); k++ {
					if inEdges[via][k] == to {

						entryPoint = k
						break
					}
				}

				if entryPoint == -1 || exitPoint == -1 {
					continue
				}

				turnMatrices[via][entryPoint*int(outDegree[via])+exitPoint] = uturnType
			} else if i < len(way.graphNodes)-1 {

				// backward
				to := way.graphNodes[i-1]
				if to != via {
					entryPoint := -1
					exitPoint := -1
					for k := 0; k < len(outEdges[via]); k++ {
						if outEdges[via][k] == to {

							exitPoint = k
							break
						}
					}

					for k := 0; k < len(inEdges[via]); k++ {
						if inEdges[via][k] == to {

							entryPoint = k
							break
						}
					}

					if entryPoint != -1 && exitPoint != -1 {
						turnMatrices[via][entryPoint*int(outDegree[via])+exitPoint] = uturnType
					}
				}

				// forward
				to = way.graphNodes[i+1]
				if to != via {
					entryPoint := -1
					exitPoint := -1
					for k := 0; k < len(outEdges[via]); k++ {
						if outEdges[via][k] == to {

							exitPoint = k
							break
						}
					}

					for k := 0; k < len(inEdges[via]); k++ {
						if inEdges[via][k] == to {

							entryPoint = k
							break
						}
					}

					if entryPoint != -1 && exitPoint != -1 {
						turnMatrices[via][entryPoint*int(outDegree[via])+exitPoint] = uturnType
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
					if outEdges[via][k] == to {

						exitPoint = k
						break
					}
				}

				for k := 0; k < len(inEdges[via]); k++ {
					if inEdges[via][k] == to {

						entryPoint = k
						break
					}
				}

				if entryPoint == -1 || exitPoint == -1 {
					continue
				}

				turnMatrices[via][entryPoint*int(outDegree[via])+exitPoint] = uturnType
			}
		}
	}
}

// handler via-node turn restriction
// didaptasi dari https://github.com/michaelwegner/CRP/blob/master/io/OSMParser.cpp
func addViaNodeTurnRestriction[W util.RoutingNumber](p *Extractor[W], wayId int64, way osmWay, fromNodes []da.Index, restriction restriction, fromResId int, outEdges [][]da.Index,
	inEdges [][]da.Index, outDegree []int, vertices []da.Vertex, turnMatrices [][]pkg.TurnType, conditionalTurnRestrictions *[]da.ConditionalTurnRestriction,
	isParallelOutEdge, isParallelInEdge [][]bool) {
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
		entryPoint adalah index dari inEdge (dari list of incoming edges dari via node) yang headnya ke viaNode
		exitPoint adalah index dari outEdge (dari list of outgoing edges dari via node) yang tailnya dari viaNode

		basically ini untuk store turn cost belok dari road segment (u2,via) ke road segment (via,w1)

		fromNodes adalah graph nodes dari from-way
		toNodes adalah graph nodes dari to-way

		disini kita pakai variable u untuk represent u2 dan w untuk represent w1
	*/

	if len(fromNodes) < 2 {
		return
	}
	for i := 0; i < len(fromNodes); i++ {
		if fromNodes[i] == restriction.via {
			if i == 0 && way.oneWay {
				// no predecessor
				continue
			}

			var u da.Index // predecessor node dari via node nya turn restriction.
			if i == 0 {
				// note that di osm_parser.go , way bisa two-way (dua arah) yang mana setiap pasang junction/end node dari osm way kita pecah jadi dua edges (kalau two-way)...
				// kalau via node dari turn restriction di first node dari from-way..
				// berarti predecessor nya ada di next node (i+1).. ke arah forward..
				// from-way nodes: via<->u2<->u3<->-.....<->un

				u = fromNodes[i+1]
			} else {
				u = fromNodes[i-1]
			}

			if u == restriction.via {
				continue
			}

			w := da.Index(math.MaxUint32) // successor node dari via node nya turn restriction
			toNodes := p.ways[int64(restriction.to)].graphNodes
			for j := 0; j < len(toNodes); j++ {
				if toNodes[j] == restriction.via {
					if j == len(toNodes)-1 {
						// note that di osm_parser.go , way bisa two-way (dua arah) yang mana setiap pasang junction/end node dari osm way kita pecah jadi dua edges (kalau two-way)...
						// kalau via node dari turn restriction di last node dari to-way..
						// berarti predecessor nya ada di next node (i-1).. ke arah backward
						// to-way nodes: u1<->u2<->u3<->-.....<->via

						w = toNodes[j-1]
					} else {
						w = toNodes[j+1]
					}
					break
				}
			}

			if w != da.Index(math.MaxUint32) && w != restriction.via {

				// (from, via, to) nodes dari turn restriction
				via := da.Index(restriction.via)

				entryPoint := da.Index(math.MaxUint32)
				exitPoint := da.Index(math.MaxUint32)

				for k := 0; k < len(inEdges[via]); k++ {
					if inEdges[via][k] == u {
						entryPoint = da.Index(k)
						break
					}
				}

				if entryPoint == da.Index(math.MaxUint32) {
					continue
				}

				rowOffset := entryPoint * da.Index(outDegree[via])
				for k := 0; k < len(outEdges[via]); k++ {
					if outEdges[via][k] == w && !isParallelOutEdge[via][k] {
						exitPoint = da.Index(k)
					}

					uCoord := vertices[u].GetCoordinate()
					viaCoord := vertices[via].GetCoordinate()
					wCoord := vertices[w].GetCoordinate()

					if !restriction.conditional {
						switch restriction.turnRestriction {
						// handle ONLY_LEFT_TURN/ONLY_RIGHT_TURN/ONLY_STRAIGHT_ON
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

							*/ // nolint: gofmt

							prevInitialBearing := geo.ComputeInitialBearing(uCoord.GetLat(), uCoord.GetLon(), viaCoord.GetLat(),
								viaCoord.GetLon())
							turn := geo.GetTurnDirection(viaCoord.GetLat(), viaCoord.GetLon(), wCoord.GetLat(),
								wCoord.GetLon(), prevInitialBearing)
							if turn == da.TURN_SLIGHT_RIGHT || turn == da.TURN_RIGHT || turn == da.TURN_SHARP_RIGHT || turn == da.CONTINUE_ON_STREET {
								// dissallow semua turn right...
								// https://www.openstreetmap.org/relation/19516441#map=18/-7.774471/110.380569
								// https://www.openstreetmap.org/relation/19514924
								turnMatrices[via][rowOffset+da.Index(k)] = pkg.NO_ENTRY
							}

						case ONLY_RIGHT_TURN:

							prevInitialBearing := geo.ComputeInitialBearing(uCoord.GetLat(), uCoord.GetLon(), viaCoord.GetLat(),
								viaCoord.GetLon())
							turn := geo.GetTurnDirection(viaCoord.GetLat(), viaCoord.GetLon(), wCoord.GetLat(),
								wCoord.GetLon(), prevInitialBearing)
							if turn == da.TURN_SLIGHT_LEFT || turn == da.TURN_LEFT || turn == da.TURN_SHARP_LEFT || turn == da.CONTINUE_ON_STREET {
								// dissallow semua turn left...
								// https://www.openstreetmap.org/relation/19514925
								turnMatrices[via][rowOffset+da.Index(k)] = pkg.NO_ENTRY
							}

						case ONLY_STRAIGHT_ON:
							prevInitialBearing := geo.ComputeInitialBearing(uCoord.GetLat(), uCoord.GetLon(), viaCoord.GetLat(),
								viaCoord.GetLon())
							turn := geo.GetTurnDirection(viaCoord.GetLat(), viaCoord.GetLon(), wCoord.GetLat(),
								wCoord.GetLon(), prevInitialBearing)
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
				}

				if exitPoint == da.Index(math.MaxUint32) {
					continue
				}

				if rowOffset+exitPoint >= da.Index(len(turnMatrices[via])) {
					continue
				}

				turnType := pkg.NONE
				switch restriction.turnRestriction {
				case NO_LEFT_TURN:
					turnType = pkg.NO_ENTRY

				case NO_RIGHT_TURN: // contoh: https://www.openstreetmap.org/relation/5710505
					turnType = pkg.NO_ENTRY

				case NO_STRAIGHT_ON:
					turnType = pkg.NO_ENTRY

				case NO_U_TURN: // harus NO_ENTRY karena gak boleh u-turn: example: https://www.openstreetmap.org/relation/10732316#map=19/-7.566370/110.775455
					turnType = pkg.NO_ENTRY

				case NO_ENTRY:
					turnType = pkg.NO_ENTRY

				case ONLY_LEFT_TURN:
					turnType = pkg.LEFT_TURN

				case ONLY_RIGHT_TURN:
					turnType = pkg.RIGHT_TURN
				case ONLY_STRAIGHT_ON: // udah kita dissalow semua right & left turn di loc diatas
					turnType = pkg.NONE
				default:
					turnType = pkg.NONE
				}

				if !restriction.conditional {
					turnMatrices[via][rowOffset+exitPoint] = turnType
				} else {

					ctr := da.NewConditionalTurnRestriction(u, via, w, make([]da.Index, 0), false, restriction.timeRangeVal,
						turnType)
					*conditionalTurnRestrictions = append(*conditionalTurnRestrictions, ctr)

				}
			}
			break
		}
	}
}

// addParallelViaEdges. add parallel via-edges for each via-ways
func addParallelViaEdges[W util.RoutingNumber](p *Extractor[W], wayId int64, way osmWay, newDataId int, outEdges [][]da.Index, inEdges [][]da.Index,
	entryPointsAdjList, exitPointsAdjList [][]da.Index, rn *da.RoadNetworkDataContainer,
	segmentDataIds [][]da.Index, outWeights [][]W, outLengths, inLengths [][]uint32, outDegree, inDegree []int, isParallelOutEdge, isParallelInEdge [][]bool,
) int {
	fromRestrictions := p.restrictions[wayId]
	for _, restriction := range fromRestrictions {
		if restriction.isWay {
			/*
								turn restriction berbentuk: {from-way, via-way, to-way} atau {from-way, via-ways, to-way}
								di kode ini:
								wayId/way: from-way
								restriction.to: to-way
								restriction.viaWays: via-way atau via-ways


								contoh: https://www.openstreetmap.org/relation/15268026
								saat ini kita cuma support viaway yang cuma punya 2 nodes (biasanya yang tipe restriction nya u-turn kaya contoh diatas).

								masalahnya:
								Turn Table nya Customizable Route Planning (CRP): https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
								cuma suppport turn cost dari entryPoint i dari vertex u ke exitPoint j, atau
								T[u][i,j] = turn cost dari via vertex u dari entryPoint i (-th inEdge yang head nya u) ke exitPoint j (j-th outEdge yang tailnya u).

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

								kita bikin parallel edge dari via-way (atau parallel edges dari via-ways), contoh:
								https://www.openstreetmap.org/relation/15268026
								tambah 1 edge dari via-way: https://www.openstreetmap.org/way/1131069658
								jadi ada dua edge parallel dari via-way diatas...
								yang satu khusus diakses oleh from-way dari u-turn restriction dan satunya bisa diakses oleh other edges kecuali edge dari from-way....

								ilustrasi:

									|
									| Jalan Subali Raya
									|
								    \/
								----------->		 Jalan Siliwangi ke arah timur (from-edge)
										   |\
							via-edge 1	   | \ via-edge 2 (parallel dengan via-edge 1)
										   |  |
										   | /
										   |/
										   \/
				      		   <------------ Jalan Siliwangi ke arah barat (to-edge)



								nah dari from-edge (https://www.openstreetmap.org/way/1131069660) ke via-edge 1 dikasih turn cost INF dan ke via-edge 2 dikasih turn cost 0...
								dari edges selain from-edge ke via-edge 2 dikasih turn cost INF dan ke via-edge 1 dikasih turn cost 0...
								dari via-edge 2 ke to-edge dikasih turn cost INF (karena OSM u-turn restriction diatas).. biar dari  Jalan Siliwangi ke arah timur (from-edge 2) -> via-edge 2 -> to-edge gabisa lewat...
								dari via-edge 1 ke to-edge dikasih turn cost 0... biar dari jalan selain from-edge -> via-edge 1 -> to-edge bisa lewat ...

								https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-6.987043%2C110.366592%3B-6.988103%2C110.366536#map=19/-6.987536/110.367400
								https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-6.98786%2C110.366571%3B-6.988103%2C110.366536#map=19/-6.987839/110.367400


								inspired by how to handle polyvalent turn dari paper CRP dan https://github.com/Project-OSRM/osrm-backend/issues/2681
			*/ //  nolint: gofmt

			fromNodes := way.graphNodes
			viaWays := restriction.viaWays // bisa aja via-way turn restriction, via-way nya ada lebih dari 1: https://www.openstreetmap.org/relation/17842412
			for q, via := range viaWays {
				var toNodes []da.Index

				if q == len(viaWays)-1 {
					// this via-way via is end of via-ways
					toNodes = p.ways[restriction.to].graphNodes
				} else {
					// this via-way via belum di end of via-ways
					toNodes = p.ways[viaWays[q+1]].graphNodes
				}

				viaWayNodes := p.ways[via].graphNodes
				viaWayNodesSet := make(map[da.Index]struct{}, len(viaWayNodes))
				for i := 0; i < len(viaWayNodes); i++ {
					viaWayNodesSet[viaWayNodes[i]] = struct{}{}
				}

				tail := da.Index(math.MaxUint32) // tail dari via-way via saat ini
				head := da.Index(math.MaxUint32) // head dari via-way via saat ini

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

				viaExitPoint := 0 // index of outgoing via-edge in outgoing edges of tail
				viaDataId := da.Index(da.INVALID_SEGMENT_ID)
				for tailExitPoint, eHead := range outEdges[tail] {
					cViaDataId := segmentDataIds[tail][tailExitPoint]
					eHead := eHead
					if rn.GetOsmWayId(cViaDataId) == uint64(via) && eHead == head &&
						!isParallelOutEdge[tail][tailExitPoint] {
						// harus bukan parallel via-edge juga di via-edge ini
						// karena kalau misal ada banyak via-way turn restrictions yang lewat this via-edge (atau via-way), di akhir cuma ada 1 parallel via-edge dari via-edge ini
						// biar this semua turn restrictions yang lewat this via-edge (atau via-way) ini, bisa diarahin ke 1 parallel via-edge aja yang udah incorporate semua via-way turn restrictions nya...

						viaDataId = cViaDataId
						viaExitPoint = tailExitPoint
						break
					}
				}

				if viaDataId == da.INVALID_SEGMENT_ID {
					continue
				}

				// add via parallel edge
				headEntryPoint := da.Index(len(inEdges[head]))
				tailExitPoint := da.Index(len(outEdges[tail]))
				entryPointsAdjList[tail] = append(entryPointsAdjList[tail], headEntryPoint)
				exitPointsAdjList[head] = append(exitPointsAdjList[head], tailExitPoint)

				// add via parallel outgoing edge
				outEdges[tail] = append(outEdges[tail], head)
				isParallelOutEdge[tail] = append(isParallelOutEdge[tail], true)
				outDegree[tail]++
				outWeights[tail] = append(outWeights[tail], outWeights[tail][viaExitPoint])
				outLengths[tail] = append(outLengths[tail], outLengths[tail][viaExitPoint])
				segmentDataIds[tail] = append(segmentDataIds[tail], da.Index(newDataId))

				// add via parallel incoming edge
				inEdges[head] = append(inEdges[head], tail)
				inLengths[head] = append(inLengths[head], outLengths[tail][viaExitPoint])
				isParallelInEdge[head] = append(isParallelInEdge[head], true)
				inDegree[head]++

				// add via parallel edge annotation data
				isRoundabout := rn.IsRoundabout(viaDataId)
				if isRoundabout {
					rn.SetSegmentBit(da.Index(newDataId), da.FlagIsRoundabout, true)
				}
				sPoint, ePoint := rn.GetSegmentGeometryEndpoints(viaDataId)
				rn.AppendSegmentData(
					int64(rn.GetOsmWayId(viaDataId)),
					sPoint, ePoint,
					rn.GetStreetNameId(viaDataId),
					rn.GetRoadClass(viaDataId),
					rn.GetRoadClassLink(viaDataId),
					rn.GetRoadLanes(viaDataId),
					rn.GetTurnLaneData(viaDataId),
				)
				rn.SetSegmentBit(da.Index(newDataId), da.FlagParallel, true)

				// add via-node turn restriction dari this via-edge to this new parallel via edge

				newDataId++
				fromNodes = viaWayNodes
			}
		}
	}
	return newDataId
}

// addViaWayTurnRestriction.  handler via-way turn restriction (dan multiple via-ways turn restriction)
// restriction is one via-way (or via-ways) turn restriction from wayId
func addViaWayTurnRestriction[W util.RoutingNumber](p *Extractor[W], wayId int64, way osmWay, fromNodes []da.Index, restriction restriction, fromResId int, outEdges [][]da.Index,
	inEdges [][]da.Index, rn *da.RoadNetworkDataContainer, segmentDataIds [][]da.Index, turnMatrices [][]pkg.TurnType, outDegree, inDegree []int, isParallelOutEdge, isParallelInEdge [][]bool) {
	if !IsNotAllowedToTurnType(restriction.turnRestriction) || restriction.conditional {
		// currently only support via-way turn restriction yang no_* (no_left_turn, no_u_turn, etc.)
		// currently gak support conditional via-way turn restriction  (30 april 2026).
		return
	}

	/*
						turn restriction berbentuk: {from-way, via-way, to-way} atau {from-way, via-ways, to-way}
						di kode ini:
						wayId/way: from-way
						restriction.to: to-way
						restriction.viaWays: via-way atau via-ways


						contoh: https://www.openstreetmap.org/relation/15268026
						saat ini kita cuma support viaway yang cuma punya 2 nodes (biasanya yang tipe restriction nya u-turn kaya contoh diatas).

						masalahnya:
						Turn Table nya Customizable Route Planning (CRP): https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
						cuma suppport turn cost dari entryPoint i dari vertex u ke exitPoint j, atau
						T[u][i,j] = turn cost dari via vertex u dari entryPoint i (-th inEdge yang head nya u) ke exitPoint j (j-th outEdge yang tailnya u).

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

						kita bikin parallel edge dari via-way (atau parallel edges dari via-ways), contoh:
						https://www.openstreetmap.org/relation/15268026
						tambah 1 edge dari via-way: https://www.openstreetmap.org/way/1131069658
						jadi ada dua edge parallel dari via-way diatas...
						yang satu khusus diakses oleh from-way dari u-turn restriction dan satunya bisa diakses oleh other edges kecuali edge dari from-way....

						ilustrasi:

							|
							| Jalan Subali Raya
							|
						    \/
						----------->		 Jalan Siliwangi ke arah timur (from-edge)
								   |\
					via-edge 1	   | \ via-edge 2 (parallel dengan via-edge 1)
								   |  |
								   | /
								   |/
								   \/
		      		   <------------ Jalan Siliwangi ke arah barat (to-edge)



						nah dari from-edge (https://www.openstreetmap.org/way/1131069660) ke via-edge 1 dikasih turn cost INF dan ke via-edge 2 dikasih turn cost 0...
						dari edges selain from-edge ke via-edge 2 dikasih turn cost INF dan ke via-edge 1 dikasih turn cost 0...
						dari via-edge 2 ke to-edge dikasih turn cost INF (karena OSM u-turn restriction diatas).. biar dari  Jalan Siliwangi ke arah timur (from-edge 2) -> via-edge 2 -> to-edge gabisa lewat...
						dari via-edge 1 ke to-edge dikasih turn cost 0... biar dari jalan selain from-edge -> via-edge 1 -> to-edge bisa lewat ...

						https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-6.987043%2C110.366592%3B-6.988103%2C110.366536#map=19/-6.987536/110.367400
						https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-6.98786%2C110.366571%3B-6.988103%2C110.366536#map=19/-6.987839/110.367400


						inspired by how to handle polyvalent turn dari paper CRP dan https://github.com/Project-OSRM/osrm-backend/issues/2681
	*/ //  nolint: gofmt
	// bisa aja via-way turn restriction, via-way nya ada lebih dari 1: https://www.openstreetmap.org/relation/17842412

	viaWays := restriction.viaWays         // list of via-ways from this (from-way, via-ways, to-way) turn restriction
	moreThanOneViaWays := len(viaWays) > 1 //  if turn restriction have more than one via-ways
	for q, via := range viaWays {

		var (
			toNodes          []da.Index
			lastViaway       bool
			prevEdgeParallel bool // flag that true if this prevoius edge from via way is an parallel edge
		)
		if q == len(viaWays)-1 {
			// last via-way
			// then toNodes is the to-way.graphNodes
			toNodes = p.ways[restriction.to].graphNodes
			lastViaway = true

			if moreThanOneViaWays {
				// if more than oneViaWay, then the previous edge from this via way is parallel edge
				prevEdgeParallel = true
			}
		} else {
			// not the last via-way
			// then toNodes is the next via-way[q+1].graphNodes
			toNodes = p.ways[viaWays[q+1]].graphNodes

			if q > 0 {
				// this is the second,third,...,last via-way, then the previous edge from this via way is parallel edge
				prevEdgeParallel = true
			}
		}

		viaWayNodes := p.ways[via].graphNodes

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

		viaParallelExitPoint := da.Index(da.INVALID_SEGMENT_ID) // exit point dari this current via parallel edge. exit point = indeks dari via parallel edge di list of outgoing edges dari tail dari via parallel edge.
		for i := 0; i < len(outEdges[tail]); i++ {
			eDataId := segmentDataIds[tail][i]
			eHead := outEdges[tail][i]

			if rn.GetOsmWayId(eDataId) == uint64(via) && eHead == head && isParallelOutEdge[tail][i] {
				viaParallelExitPoint = da.Index(i)
			}
		}

		if viaParallelExitPoint == da.INVALID_EXIT_POINT {
			continue
		}

		viaParallelEntryPoint := da.Index(math.MaxUint32) // entry point dari this current via parallel edge. entry point = indeks dari via parallel edge di list of incoming edges dari head dari via parallel edge.
		for i := 0; i < len(inEdges[head]); i++ {
			eTail := inEdges[head][i]

			if eTail == tail && isParallelInEdge[head][i] {
				viaParallelEntryPoint = da.Index(i)
			}
		}

		if viaParallelEntryPoint == math.MaxUint32 {
			continue
		}

		for i := 0; i < len(fromNodes); i++ {

			if fromNodes[i] == tail {
				if i == 0 && way.oneWay {
					// no predecessor
					continue
				}

				var predecessor = da.Index(math.MaxUint32) // predecessor dari tail dari via-edge nya turn restriction
				if i == 0 {
					// note that di osm_parser.go , way bisa two-way (dua arah) yang mana setiap pasang junction/end node dari osm way kita pecah jadi dua edges (kalau two-way)...
					// kalau via node dari turn restriction di first node dari from-way..
					// berarti predecessor nya ada di next node (i+1).. ke arah forward..
					// from-way nodes: via<->u2<->u3<->-.....<->un
					predecessor = fromNodes[i+1]
				} else {
					predecessor = fromNodes[i-1]
				}

				if predecessor == head && predecessor != da.Index(math.MaxUint32) {
					continue
				}

				successor := da.Index(math.MaxUint32) // successor dari head dari via-edge nya turn restriction

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
					// only store via-way (or via-ways) turn restrictions if successor node of head of this current via-edge exists
					tailEntryPoint := da.Index(math.MaxUint32) // entry point dari incoming from-edge (predecessor, tail)

					for k := da.Index(0); k < da.Index(len(inEdges[tail])); k++ {
						inEdgeTail := inEdges[tail][k]
						parallel := isParallelInEdge[tail][k]
						validInEdge := ((prevEdgeParallel && parallel) ||
							(!prevEdgeParallel && !parallel))

						if inEdgeTail == predecessor && validInEdge {
							tailEntryPoint = k
						} else {
							// kasih turn cost INF buat transisi dari  all other inEdges dari tail (selain from-edge) -> tail -> via-edge 2
							turnMatrices[tail][k*da.Index(outDegree[tail])+viaParallelExitPoint] = pkg.NO_ENTRY
						}
					}

					if tailEntryPoint == da.Index(math.MaxUint32) {
						continue
					}

					viaEdgeExitPoint := da.Index(math.MaxUint32) // exit point dari outgoing via-edge (tail, head) that not the parallel via-edge
					for k := 0; k < len(outEdges[tail]); k++ {
						outEdgeHead := outEdges[tail][k]
						validOutEdge := !isParallelOutEdge[tail][k]
						if outEdgeHead == head && validOutEdge {

							viaEdgeExitPoint = da.Index(k)
							break
						}
					}

					if viaEdgeExitPoint == da.Index(math.MaxUint32) {
						continue
					}

					rowOffset := tailEntryPoint * da.Index(outDegree[tail]) // rowOffset di flatenned 1-d matrix from incoming from-edge

					// kasih turn cost 0 buat transisi dari from-edge -> tail -> parallel via-edge 2
					turnMatrices[tail][rowOffset+viaParallelExitPoint] = pkg.NONE

					// kasih turn cost INF buat transisi dari from-edge -> tail -> via-edge 1
					turnMatrices[tail][rowOffset+viaEdgeExitPoint] = pkg.NO_ENTRY

					// so that all route coming from this from-edge routed to parallel via-edge 2, that in the end will not routed to to-edge of this via-way (or via-ways) turn restriction

					if lastViaway {
						// add turn cost to to-edge of this via-way (or via-ways) turn restrictions

						// dari parallel via-edge 2 ke to-edge assign turn cost INF (karena OSM u-turn restriction diatas).. so that route from from-edge -> parallel via-edge 2 -> to-edge forbidden...
						// dari via-edge 1 ke to-edge assign turn cost 0... so that route from all other edge (except from-edge) -> via-edge 1 -> to-edge can pass through ...
						headExitPoint := da.Index(math.MaxUint32) // exit point dari outgoing to-edge (h, successor)
						for k := 0; k < len(outEdges[head]); k++ {
							outEdgeHead := outEdges[head][k]
							if outEdgeHead == successor && !isParallelOutEdge[head][k] {
								// cek parallel edge
								headExitPoint = da.Index(k)

								break
							}
						}

						if headExitPoint == da.Index(math.MaxUint32) {
							continue
						}

						rowOffset = viaParallelEntryPoint * da.Index(outDegree[head]) // rowOffset of 1-d flattened 2-d matrix.

						// give forbidden turn from parallel via-edge 2 -> to-edge
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

func setConditionalRestrictions[W util.RoutingNumber](p *Extractor[W], roadNetwork bool, graph *da.Graph, rn *da.RoadNetworkDataContainer, segmentDataIds [][]da.Index,
	conditionalTurnRestrictions []da.ConditionalTurnRestriction,
) {
	conditionalReversibleEdges := make([]da.ConditionalReversibleEdge, 0)
	conditionalSpeedLimits := make([]da.ConditionalSpeedLimit, 0)
	conditionalTrafficModesVal := make([]da.ConditionalTrafficMode, 0)
	if roadNetwork {
		graph.ForOutEdges(func(exitPoint, head, tail, entryPoint da.Index, percentage float64, eId da.Index) {
			eDataId := segmentDataIds[tail][exitPoint]
			eWayId := rn.GetOsmWayId(eDataId)
			reversibleVal := p.conditionalReversibleWayVals[int64(eWayId)]
			if reversibleVal != "" {
				cre := da.NewConditionalReversibleEdge(eId, reversibleVal)
				conditionalReversibleEdges = append(conditionalReversibleEdges, cre)
			}

			speedLimitVal, ok := p.conditionalSpeedLimits[int64(eWayId)]
			if ok {
				csl := da.NewConditionalSpeedLimit(eId, speedLimitVal)
				conditionalSpeedLimits = append(conditionalSpeedLimits, csl)
			}

			tfmVal, ok := p.conditionalTrafficModesVal[int64(eWayId)]
			if ok {
				tfm := da.NewConditionalTrafficMode(eId, tfmVal)
				conditionalTrafficModesVal = append(conditionalTrafficModesVal, tfm)
			}
		})
	}

	rn.SetConditionalBarrierNodes(p.conditionalBarrierNodes)
	rn.SetConditionalReversibleEdges(conditionalReversibleEdges)
	rn.SetConditionalSpeedLimits(conditionalSpeedLimits)
	rn.SetConditionalTrafficModes(conditionalTrafficModesVal)
	rn.SetConditionalTurnRestrictions(conditionalTurnRestrictions)

}
