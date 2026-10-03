package datastructure

import (
	"fmt"

	"github.com/bits-and-blooms/bitset"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (rn *RoadNetworkDataContainer) WriteToFile(filename string) error {
	return util.WriteCompressedFile(filename, func(w *util.BinaryWriter) error {
		return writeRoadNetworkDataContainer(w, rn)
	})
}

func ReadRoadNetworkDataContainer(filename string) (*RoadNetworkDataContainer, error) {
	file, r, err := util.OpenCompressedFile(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()
	return readRoadNetworkDataContainer(r)
}

func writeTurnLanesData(w *util.BinaryWriter, values []TurnLanesData) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	for _, tld := range values {
		if err := w.Length(len(tld.tipeMask)); err != nil {
			return err
		}
		for _, mask := range tld.tipeMask {
			if err := writeBitSet(w, mask); err != nil {
				return err
			}
		}
	}
	return nil
}

func readTurnLanesData(r *util.BinaryReader) ([]TurnLanesData, error) {
	count, err := r.Length()
	if err != nil {
		return nil, err
	}
	values := make([]TurnLanesData, count)
	for i := range values {
		maskCount, err := r.Length()
		if err != nil {
			return nil, err
		}
		masks := make([]*bitset.BitSet, maskCount)
		for j := range masks {
			masks[j], err = readBitSet(r)
			if err != nil {
				return nil, err
			}
		}
		values[i] = TurnLanesData{tipeMask: masks}
	}
	return values, nil
}

func writeRoadNetworkDataContainer(w *util.BinaryWriter, rn *RoadNetworkDataContainer) error {
	if err := w.Length(len(rn.osmNodePoints)); err != nil {
		return err
	}
	for _, point := range rn.osmNodePoints {
		if err := w.Int32(point.GetFixedLat()); err != nil {
			return err
		}
		if err := w.Int32(point.GetFixedLon()); err != nil {
			return err
		}
	}
	if err := w.Uint8(rn.osmwayBitSize); err != nil {
		return err
	}
	if err := writePackedSlice(w, rn.segmentOsmWayId); err != nil {
		return err
	}

	if err := w.WriteUInt64s(rn.osmNodeIds); err != nil {
		return err
	}

	metadataCount := len(rn.segmentStartPointsIndex)
	if len(rn.segmentEndPointsIndex) != metadataCount || len(rn.streetName) != metadataCount ||
		len(rn.roadClass) != metadataCount || len(rn.roadClassLink) != metadataCount || len(rn.lanes) != metadataCount {
		return fmt.Errorf("graph edge metadata lengths do not match")
	}
	if err := w.Length(metadataCount); err != nil {
		return err
	}
	for i := 0; i < metadataCount; i++ {
		if err := w.Uint32(uint32(rn.segmentStartPointsIndex[i])); err != nil {
			return err
		}
		if err := w.Uint32(uint32(rn.segmentEndPointsIndex[i])); err != nil {
			return err
		}
		if err := w.Uint32(rn.streetName[i]); err != nil {
			return err
		}
		if err := w.Uint8(uint8(rn.roadClass[i])); err != nil {
			return err
		}
		if err := w.Uint8(uint8(rn.roadClassLink[i])); err != nil {
			return err
		}
		if err := w.Uint8(rn.lanes[i]); err != nil {
			return err
		}
	}
	if err := w.Length(len(rn.segmentHighwayType)); err != nil {
		return err
	}
	for _, value := range rn.segmentHighwayType {
		if err := w.Uint8(uint8(value)); err != nil {
			return err
		}
	}

	if err := w.Length(len(rn.segmentFlags)); err != nil {
		return err
	}
	for _, value := range rn.segmentFlags {
		if err := w.Uint8(uint8(value)); err != nil {
			return err
		}
	}

	if err := writeTurnLanesData(w, rn.segmentTurnLanesData); err != nil {
		return err
	}
	if err := w.Length(len(rn.nameTable)); err != nil {
		return err
	}
	for _, value := range rn.nameTable {
		if err := w.String(value); err != nil {
			return err
		}
	}

	// ---- conditional turn restrictions related ----
	if err := w.Length(len(rn.conditionalBarrierNodes)); err != nil {
		return err
	}
	for _, value := range rn.conditionalBarrierNodes {
		if err := w.Int64(value.osmNodeId); err != nil {
			return err
		}
		if err := w.String(value.timeRangeVal); err != nil {
			return err
		}
	}
	if err := w.Length(len(rn.conditionalReversibleEdges)); err != nil {
		return err
	}
	for _, value := range rn.conditionalReversibleEdges {
		if err := w.Uint32(uint32(value.edgeId)); err != nil {
			return err
		}
		if err := w.String(value.timeRangeVal); err != nil {
			return err
		}
	}
	if err := w.Length(len(rn.conditionalSpeedLimits)); err != nil {
		return err
	}
	for _, value := range rn.conditionalSpeedLimits {
		if err := w.Uint32(uint32(value.edgeId)); err != nil {
			return err
		}
		if err := w.String(value.timeRangeSpeedVal); err != nil {
			return err
		}
	}
	if err := w.Length(len(rn.conditionalTrafficModes)); err != nil {
		return err
	}
	for _, value := range rn.conditionalTrafficModes {
		if err := w.Uint32(uint32(value.edgeId)); err != nil {
			return err
		}
		if err := w.String(value.timeRangeVal); err != nil {
			return err
		}
	}
	if err := w.Length(len(rn.conditionalTurnRestrictions)); err != nil {
		return err
	}
	for _, value := range rn.conditionalTurnRestrictions {
		for _, id := range []Index{value.fromVId, value.viaVId, value.toVId} {
			if err := w.Uint32(uint32(id)); err != nil {
				return err
			}
		}
		if err := w.Bool(value.viaWay); err != nil {
			return err
		}
		if err := w.Uint8(uint8(value.turnType)); err != nil {
			return err
		}
		if err := writeIndices(w, value.viaEIds); err != nil {
			return err
		}
		if err := w.String(value.timeRangeVal); err != nil {
			return err
		}
	}

	// bounding box related
	for _, value := range []float64{rn.boundingBox.minLat, rn.boundingBox.minLon, rn.boundingBox.maxLat, rn.boundingBox.maxLon} {
		if err := w.Float64(value); err != nil {
			return err
		}
	}

	if err := w.Length(len(rn.segmentH3CellId)); err != nil {
		return err
	}
	for key, value := range rn.segmentH3CellId {
		if err := w.String(key); err != nil {
			return err
		}
		vals := make([]uint32, len(value))
		for i := 0; i < len(value); i++ {
			vals[i] = uint32(value[i])
		}
		if err := w.WriteUint32s(vals); err != nil {
			return err
		}
	}

	return nil
}

func readRoadNetworkDataContainer(r *util.BinaryReader) (*RoadNetworkDataContainer, error) {
	pointCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	points := make([]Coordinate, pointCount)
	if err := r.ReadInt32Pairs(len(points), func(i int, lat, lon int32) {
		points[i] = NewFixedCoordinate(lat, lon)
	}); err != nil {
		return nil, err
	}
	bitSize, err := r.Uint8()
	if err != nil {
		return nil, err
	}
	osmWayIDs, err := readPackedSlice(r)
	if err != nil {
		return nil, err
	}

	osmNodeIds, err := r.ReadUint64s()
	if err != nil {
		return nil, err
	}

	metadataCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	rn := &RoadNetworkDataContainer{
		osmNodePoints:           points,
		osmNodeIds:              osmNodeIds,
		segmentOsmWayId:         osmWayIDs,
		osmwayBitSize:           bitSize,
		segmentStartPointsIndex: make([]Index, metadataCount),
		segmentEndPointsIndex:   make([]Index, metadataCount),
		streetName:              make([]uint32, metadataCount),
		roadClass:               make([]pkg.OsmHighwayType, metadataCount),
		roadClassLink:           make([]pkg.OsmHighwayType, metadataCount),
		lanes:                   make([]uint8, metadataCount),
	}
	for i := 0; i < int(metadataCount); i++ {
		start, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		end, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		rn.segmentStartPointsIndex[i], rn.segmentEndPointsIndex[i] = Index(start), Index(end)
		rn.streetName[i], err = r.Uint32()
		if err != nil {
			return nil, err
		}
		roadClass, err := r.Uint8()
		if err != nil {
			return nil, err
		}
		rn.roadClass[i] = pkg.OsmHighwayType(roadClass)
		roadClassLink, err := r.Uint8()
		if err != nil {
			return nil, err
		}
		rn.roadClassLink[i] = pkg.OsmHighwayType(roadClassLink)
		rn.lanes[i], err = r.Uint8()
		if err != nil {
			return nil, err
		}
	}
	highwayCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	rn.segmentHighwayType = make([]pkg.OsmHighwayType, highwayCount)
	for i := range rn.segmentHighwayType {
		value, err := r.Uint8()
		if err != nil {
			return nil, err
		}
		rn.segmentHighwayType[i] = pkg.OsmHighwayType(value)
	}

	segmentFlagCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	rn.segmentFlags = make([]SegmentFlagType, segmentFlagCount)
	for i := range rn.segmentFlags {
		value, err := r.Uint8()
		if err != nil {
			return nil, err
		}
		rn.segmentFlags[i] = SegmentFlagType(value)
	}

	rn.segmentTurnLanesData, err = readTurnLanesData(r)
	if err != nil {
		return nil, err
	}
	nameCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	rn.nameTable = make([]string, nameCount)
	for i := range rn.nameTable {
		rn.nameTable[i], err = r.String()
		if err != nil {
			return nil, err
		}
	}
	barrierCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	rn.conditionalBarrierNodes = make([]ConditionalBarrierNode, barrierCount)
	for i := range rn.conditionalBarrierNodes {
		rn.conditionalBarrierNodes[i].osmNodeId, err = r.Int64()
		if err != nil {
			return nil, err
		}
		rn.conditionalBarrierNodes[i].timeRangeVal, err = r.String()
		if err != nil {
			return nil, err
		}
	}
	reversibleCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	rn.conditionalReversibleEdges = make([]ConditionalReversibleEdge, reversibleCount)
	for i := range rn.conditionalReversibleEdges {
		id, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		rn.conditionalReversibleEdges[i].edgeId = Index(id)
		rn.conditionalReversibleEdges[i].timeRangeVal, err = r.String()
		if err != nil {
			return nil, err
		}
	}
	speedCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	rn.conditionalSpeedLimits = make([]ConditionalSpeedLimit, speedCount)
	for i := range rn.conditionalSpeedLimits {
		id, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		rn.conditionalSpeedLimits[i].edgeId = Index(id)
		rn.conditionalSpeedLimits[i].timeRangeSpeedVal, err = r.String()
		if err != nil {
			return nil, err
		}
	}
	modeCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	rn.conditionalTrafficModes = make([]ConditionalTrafficMode, modeCount)
	for i := range rn.conditionalTrafficModes {
		id, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		rn.conditionalTrafficModes[i].edgeId = Index(id)
		rn.conditionalTrafficModes[i].timeRangeVal, err = r.String()
		if err != nil {
			return nil, err
		}
	}
	turnCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	rn.conditionalTurnRestrictions = make([]ConditionalTurnRestriction, turnCount)
	for i := range rn.conditionalTurnRestrictions {
		value := &rn.conditionalTurnRestrictions[i]
		ids := []*Index{&value.fromVId, &value.viaVId, &value.toVId}
		for _, target := range ids {
			id, err := r.Uint32()
			if err != nil {
				return nil, err
			}
			*target = Index(id)
		}
		value.viaWay, err = r.Bool()
		if err != nil {
			return nil, err
		}
		turnType, err := r.Uint8()
		if err != nil {
			return nil, err
		}
		value.turnType = pkg.TurnType(turnType)
		value.viaEIds, err = readIndices(r)
		if err != nil {
			return nil, err
		}
		value.timeRangeVal, err = r.String()
		if err != nil {
			return nil, err
		}
	}

	bounds := [4]float64{}
	for i := range bounds {
		bounds[i], err = r.Float64()
		if err != nil {
			return nil, err
		}
	}

	rn.boundingBox = NewBoundingBox(bounds[0], bounds[1], bounds[2], bounds[3])

	h3CellCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	rn.segmentH3CellId = make(map[string][]Index, h3CellCount)
	for range h3CellCount {
		key, err := r.String()
		if err != nil {
			return nil, err
		}
		vals, err := r.ReadUint32s()
		if err != nil {
			return nil, err
		}
		values := make([]Index, len(vals))
		for i := 0; i < len(values); i++ {
			values[i] = Index(vals[i])
		}
		rn.segmentH3CellId[key] = values
	}

	return rn, nil
}
