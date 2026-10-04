// Package mapattributes berisi MapAttributes Engine (see  https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e)
package mapattributes

import (
	"bytes"
	"fmt"

	s2geo "github.com/golang/geo/s2"
	"github.com/klauspost/compress/s2"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

// MapAttributesEngine engine untuk get subset of RoadNetworkGraph yang berada didalam quadKey web mercator tiles. terinspirasi dari MapAttributes service: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
type MapAttributesEngine[W util.RoutingNumber] struct {
	g      *da.Graph
	rn     *da.RoadNetworkDataContainer
	met    *met.Metric[W]
	idx    *spatialindex.S2RoadSegmentsIndex
	logger *zap.Logger
}

func NewMapAttributesEngine[W util.RoutingNumber](graph *da.Graph, rn *da.RoadNetworkDataContainer, logger *zap.Logger, met *met.Metric[W], idx *spatialindex.S2RoadSegmentsIndex) *MapAttributesEngine[W] {

	engine := &MapAttributesEngine[W]{
		g:      graph,
		rn:     rn,
		logger: logger,
		met:    met,
		idx:    idx,
	}

	return engine
}

// // segment segment layer in the mapbox vector tile (https://github.com/mapbox/vector-tile-spec/tree/master/2.1)
// type segment struct {
// 	rnId     da.Index // road segment id in query engine graph.
// 	speed    float64  // speed limit of this road segment in m/s
// 	length   float64  // lenght of this road segment meters
// 	name     string   // street name
// 	geometry string   // geometry of road segment in google polyline string format
// }

// // turn represent turn from road segment e1 to road segment e2
// type turn struct {
// 	weight uint32   // duration of road segment e1 + turn cost of turn (e1,e2). in centiseconds.
// 	u      da.Index //  id of road segment e1
// 	v      da.Index //  id of road segment e2
// }

// GetMapAttributes get MapAttributes by s2 CellId.
func (me *MapAttributesEngine[W]) GetMapAttributes(s2CellId s2geo.CellID) ([]byte, error) {
	// query road segments from s2 index
	segmentIds := me.idx.GetCellSegments(s2CellId)

	buf := &bytes.Buffer{}
	sn := s2.NewWriter(buf)
	bw := util.NewBinaryWriter(sn)
	err := bw.Length(len(segmentIds))
	if err != nil {
		return make([]byte, 0), fmt.Errorf("failed to write uint32: %w", err)
	}

	nt := uint32(0)
	for _, segId := range segmentIds {
		speed := me.met.GetSegmentSpeed(segId)
		length := me.met.GetSegmentLength(segId)
		geom := me.rn.GetSegmentGeometry(segId)
		polyline := da.GooglePoylineFromCoords(geom)
		err = bw.Uint32(uint32(segId))
		if err != nil {
			return make([]byte, 0), fmt.Errorf("failed to write uint32: %w", err)
		}
		err = bw.Float64(speed)
		if err != nil {
			return make([]byte, 0), fmt.Errorf("failed to write float64: %w", err)
		}
		err = bw.Float64(length)
		if err != nil {
			return make([]byte, 0), fmt.Errorf("failed to write float64: %w", err)
		}
		err = bw.String(polyline)
		if err != nil {
			return make([]byte, 0), fmt.Errorf("failed to write string: %w", err)
		}
		nt += uint32(me.g.GetOutDegree(segId))
	}

	err = bw.Uint32(nt)
	if err != nil {
		return make([]byte, 0), fmt.Errorf("failed to write uint32: %w", err)
	}
	for _, u := range segmentIds {
		var err error
		me.g.ForOutEdgesOf(u, func(eId, v, _ da.Index) {
			weight := me.met.GetWeight(eId)
			err = bw.Uint32(uint32(weight))
			err = bw.Uint32(uint32(u))
			err = bw.Uint32(uint32(v))
		})
		if err != nil {
			return make([]byte, 0), fmt.Errorf("failed to write uint32: %w", err)
		}
	}
	err = sn.Close()
	if err != nil {
		return make([]byte, 0), fmt.Errorf("failed to close snappy: %w", err)
	}
	return buf.Bytes(), nil
}

// jangan dihapus.. ini buat tile service aja.
// // GetMapAttributes get MapAttributes by web mercator quadKey and zoom level. also return MapAttributes of 4 cells neighbor of this (quadKey,zoom) cell
// func (me *MapAttributesEngine[W]) GetMapAttributes(s2CellId string) ([]byte, error) {
// 	segmentIds := make([]da.Index, 0, SEGMENTS_SIZE)
// 	segments := make([]segment, 0, SEGMENTS_SIZE)
// 	segmentGeometries := []*da.Coordinates{} // geometry of the road segment in google polyline format
// 	turnGeometries := []*da.Coordinates{}
// 	turns := make([]turn, 0, SEGMENTS_SIZE)

// 	tile := maptile.FromQuadkey(quadKey, maptile.Zoom(zoom))
// 	bound := tile.Bound()
// 	bmLat, bmLon := bound.Max.Y(), bound.Max.X()
// 	qp := tile.Center()
// 	qLat, qLon := qp.Y(), qp.Lat()
// 	radius := geo.CalculateGreatCircleDistance(qLat, qLon, bmLat, bmLon)
// 	segmentIds = append(segmentIds, me.rt.SearchWithinRadius(qLat, qLon, radius, 3)...)

// 	for _, segId := range segmentIds {
// 		geom := me.rn.GetSegmentGeometry(segId)
// 		speed := me.met.GetSegmentSpeed(segId)
// 		duration := me.met.GetWeight(segId)
// 		length := me.met.GetSegmentLength(segId)
// 		geometry := da.NewCoordinatesWithInitialValues(geom)
// 		name := me.rn.GetStreetName(segId)
// 		seg := segment{speed: speed, duration: uint32(duration), length: length,
// 			name: name, rnId: segId}
// 		segmentGeometries = append(segmentGeometries, geometry)
// 		seg.geometry = da.GooglePoylineFromCoords(geom)
// 		segments = append(segments, seg)
// 	}

// 	for _, u := range segmentIds {
// 		tail := me.rn.GetSegmentHeadCoord(u)
// 		prev := me.GetPrevPoint(u, tail, 20)
// 		me.g.ForOutEdgesOf(u, func(eId, v, _ da.Index) {
// 			head := me.GetHeadPoint(v, tail, 20)
// 			prevInitialBearing := geo.ComputeInitialBearing(prev.GetLat(), prev.GetLon(),
// 				tail.GetLat(), tail.GetLon())
// 			turnSign := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon(), prevInitialBearing)
// 			delta := geo.ComputeRelativeBearing(prev.GetLat(), prev.GetLon(), tail.GetLat(), tail.GetLon(), prevInitialBearing)
// 			turnAngle := util.RadiansToDegree(delta)
// 			weight := me.met.GetWeight(eId)
// 			tt := turn{bearing: prevInitialBearing, turnAngle: turnAngle, weight: uint32(weight), turnSign: turnSign, u: u, v: v}
// 			geom := me.rn.GetSegmentGeometry(u)
// 			geometry := da.NewCoordinatesWithInitialValues(geom)
// 			turnGeometries = append(turnGeometries, geometry)
// 			turns = append(turns, tt)
// 		})
// 	}

// 	fcSegments := geojson.NewFeatureCollection()
// 	for i, seg := range segments {
// 		geom := segmentGeometries[i]
// 		points := make([]orb.Point, geom.Length())
// 		for i := range points {
// 			p := geom.Get(i)
// 			points[i] = orb.Point{p.GetLon(), p.GetLat()}
// 		}
// 		ls := orb.LineString(points)
// 		prop := geojson.Properties{}
// 		prop["speed"] = seg.speed
// 		prop["duration"] = seg.duration
// 		prop["length"] = seg.length
// 		prop["name"] = seg.name
// 		prop["rnId"] = seg.rnId
// 		prop["geometry"] = seg.geometry
// 		gf := geojson.NewFeature(ls)
// 		gf.Properties = prop
// 		gf.ID = seg.rnId
// 		gf.Type = "segment"
// 		fcSegments.Append(gf)
// 	}

// 	fcTurns := geojson.NewFeatureCollection()
// 	for i, tt := range turns {
// 		geom := turnGeometries[i]
// 		points := make([]orb.Point, geom.Length())
// 		for j := range points {
// 			p := geom.Get(j)
// 			points[j] = orb.Point{p.GetLon(), p.GetLat()}
// 		}
// 		ls := orb.LineString(points)
// 		prop := geojson.Properties{}
// 		prop["bearing"] = tt.bearing
// 		prop["turnAngle"] = tt.turnAngle
// 		prop["weight"] = tt.weight
// 		prop["turnSign"] = tt.turnSign
// 		prop["u"] = tt.u
// 		prop["v"] = tt.v
// 		gf := geojson.NewFeature(ls)
// 		gf.Properties = prop
// 		gf.Type = "turn"
// 		fcTurns.Append(gf)
// 	}

// 	collections := map[string]*geojson.FeatureCollection{}
// 	collections["segments"] = fcSegments
// 	collections["turns"] = fcTurns

// 	layers := mvt.NewLayers(collections)
// 	layers.ProjectToTile(tile)
// 	return mvt.MarshalGzipped(layers)
// }
