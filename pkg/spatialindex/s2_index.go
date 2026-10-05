package spatialindex

import (
	"fmt"

	"github.com/golang/geo/s2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

// inspired by https://github.com/buckhx/gofence/blob/master/geofence/s2.go
// described in this cool article: https://medium.com/@buckhx/unwinding-uber-s-most-efficient-service-406413c5871d

/*
S2RoadSegmentsIndex  for retrieving road segments inside a s2 level-15 cell & its level-15 cell neighbors like described in: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
*/
type S2RoadSegmentsIndex struct {
	idx map[s2.CellID][]da.Index
}

func NewS2RoadSegmentsIndex(g *da.Graph, rn *da.RoadNetworkDataContainer, log *zap.Logger) *S2RoadSegmentsIndex {

	coverer := NewFlatCoverer(15)
	log.Sugar().Infof("building s2 cell road segments index....")
	idx := make(map[s2.CellID][]da.Index, 1000)
	g.ForVertices(func(_ da.Vertex, segId da.Index) {
		geom := rn.GetSegmentGeometry(segId)
		coords := make([]s2.LatLng, len(geom))
		for i := 0; i < len(coords); i++ {
			c := geom[i]
			coords[i] = s2.LatLngFromDegrees(c.GetLat(), c.GetLon())
		}
		polyline := s2.PolylineFromLatLngs(coords)
		cids := coverer.Covering(polyline)
		for _, cid := range cids {
			idx[cid] = append(idx[cid], segId)
		}
	})
	log.Sugar().Infof("s2 cell road segments index built....")
	return &S2RoadSegmentsIndex{idx: idx}
}

/*
GetCellSegments find road segments inside s2 cell with id=s2CellId and inside its neighbor cells .
*/
func (si *S2RoadSegmentsIndex) GetCellSegments(s2CellId s2.CellID) []da.Index {

	qRes := si.idx[s2CellId]
	// get query results
	segIds := make([]da.Index, 0, len(qRes)*3)
	segIds = append(segIds, qRes...)

	nCells := s2CellId.EdgeNeighbors()
	for _, c := range nCells {
		qRes = si.idx[c]
		segIds = append(segIds, qRes...)
	}

	return segIds
}

// FlatCoverer Embeds a s2.RegionCover, but does it's own covering
// Pick the deepest level and normalize a cellunion
type FlatCoverer struct {
	rc *s2.RegionCoverer
}

func NewFlatCoverer(level int) *FlatCoverer {
	return &FlatCoverer{rc: &s2.RegionCoverer{
		MinLevel: level,
		MaxLevel: level,
		LevelMod: 0,
		MaxCells: 1 << 16,
	}}
}

func (c *FlatCoverer) Covering(r s2.Region) s2.CellUnion {
	cids := c.rc.Covering(r)
	cids.Normalize()
	return cids
}

func (si *S2RoadSegmentsIndex) WriteToFile() error {
	root := config.ProfilesRoot()
	base := fmt.Sprintf("%s/%s/%s", root, pkg.ProfileName, pkg.RegionName)
	filepath := base + ".nsidx"
	return util.WriteCompressedFile(filepath, func(w *util.BinaryWriter) error {
		n := len(si.idx)
		err := w.Uint32(uint32(n))
		if err != nil {
			return err
		}
		for key, val := range si.idx {
			err = w.Uint64(uint64(key))
			if err != nil {
				return err
			}
			vv := make([]uint32, len(val))
			for i := 0; i < len(val); i++ {
				vv[i] = uint32(val[i])
			}
			err = w.WriteUint32s(vv)
			if err != nil {
				return err
			}
		}
		return nil
	})
}

func ReadS2RoadSegmentsIndexFromFile() (*S2RoadSegmentsIndex, error) {
	root := config.ProfilesRoot()
	base := fmt.Sprintf("%s/%s/%s", root, pkg.ProfileName, pkg.RegionName)
	filepath := base + ".nsidx"
	file, r, err := util.OpenCompressedFile(filepath)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	n, err := r.Uint32()
	if err != nil {
		return nil, fmt.Errorf("failed to read number of s2 cellId: %v", err)
	}
	idx := make(map[s2.CellID][]da.Index, n)
	for i := uint32(0); i < n; i++ {
		key, err := r.Uint64()
		if err != nil {
			return nil, fmt.Errorf("failed to read s2 cellId key: %v", err)
		}
		val, err := r.ReadUint32s()
		if err != nil {
			return nil, fmt.Errorf("failed to read s2 cellId segIds: %v", err)
		}
		segIds := make([]da.Index, len(val))
		for j := 0; j < len(val); j++ {
			segIds[j] = da.Index(val[j])
		}
		idx[s2.CellID(key)] = segIds
	}

	sidx := &S2RoadSegmentsIndex{idx: idx}
	return sidx, nil
}

// solusi2: crazy 1.6 gb query engine memory usage
// /*
// S2RoadSegmentsIndex  for retrieving road segments inside a s2 cell & its cell neigbors like described in: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
// */
// type S2RoadSegmentsIndex struct {
// 	idx *s2.ShapeIndex
// }

// func NewS2RoadSegmentsIndex(g *da.Graph, rn *da.RoadNetworkDataContainer, log *zap.Logger) *S2RoadSegmentsIndex {
// 	log.Sugar().Infof("building s2 cell road segments index....")
// 	idx := s2.NewShapeIndex()
// 	g.ForVertices(func(_ da.Vertex, segId da.Index) {
// 		geom := rn.GetSegmentGeometry(segId)
// 		coords := make([]s2.LatLng, len(geom))
// 		for i := 0; i < len(coords); i++ {
// 			c := geom[i]
// 			coords[i] = s2.LatLngFromDegrees(c.GetLat(), c.GetLon())
// 		}
// 		polyline := s2.PolylineFromLatLngs(coords)
// 		idx.Add(polyline)
// 	})
// 	idx.Build()
// 	log.Sugar().Infof("s2 cell road segments index built....")
// 	return &S2RoadSegmentsIndex{idx: idx}
// }

// /*
// GetCellSegments find road segments inside s2 cell with id=s2CellId and inside its neighbor cells .
// https://s2geometry.io/devguide/s2closestedgequery
// https://s2geometry.io/devguide/s2shapeindex
// https://s2geometry.io/devguide/cpp/quickstart
// https://pkg.go.dev/github.com/golang/geo/s2#example-EdgeQuery.FindEdges-FindClosestEdges
// https://pkg.go.dev/github.com/golang/geo/earth
// */
// func (si *S2RoadSegmentsIndex) GetCellSegments(s2CellId s2.CellID) []da.Index {
// 	// build query object
// 	dist := 1 * unit.Meter
// 	distAngle := earth.AngleFromLength(dist)
// 	md := s1.ChordAngleFromAngle(distAngle)
// 	opts := s2.NewClosestEdgeQueryOptions().MaxResults(math.MaxUint32).DistanceLimit(md)
// 	q := s2.NewClosestEdgeQuery(si.idx, opts)
// 	cell := s2.CellFromCellID(s2CellId)

// 	// query
// 	target := s2.NewMinDistanceToCellTarget(cell)
// 	qRes := q.FindEdges(target)
// 	// cell neighbors
// 	nCells := cell.ID().EdgeNeighbors()
// 	for _, c := range nCells {
// 		cell = s2.CellFromCellID(c)
// 		target = s2.NewMinDistanceToCellTarget(cell)
// 		qRes = append(qRes, q.FindEdges(target)...)
// 	}

// 	// get query results
// 	segIds := make([]da.Index, len(qRes))
// 	for i, res := range qRes {
// 		id := res.ShapeID()
// 		segIds[i] = da.Index(id)
// 	}

// 	return segIds
// }
