package spatialindex

import (
	"fmt"
	"math"

	"github.com/golang/geo/earth"
	"github.com/golang/geo/s2"
	"github.com/google/go-units/unit"
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
	sidx := &S2RoadSegmentsIndex{}
	sidx.idx = make(map[s2.CellID][]da.Index, 1000)
	g.ForVertices(func(_ da.Vertex, segId da.Index) {
		geom := rn.GetSegmentGeometry(segId)
		sidx.AddPolyline(segId, geom, coverer)
	})
	log.Sugar().Infof("s2 cell road segments index built....")
	return sidx
}

func (si *S2RoadSegmentsIndex) AddPolyline(id da.Index, geom []da.Coordinate, coverer *FlatCoverer) {
	coords := make([]s2.LatLng, len(geom))
	for i := 0; i < len(coords); i++ {
		c := geom[i]
		coords[i] = s2.LatLngFromDegrees(c.GetLat(), c.GetLon())
	}
	polyline := s2.PolylineFromLatLngs(coords)
	cids := coverer.Covering(polyline)
	for _, cid := range cids {
		si.idx[cid] = append(si.idx[cid], id)
	}
}

/*
GetCellSegments return all road segments inside s2 cell with id=s2CellId, inside its neighbor cells, inside all s2 level-15 cells within radius from center point of this cell (in km).
*/
func (si *S2RoadSegmentsIndex) GetCellSegments(s2CellId s2.CellID, radius float64) []da.Index {
	set := make(map[da.Index]bool, len(si.idx[s2CellId]))
	segIds := make([]da.Index, 0, len(si.idx[s2CellId])*4)
	for _, id := range si.idx[s2CellId] {
		if !set[id] {
			set[id] = true
			segIds = append(segIds, id)
		}
	}

	nCells := s2CellId.AllNeighbors(s2CellId.Level()) // return all 8 s2 level-5 cell neighbors
	for _, c := range nCells {
		for _, id := range si.idx[c] {
			if !set[id] {
				set[id] = true
				segIds = append(segIds, id)
			}
		}
	}

	if util.Lt(radius, 0.275) {
		// if radius < 275 meter, immideatly return all road segments inside s2 cell with id=s2CellId, inside its neighbor cells
		return segIds
	}

	// else get all road segments inside other cells within radius from center of s2 cell s2CellId
	cell := s2.CellFromCellID(s2CellId)
	p := cell.Center() // center point of this s2 cell
	nCells = si.GetCellsByRadius(p, radius)
	for _, c := range nCells {
		for _, id := range si.idx[c] {
			if !set[id] {
				set[id] = true
				segIds = append(segIds, id)
			}
		}
	}

	return segIds
}

// GetCellsByRadius get s2 level-15 cells within radius (in km) from center point p
func (si *S2RoadSegmentsIndex) GetCellsByRadius(p s2.Point, radius float64) []s2.CellID {
	surfaceArea := sphericalCapSurfaceArea(radius)
	// https://pkg.go.dev/github.com/golang/geo/s2#RegionCoverer
	cap := s2.CapFromCenterArea(p, surfaceArea)
	rc := &s2.RegionCoverer{MinLevel: 15, MaxLevel: 15, MaxCells: 1 << 16}
	cids := rc.Covering(cap) // dont do any normalize in here
	// https://s2geometry.io/devguide/s2cell_hierarchy.html
	// “normalized”, meaning that groups of 4 child cells have been replaced by their parent cell whenever possible
	return cids
}

// calculate surface area of spherical cap S from arc length (or great circle distance in km) https://pkg.go.dev/github.com/golang/geo/s2#Cap
// ilustration of spherical cap: https://blog.gojek.io/content/images/2021/02/image-339.png (taken from https://www.gojek.io/blog/appreciating-the-geo-s2-library)
// S=2*pi*R*h. where h is the spherical cap height
// derivation (which is just calculating surface area of spherical cap using integral): https://www.youtube.com/watch?v=-5pgU976Kyo&t=521s
// or https://en.wikipedia.org/wiki/Spherical_cap#Deriving_the_volume_and_surface_area_using_calculus
// in s2 geometry the earth is modeled as unit sphere (https://s2geometry.io/about/overview, https://s2geometry.io/devguide/cpp/quickstart.html) (radius R=1)
func sphericalCapSurfaceArea(arcLength float64) float64 {
	l := unit.Length(arcLength) * unit.Kilometer
	r := earth.AngleFromLength(l)
	// https://pkg.go.dev/github.com/golang/geo/s2#Cap
	h := 1 - math.Cos(r.Radians())
	surfaceArea := 2 * math.Pi * h
	return surfaceArea
}

// FlatCoverer Embeds a s2.RegionCover, but does it's own covering
// Pick the deepest level.
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
	cids := c.rc.Covering(r) // dont do any normalize in here
	// https://s2geometry.io/devguide/s2cell_hierarchy.html
	// “normalized”, meaning that groups of 4 child cells have been replaced by their parent cell whenever possible
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
		return nil, fmt.Errorf("failed to read number of s2 cellId: %w", err)
	}
	idx := make(map[s2.CellID][]da.Index, n)
	for i := uint32(0); i < n; i++ {
		key, err := r.Uint64()
		if err != nil {
			return nil, fmt.Errorf("failed to read s2 cellId key: %w", err)
		}
		val, err := r.ReadUint32s()
		if err != nil {
			return nil, fmt.Errorf("failed to read s2 cellId segIds: %w", err)
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
