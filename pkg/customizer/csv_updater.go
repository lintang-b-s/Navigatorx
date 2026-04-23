package customizer

import (
	"encoding/csv"
	"fmt"
	"os"
	"sort"
	"strconv"
	"strings"

	"github.com/cockroachdb/errors"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
referensi: https://github.com/Telenav/open-source-spec/blob/master/osrm/doc/osrm_customization.md

format file csv ngikutin: https://github.com/Project-OSRM/osrm-backend/wiki/Traffic
tapi kita gak ada istilah rate..
langsung pakai weight aja (travel time) in seconds..

format file csv, jika osm way two-way:
from_forward_osm_id, to_forward_osm_id, edge_weight
from_backward_osm_id, to_backward_osm_id, edge_weight

kalau osm way oneway tinggal supply forward aja.
sama kaya osrm, direction (from, to) forward harus sesuai urutan nodes dari osm way dan merupakan JUNCTION_NODE/END_NODE ....
JUNCTION_NODE= osm node yang jadi junction dari 2 osm way atau lebih
END_NODE = osm node yang berada diurutan pertama/terakhir dari osm way dan bukan merupakan JUNCTION

contoh: https://www.openstreetmap.org/way/1215908604#map=18/-7.569491/110.819618
expand Nodes nya, dari urutan forward adalah dari atas ke bawah. kalau osm way two-way backwardnya tinggal dari bawah ke atas.


oke sekarang kita masuk ke logic buat update weight dari edges dan  update shortcut weights nya:
1. buat find corresponding edge (from,to) dari (from_forward_osm_id, to_forward_osm_id) kita butuh LookupTable kaya punya osrm
kita bikin VerticesLookupTable: mapping dari vertex id ke osm id (cuma slice tapi sorted by value/osmId)..
2. buat find segment (from_forward_osm_id, to_forward_osm_id) kita tinggal binary search di sorted lookuptable nya
get index/vertex id dari from_forward_osm_id terus coba cek satu persatu outEdge nya kalau osmwayId dari head nya sama dengan to_forward_osm_id return edge nya (atau length nya doang?)
buat backward edge tinggal dari to_backward_osm_id terus cek satu persatu outEdge nya (forward dan backward edge dari two-way osm way ada dua outEdge, note that inEdge hanyalah outEdge tapi arah traverse nya dari head ke tail).
3. kita harus bikin edgeSpeeds: map dari outEdgeId/exitId ke maxSpeed dari edgenya
karena kita udah dapet data edge yang mau diupdate kita tinggal update corresponding speed nya di edgeSpeeds slice..
tapi pas update pastikan pakai write lock. dan read ke edgeSpeeds pakai read lock..

setelah itu kita tinggal jalanin customizer.Build()

4. karena hasil dari customize diwrite ke file metrics...
kita harus bikin background worker (goroutine dengan inf for loop) yang ngeread apakah file metrics berubah (dari modified time nya)..
kalau berubah kita read dan swap metrics (shortcuts weight) yang ada di memory + ada write lock nya

oke gitu doang

todo: add background worker buat update conditional turn restriction & conditional barrier restriction 
contoh conditional barrier restriction: https://www.openstreetmap.org/node/10303116750
*/

// LookupTable. buat simpan mapping dari osmNodeId -> graph vertexId
type LookupTable[T comparable] struct {
	data             []T
	newToOldPosition []int
	less             func(a, b T) bool
}

// NewNewLookupTable. bikin LookupTable dengan tipe generic T.
// less = if a<b return true
func NewLookupTable[T comparable](data []T, less func(a, b T) bool) *LookupTable[T] {

	n := len(data)
	newToOldPosition := make([]int, n)
	for i := 0; i < n; i++ {
		newToOldPosition[i] = i
	}

	sort.Slice(newToOldPosition, func(i, j int) bool {
		return less(data[newToOldPosition[i]], data[newToOldPosition[j]])
	})

	sortedData := make([]T, n)
	for i := 0; i < n; i++ {
		sortedData[i] = data[newToOldPosition[i]]
	}

	lt := &LookupTable[T]{
		data:             sortedData,
		newToOldPosition: newToOldPosition,
		less:             less,
	}
	return lt
}

// Get. get graph vertex id given key osmNodeId.
// O(logn) binary search
func (lt *LookupTable[T]) Get(key T) int {
	n := len(lt.data)

	l := 0
	r := n - 1

	for l <= r {
		mid := l + (r-l)/2
		if lt.data[mid] == key {
			return lt.newToOldPosition[mid]
		} else if lt.less(key, lt.data[mid]) {
			r = mid - 1
		} else {
			l = mid + 1
		}
	}

	return INVALID_LOOKUPTABLE_VAL_ID
}

func (c *Customizer) readEdgeSpeedsFromFile(filepath string) ([]da.Index, []float64, error) {
	f, err := os.Open(filepath)
	if err != nil {
		return make([]da.Index, 0), make([]float64, 0), errors.Wrapf(err, "customizer.readEdgeSpeedsFile: failed to open file %v", filepath)
	}

	defer f.Close()

	csvReader := csv.NewReader(f)
	data, err := csvReader.ReadAll()
	if err != nil {
		return make([]da.Index, 0), make([]float64, 0), errors.Wrapf(err, "customizer.readEdgeSpeedsFile: failed to readAll csv data")
	}

	n := len(data)
	updatedEdges := make([]da.Index, 0, n)
	updatedEdgeSpeeds := make([]float64, 0, n)
	for rowId := 0; rowId < n; rowId++ {
		row := data[rowId]
		fromOsmIdString := strings.TrimSpace(row[0])
		fromOsmId, err := util.ParseUInt64(fromOsmIdString)
		if err != nil {
			return make([]da.Index, 0), make([]float64, 0), errors.Wrapf(err, "customizer.readEdgeSpeedsFile: failed to parse uint64 fromOsmId: %s", fromOsmIdString)
		}
		toOsmIdString := strings.TrimSpace(row[1])
		toOsmId, err := util.ParseUInt64(toOsmIdString)
		if err != nil {
			return make([]da.Index, 0), make([]float64, 0), errors.Wrapf(err, "customizer.readEdgeSpeedsFile: failed to parse uint64 toOsmId: %s", toOsmIdString)
		}

		fromVId := c.verticesLookupTable.Get(fromOsmId)
		if fromVId == INVALID_LOOKUPTABLE_VAL_ID {
			c.logger.Sugar().Warnf("no edge found from %v to %v", fromOsmId, toOsmId)
			continue
		}
		toVId := c.verticesLookupTable.Get(toOsmId)
		if toVId == INVALID_LOOKUPTABLE_VAL_ID {
			c.logger.Sugar().Warnf("no edge found from %v to %v", fromOsmId, toOsmId)
			continue
		}

		updatedEId := da.INVALID_EDGE_ID

		c.graph.ForOutEdgeIdsOf(da.Index(fromVId), func(eId da.Index) {
			head := c.graph.GetHeadOfOutEdge(eId)
			if head == da.Index(toVId) {
				updatedEId = eId
			}
		})

		if updatedEId == da.INVALID_EDGE_ID {
			c.logger.Sugar().Warnf("no edge found from %v to %v ", fromOsmId, toOsmId)
			continue
		}

		updatedEdgeSpeedString := strings.TrimSpace(row[2])
		updatedEdgeSpeed, err := strconv.ParseFloat(updatedEdgeSpeedString, 64)
		if err != nil {
			return make([]da.Index, 0), make([]float64, 0), errors.Wrapf(err, "customizer.readEdgeSpeedsFile: failed to parse segent speed: %s", updatedEdgeSpeedString)
		}

		updatedEdges = append(updatedEdges, updatedEId)
		updatedEdgeSpeed = util.KMHToMSeconds(updatedEdgeSpeed) // convert to m/s
		updatedEdgeSpeeds = append(updatedEdgeSpeeds, updatedEdgeSpeed)
	}

	return updatedEdges, updatedEdgeSpeeds, nil
}

// updatedSegment. satu row di segment speed csv file
// fromOsmId, toOsmId, speed (in km/h).
type UpdatedSegment struct {
	fromOsmId int64
	toOsmId   int64
	speed     float64 // in km/h
}

func NewUpdatedSegment(fromOsmId, toOsmId int64, speed float64) UpdatedSegment {
	return UpdatedSegment{fromOsmId: fromOsmId, toOsmId: toOsmId, speed: speed}
}

// WriteUpdatedSegmentsToCSV. write segment csv file
func WriteUpdatedSegmentsToCSV(filepath string, segments []UpdatedSegment) error {
	f, err := os.Create(filepath)
	if err != nil {
		return errors.Wrapf(err, "WriteUpdatedSegmentsToCSV: failed to create file %v", filepath)
	}
	defer f.Close()

	for _, seg := range segments {
		speedStr := strconv.FormatFloat(seg.speed, 'f', -1, 64)
		_, err := fmt.Fprintf(f, "%d, %d, %s\n", seg.fromOsmId, seg.toOsmId, speedStr)
		if err != nil {
			return errors.Wrapf(err, "WriteUpdatedSegmentsToCSV: failed to write row for segment (%d,%d)",
				seg.fromOsmId, seg.toOsmId)
		}
	}

	return nil
}
