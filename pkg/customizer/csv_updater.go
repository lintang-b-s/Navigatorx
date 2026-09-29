// Package customizer provides tools for updating edge weights and metrics from external sources.
package customizer

import (
	"encoding/csv"
	"fmt"
	"os"
	"strconv"
	"strings"

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

func (c *Customizer[W]) readSegmentSpeedsFromFile(filepath string) ([]da.Index, []float64, error) {
	f, err := os.Open(filepath)
	if err != nil {
		return make([]da.Index, 0), make([]float64, 0), fmt.Errorf("customizer.readEdgeSpeedsFile: failed to open file %v: %w", filepath, err)
	}

	defer f.Close()

	csvReader := csv.NewReader(f)
	data, err := csvReader.ReadAll()
	if err != nil {
		return make([]da.Index, 0), make([]float64, 0), fmt.Errorf("customizer.readEdgeSpeedsFile: failed to readAll csv data: %w", err)
	}

	n := len(data)
	upSegmentIds := make([]da.Index, 0, n)
	upSegmentSpeeds := make([]float64, 0, n)
	for rowId := 0; rowId < n; rowId++ {
		row := data[rowId]
		uStr := strings.TrimSpace(row[0])
		u, err := util.ParseTextUInt64(uStr)
		if err != nil {
			return make([]da.Index, 0), make([]float64, 0), fmt.Errorf("customizer.readEdgeSpeedsFile: failed to parse uint64 fromOsmId: %v: %w", u, err)
		}
		vStr := strings.TrimSpace(row[1])
		v, err := util.ParseTextUInt64(vStr)
		if err != nil {
			return make([]da.Index, 0), make([]float64, 0), fmt.Errorf("customizer.readEdgeSpeedsFile: failed to parse uint64 toOsmId: %v: %w", v, err)
		}

		ebgnId := c.segmentLookupTable.Get(da.NewSegmentKV(u, v, 0))
		if ebgnId == da.INVALID_LK_TABLE_ID {
			c.logger.Sugar().Warnf("no edge found from %v to %v", u, v)
			continue
		}

		upSegmentSpeedStr := strings.TrimSpace(row[2])
		upEdgeSpeed, err := util.ParseTextFloat64(upSegmentSpeedStr)
		if err != nil {
			return make([]da.Index, 0), make([]float64, 0), fmt.Errorf("customizer.readEdgeSpeedsFile: failed to parse segent speed: %v: %w", upEdgeSpeed, err)
		}

		if upEdgeSpeed < 0 {
			return make([]da.Index, 0), make([]float64, 0),
				fmt.Errorf("customizer.readEdgeSpeedsFile: segment speed must be non-negative: %v", upEdgeSpeed)
		}

		upSegmentIds = append(upSegmentIds, ebgnId)
		upSegmentSpeeds = append(upSegmentSpeeds, upEdgeSpeed)
	}

	return upSegmentIds, upSegmentSpeeds, nil
}

func (c *Customizer[W]) readTurnPenaltiesFromFile(filepath string) ([]da.Index, []float64, error) {
	f, err := os.Open(filepath)
	if err != nil {
		return make([]da.Index, 0), make([]float64, 0), fmt.Errorf("customizer.readTurnPenaltiesFromFile: failed to open file %v: %w", filepath, err)
	}

	defer f.Close()

	csvReader := csv.NewReader(f)
	data, err := csvReader.ReadAll()
	if err != nil {
		return make([]da.Index, 0), make([]float64, 0), fmt.Errorf("customizer.readTurnPenaltiesFromFile: failed to readAll csv data: %w", err)
	}

	n := len(data)
	upTurnIds := make([]da.Index, 0, n)
	upTurnPenalties := make([]float64, 0, n)
	for rowId := 0; rowId < n; rowId++ {
		row := data[rowId]
		uStr := strings.TrimSpace(row[0])
		u, err := util.ParseTextUInt64(uStr)
		if err != nil {
			return make([]da.Index, 0), make([]float64, 0), fmt.Errorf("customizer.readTurnPenaltiesFromFile: failed to parse uint64 u: %s: %w", uStr, err)
		}
		vStr := strings.TrimSpace(row[1])
		v, err := util.ParseTextUInt64(vStr)
		if err != nil {
			return make([]da.Index, 0), make([]float64, 0), fmt.Errorf("customizer.readTurnPenaltiesFromFile: failed to parse uint64 v: %s: %w", vStr, err)
		}

		wStr := strings.TrimSpace(row[2])
		w, err := util.ParseTextUInt64(wStr)
		if err != nil {
			return make([]da.Index, 0), make([]float64, 0), fmt.Errorf("customizer.readTurnPenaltiesFromFile: failed to parse uint64 w: %s: %w", wStr, err)
		}

		ebgeId := c.turnLookupTable.Get(da.NewTurnKV(u, v, w, 0))
		if ebgeId == da.INVALID_LK_TABLE_ID {
			c.logger.Sugar().Warnf("no vertex %v found", u)
			continue
		}

		turnPenaltyString := strings.TrimSpace(row[3])
		turnPenalty, err := util.ParseTextFloat64(turnPenaltyString)
		if err != nil {
			return make([]da.Index, 0), make([]float64, 0), fmt.Errorf("customizer.readTurnPenaltiesFromFile: failed to parse turn penalty: %s: %w", turnPenaltyString, err)
		}

		upTurnIds = append(upTurnIds, ebgeId)
		upTurnPenalties = append(upTurnPenalties, turnPenalty)
	}

	return upTurnIds, upTurnPenalties, nil
}

// UpdatedSegment is one row in a segment-speed CSV file; speed is kilometers per hour.
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
		return fmt.Errorf("WriteUpdatedSegmentsToCSV: failed to create file %v: %w", filepath, err)
	}
	defer f.Close()

	for _, seg := range segments {
		speedStr := strconv.FormatFloat(seg.speed, 'f', -1, 64)
		_, err := fmt.Fprintf(f, "%d, %d, %s\n", seg.fromOsmId, seg.toOsmId, speedStr)
		if err != nil {
			return fmt.Errorf("WriteUpdatedSegmentsToCSV: failed to write row for segment (%d,%d): %w", seg.fromOsmId, seg.toOsmId, err)
		}
	}

	return nil
}
