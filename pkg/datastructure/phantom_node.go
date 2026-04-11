package datastructure

// inspired by: https://github.com/Telenav/open-source-spec/blob/master/osrm/doc/od_in_osrm.md#phantomnode

/*
openstreetmap road network dibangun dari vertices and edges,
dengan edges adalah segmen jalan diantara dua intersection (atau diantara satu intersection dengan other endpoint, contoh: https://www.openstreetmap.org/way/259571140)
dan node adalah intersection.


namun ketika user supply query (source,destination) dalam bentuk latitude/longitude ke Navigatorx, seringkali query coordinate gak pas di node nya Openstreetmap.
sehingga biasanya routing engine seperti Navigatorx, OSRM (https://github.com/Project-OSRM/osrm-backend), Graphopper melakukan snapping query coordinate ke road segment terdekat.
hasil dari snapping adalah PhantomNode yaitu semacam virtual node hasil proyeksi dari query coordinate ke road segment terdekat.

PhantomNode dari Navigatorx (dan juga OSRM karena ini terinspirasi dari OSRM wkwkwk) berisi:
snappedCoordinate: coordinate dari PhantomNode
forwardTravelTime: travel time dari snappedCoordinate ke head dari edge
reverseTravelTime: travel time dari tail  dari edge ke snappedCoordinate
outEdgeId: id dari outEdge segmen jalan terdekat
inEdgeId: id dari inEdge segmen jalan terdekat
forwardCoords: edgeGeometry dari snappedCoordinate ke head dari edge
reverseCoords: edgeGeometry dari tail ke snappedCoordinate dari edge

note that outEdge dan inEdge merepresentasikan segment jalan yang sama hanya tail dan head nya dibalik..
misal outEdge (u,v) dengan tail u dan head v, inEdgenya adalah (v,u) dengan tail v dan head u.


contoh PhantomNode:

u1 -----------------x----------------- v1  (tail=u1, head=v1, phantomNode = x, outEdge=(u1,v1), inEdge=(v1,u1))

setiap edge di routing engine punya geometry,
karena graph edge dibuat berdasarkan Openstreetmap way object (contoh: https://www.openstreetmap.org/way/820890450)
dan setiap osm way punya beberapa OSM nodes, terutama untuk way yang curved  (contoh: https://www.openstreetmap.org/way/820890450)
buat nandain setiap intersection atau buat ngebentuk curved polylinenya...
setiap graph edge di routing engine punya geometry buat representasi bentuk aslinya.


*/

type PhantomNode struct {
	snappedCoordinate Coordinate
	forwardTravelTime float64
	reverseTravelTime float64
	forwardDistance   float64
	reverseDistance   float64
	outEdgeId         Index
	inEdgeid          Index
	forwardGeometry   []Coordinate
	reverseGeometry   []Coordinate
}

func NewPhantomNode(snappedCoordinate Coordinate,
	forwardTravelTime float64,
	reverseTravelTime float64,
	outEdgeId Index,
	inEdgeid Index, forwardDistance float64,
	reverseDistance float64,
	forwardGeometry []Coordinate,
	reverseGeometry []Coordinate) PhantomNode {
	return PhantomNode{snappedCoordinate: snappedCoordinate, forwardTravelTime: forwardTravelTime, reverseTravelTime: reverseTravelTime,
		outEdgeId: outEdgeId, inEdgeid: inEdgeid, forwardGeometry: forwardGeometry, reverseGeometry: reverseGeometry, forwardDistance: forwardDistance, reverseDistance: reverseDistance}
}

func NewInvalidPhantomNode() PhantomNode {
	return PhantomNode{outEdgeId: INVALID_EDGE_ID, inEdgeid: INVALID_EDGE_ID}
}

func IsPhantomNodeInvalid(pp PhantomNode) bool {
	return pp.outEdgeId == INVALID_EDGE_ID && pp.inEdgeid == INVALID_EDGE_ID
}

func (p *PhantomNode) GetSnappedCoord() Coordinate {
	return p.snappedCoordinate
}

func (p *PhantomNode) GetForwardTravelTime() float64 {
	return p.forwardTravelTime
}

func (p *PhantomNode) GetReverseTravelTime() float64 {
	return p.reverseTravelTime
}



func (p *PhantomNode) GetForwardDistance() float64 {
	return p.forwardDistance
}

func (p *PhantomNode) GetReverseDistance() float64 {
	return p.reverseDistance
}

func (p *PhantomNode) GetOutEdgeId() Index {
	return p.outEdgeId
}

func (p *PhantomNode) GetInEdgeId() Index {
	return p.inEdgeid
}

func (p *PhantomNode) GetForwardGeometry() []Coordinate {
	return p.forwardGeometry
}

func (p *PhantomNode) GetReverseGeometry() []Coordinate {
	return p.reverseGeometry
}
