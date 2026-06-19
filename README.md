# Navigatorx

Routing Engine For Openstreetmap data.

[![test](https://github.com/lintang-b-s/Navigatorx/actions/workflows/test.yml/badge.svg?branch=main)](https://github.com/lintang-b-s/Navigatorx/actions/workflows/test.yml)
[![lint](https://github.com/lintang-b-s/Navigatorx/actions/workflows/lint.yml/badge.svg?branch=main)](https://github.com/lintang-b-s/Navigatorx/actions/workflows/lint.yml)
[![coverage](https://raw.githubusercontent.com/lintang-b-s/Navigatorx/badges/.badges/main/coverage.svg)](/.github/.testcoverage.yml)
[![Go Report Card](https://goreportcard.com/badge/github.com/lintang-b-s/Navigatorx?cache=v1)](https://goreportcard.com/report/github.com/lintang-b-s/Navigatorx)

## Quick Start

### Download OpenStreetMap Data (.osm.pbf format)

You can download OpenStreetMap data from geofabrik (https://download.geofabrik.de/index.html)

```
pip install gdown
gdown https://drive.google.com/uc?id=1uBoFWUSRka9pqH2dVPKpcystxXmkkSgs --output ./data
```

### Collect cpu profiles (for profile guided optimization)

```
gdown https://drive.google.com/uc?id=1HBswl5-JkFXWh--AFLC2ElYC4Tbsj1i0   --output ./data
gdown https://drive.google.com/uc?id=1pRmqUFgNc_p0lEKmfzLcn3IRhUW4Cm4c  --output ./data
sh scripts/build_pgo.sh
```

### Pre-processing

run a preprocessing phase to speed up point-to-point fastest path queries. In the current implementation, only the Customizable Route Planning (CRP) ([[1]](#ref1)) algorithm is available. The CRP pre-processing phase creates multilevel partitions and overlay graph data structures.

```
export GOFLAGS="-buildvcs=false"
go build -o ./bin/preprocessor ./cmd/preprocessor
./bin/preprocessor
```

### Customization

The Customizable Route Planning CRP customization phase ([[1]](#ref1)) computes the shortcut weights at each cell of the multilevel partitioning result and computes landmark distances for the ALT algorithm (A\* search, landmarks, and triangle inequality) ([[3]](#ref3)).

```
go build -o ./bin/customizer ./cmd/customizer
./bin/customizer
```

### Query Engine

```

go build -o ./bin/engine -pgo=./bin/default.pgo  ./cmd/engine
./bin/engine
```

### Traffic Update

If you have real-time traffic data, you can also update the weights of the affected road segments (edges) while the query engine is running. <br>
example: <br>

```
./bin/customizer --segment-speed-file=./data/traffic_solo.csv,./data/blokade_solo.csv
```

For changes in the duration (weight) of road segments, the csv file follows the following format: <br>

```
from_osm_id, to_osm_id, edge_speed_in_km_h
```

from/to OSM node IDs must be connected. Note that for some OSM nodes that only have indegree and outdegree equal to 1, the node may be compressed/contracted so that only the two adjacent nodes to the contracted node remain in the compressed graph. <br>

After you run the command above, the query engine will provide the following log: <br>

```
2026-06-19T18:18:15.251949597+07:00     info    engine.checkCustomizerUpdate: file modification time changed  old=1781867857  new=1781867894
, updating the metrics and costFunction....
2026-06-19T18:18:15.890226994+07:00     info    updated the metrics and costFunction....
```

navigatorx also supports updating turn penalties with the customizer cmd flag "--turn-penalty-file", whose csv file follows the same format.

## Tests

```
gdown https://drive.google.com/uc?id=1HBswl5-JkFXWh--AFLC2ElYC4Tbsj1i0   --output ./data
gdown https://drive.google.com/uc?id=1pRmqUFgNc_p0lEKmfzLcn3IRhUW4Cm4c  --output ./data
sh ./scripts/run_test.sh
```

## Load Test

```
ulimit -n 65535 # increase file descriptor limits (https://goperf.dev/02-networking/networking-internals/)
go run eval/crp_alt/gen_rand_queries_coords/main.go
k6 run eval/crp_alt/load_tests/k6_sp.js
k6 run eval/crp_alt/load_tests/k6_alternatives.js
```

load tests result: https://github.com/lintang-b-s/skripsi_code

## API Documentation

The OpenAPI specification is available at [swagger.yaml](./swagger.yaml).

## References & Acknowledgments

### References

<a id="ref1"></a>1. Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579 .

<a id="ref2"></a>2. Abraham, I. et al. (2010) “Alternative Routes in Road Networks,” in P. Festa (ed.)
Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 23–34. Available at:
https://doi.org/10.1007/978-3-642-13193-6_3 .

<a id="ref3"></a>3. Goldberg, A. and Harrelson, C. (2005) “Computing the shortest path: A search meets
graph theory,” in. ACM-SIAM Symposium on Discrete Algorithms. Vancouver:
ACM, pp. 156 - 165.

### Acknowledgments

The author would like to express his deepest gratitude to the contributors to the open source projects below. The code in the Navigatorx project is heavily adapted and inspired by the following open source projects:

1. [CRP](https://github.com/michaelwegner/CRP)
2. [OSRM Backend](https://github.com/Project-OSRM/osrm-backend)
3. [GraphHopper](https://github.com/graphhopper/graphhopper)
4. [Telenav](https://github.com/Telenav/open-source-spec)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
