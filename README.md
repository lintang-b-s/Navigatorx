# Navigatorx

[![test](https://github.com/lintang-b-s/Navigatorx/actions/workflows/test.yml/badge.svg?branch=main)](https://github.com/lintang-b-s/Navigatorx/actions/workflows/test.yml)
[![lint](https://github.com/lintang-b-s/Navigatorx/actions/workflows/lint.yml/badge.svg?branch=main)](https://github.com/lintang-b-s/Navigatorx/actions/workflows/lint.yml)
[![coverage](https://raw.githubusercontent.com/lintang-b-s/Navigatorx/badges/.badges/main/coverage.svg)](/.github/.testcoverage.yml)
[![Go Report Card](https://goreportcard.com/badge/github.com/lintang-b-s/Navigatorx?cache=v1)](https://goreportcard.com/report/github.com/lintang-b-s/Navigatorx)


#### Quick Start

```
sh scripts/build_pgo.sh
pip install gdown
gdown https://drive.google.com/uc?id=1uBoFWUSRka9pqH2dVPKpcystxXmkkSgs --output ./data
export GOFLAGS="-buildvcs=false"
go build -o ./bin/preprocessor ./cmd/preprocessor
./bin/preprocessor

go build -o ./bin/customizer ./cmd/customizer
./bin/customizer

go build -o ./bin/generator ./cmd/generator
./bin/generator

go build -o ./bin/engine -pgo=auto  ./cmd/engine
./bin/engine
```

#### Tests

```
sh ./scripts/run_test.sh
```

#### Load Test
```
go run eval/crp_alt/gen_rand_queries_coords/main.go
k6 run eval/crp_alt/load_tests/k6_sp.js
k6 run eval/crp_alt/load_tests/k6_alternatives.js
```






