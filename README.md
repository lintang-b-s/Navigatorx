# Navigatorx

todo: add readme

#### Quick Start
```
sh scripts/build_pgo.sh
pip install gdown
gdown https://drive.google.com/uc?id=1uBoFWUSRka9pqH2dVPKpcystxXmkkSgs --output ./data
go build -o ./bin/preprocessor ./cmd/preprocessor
./bin/preprocessor

go build -o ./bin/customizer ./cmd/customizer
./bin/customizer

go build -o ./bin/engine -pgo=auto  ./cmd/engine
./bin/engine
```

#### Tests
```
cd tests && go test ./... -v -timeout=0  --cover -coverpkg=../../pkg/... -coverprofile=coverage.out
```

#### Load Test
```
k6 run eval/crp_alt/load_tests/k6_sp.js
k6 run eval/crp_alt/load_tests/k6_alternatives.js
```




