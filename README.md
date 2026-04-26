# Navigatorx

todo: add readme

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




