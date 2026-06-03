echo "Running test..."

gdown https://drive.google.com/uc?id=1uBoFWUSRka9pqH2dVPKpcystxXmkkSgs --output ./data
gdown https://drive.google.com/uc?id=1PjYZk9wl5xSykqjak-ZFvlT427X1eSa0 --output ./data

go test -p 1 ./... -v -timeout=0  -failfast  -count=1 -coverpkg=./pkg/... -coverprofile=cover.out  -skip "^TestOSN2024KRLMALT$|^TestOSN2024KRL$" # skip test yang lama selesainya
# note: tests harus tidak concurrent (-p 1) karena kita setiap TestFunction pada tests/shortestpath, tests/shortestpath_crp_alt, tests/query write ke file landmark yang sama
# selain itu kalau kita pakai concurrent, verbose log nya (-v) gak langsung keluar




