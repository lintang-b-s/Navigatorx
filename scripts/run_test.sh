echo "Running test..."

go test -p 1 ./... -v -timeout=0  -failfast  -count=1 -coverpkg=./pkg/... -coverprofile=cover.out  -skip "^TestOSN2024KRLMALT$|^TestOSN2024KRL$" # skip test yang lama selesainya
# note: tests harus tidak concurrent (-p 1) karena kita setiap TestFunction pada tests/shortestpath, tests/shortestpath_crp_alt, tests/query write ke file landmark yang sama
# selain itu kalau kita pakai concurrent, verbose log nya (-v) gak langsung keluar





