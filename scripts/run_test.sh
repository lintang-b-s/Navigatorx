echo "Running test..."

go test -p 1 ./... -v -timeout=0  -failfast  -count=1 -coverpkg=./pkg/... -coverprofile=cover.out  -skip "^TestOSN2024KRLMALT$|^TestOSN2024KRL$" # skip test yang lama selesainya






