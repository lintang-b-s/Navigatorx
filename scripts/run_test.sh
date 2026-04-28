echo "Running test..."

go test -p 1 ./... -v -timeout=0 --cover -coverpkg=./pkg/... -coverprofile=cover.out  -skip "^TestOSN2024KRL$" # skip test yang lama selesainya


