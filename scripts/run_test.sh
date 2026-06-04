echo "Running test..."

gdown https://drive.google.com/uc?id=1HBswl5-JkFXWh--AFLC2ElYC4Tbsj1i0   --output ./data
gdown https://drive.google.com/uc?id=1pRmqUFgNc_p0lEKmfzLcn3IRhUW4Cm4c  --output ./data

REPO_ROOT=$(pwd)

for test_data_dir in tests/*/data; do
  ln -sf "$REPO_ROOT/data/yogyakarta.osm.pbf" "$test_data_dir/yogyakarta.osm.pbf"
  ln -sf "$REPO_ROOT/data/solo.osm.pbf" "$test_data_dir/solo.osm.pbf"
done

go test -p 1 ./... -v -timeout=0  -failfast  -count=1 -coverpkg=./pkg/... -coverprofile=cover.out  -skip "^TestOSN2024KRLMALT$|^TestOSN2024KRL$" # skip test yang lama selesainya
# note: tests harus tidak concurrent (-p 1) karena kita setiap TestFunction pada tests/shortestpath, tests/shortestpath_crp_alt, tests/query write ke file landmark yang sama
# selain itu kalau kita pakai concurrent, verbose log nya (-v) gak langsung keluar


