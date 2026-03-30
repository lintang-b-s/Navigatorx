import http from "k6/http";
import { sleep, check } from "k6";
import { SharedArray } from "k6/data";

// https://grafana.com/docs/k6/latest/testing-guides/api-load-testing/
const queryData = new SharedArray("queries", function () {
  const file = open("../../../data/random_queries_1mil_sp_crp_alt_coords.txt");

  return file
    .split("\n")
    .filter((line) => line.trim() !== "")
    .map((line) => {
      const parts = line.trim().split(/\s+/);

      return {
        srcLat: parts[0],
        srcLon: parts[1],
        destLat: parts[2],
        destLon: parts[3],
      };
    });
});

export const options = {
  thresholds: {
    http_req_failed: ["rate<0.01"], // http errors should be less than 1%
    http_req_duration: ["p(95)<850"], // 95% of requests should be below 850ms
  },
  stages: [
    { duration: "15s", target: 900 },
    { duration: "15s", target: 900 },
  ],
};

/*
navigatorx:
success rate (dari semua query ada berapa query yang berhasil return alternative routes): 83%  (didapat dari script eval/crp_alt/alternative_routes/main.go)


900 vus (laptop tidak sambil dicharge & tdk low battery), setelah ganti cache ke https://github.com/dgraph-io/ristretto, pake ARRAY_STORAGE instead of TWO_LEVEL_STORAGE utk query heap,
dan add profile guided optimization .pgo file:

HTTP
http_req_duration.......................................................: avg=27.38ms min=504.05µs med=11.44ms max=304.66ms p(90)=75.12ms p(95)=100.51ms
{ expected_response:true }............................................: avg=27.38ms min=504.05µs med=11.44ms max=304.66ms p(90)=75.12ms p(95)=100.51ms
http_req_failed.........................................................: 0.00%  0 out of 20144
http_reqs...............................................................: 20144  649.279019/s

EXECUTION
iteration_duration......................................................: avg=1.02s   min=1s       med=1.01s   max=1.3s     p(90)=1.07s   p(95)=1.1s    
iterations..............................................................: 20144  649.279019/s
vus.....................................................................: 171    min=55         max=900
vus_max.................................................................: 900    min=900        max=900

NETWORK
data_received...........................................................: 922 MB 30 MB/s
data_sent...............................................................: 5.2 MB 166 kB/s


gokilll

osrm-backend build from source quick start (laptop tidak sambil dicharge & tdk low battery):
success rate (dari semua query ada berapa query yang berhasil return alternative routes): 56%  (didapat dari script eval/osrm/alternative_routes/main.go)
900vus
HTTP
http_req_duration.......................................................: avg=192.75ms min=623.88µs med=96.16ms max=7.99s p(90)=247.41ms p(95)=303.52ms
{ expected_response:true }............................................: avg=192.75ms min=623.88µs med=96.16ms max=7.99s p(90)=247.41ms p(95)=303.52ms
http_req_failed.........................................................: 0.00%  0 out of 11849
http_reqs...............................................................: 11849  448.740652/s

EXECUTION
iteration_duration......................................................: avg=1.19s    min=1s       med=1.09s   max=8.99s p(90)=1.24s    p(95)=1.3s    
iterations..............................................................: 11849  448.740652/s
vus.....................................................................: 331    min=42         max=900
vus_max.................................................................: 900    min=900        max=900

NETWORK
data_received...........................................................: 1.3 GB 51 MB/s
data_sent...............................................................: 2.7 MB 103 kB/s


todo: pindahin hasil eksperimen di repo baru + bandingin juga dg graphopper , valhalla


*/

export default () => {
  const randomQuery = queryData[Math.floor(Math.random() * queryData.length)];

  const res = http.get(
    `http://localhost:6060/api/computeAlternativeRoutes?origin_lat=${randomQuery.srcLat}&origin_lon=${randomQuery.srcLon}&destination_lat=${randomQuery.destLat}&destination_lon=${randomQuery.destLon}&k=3`,
    {
      headers: {
        "Content-Type": "application/json",
        Accept: "application/json",
      },
    },
  );

  check(res, {
    "Get status is 200": (r) => res.status === 200,
    "Get Content-Type header": (r) =>
      res.headers["Content-Type"] === "application/json",
  });
  sleep(1);
};
