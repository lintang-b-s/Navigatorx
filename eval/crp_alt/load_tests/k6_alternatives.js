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
  stages: [
    { duration: "20s", target: 900 },
    { duration: "5s", target: 900 },
  ],
};

/*
navigatorx:
success rate (dari semua query ada berapa query yang berhasil return alternative routes): 81%  (didapat dari script eval/crp_alt/alternative_routes/main.go)

900 vus (laptop tidak sambil dicharge & tdk low battery):
HTTP
http_req_duration.......................................................: avg=120.33ms min=706.32µs med=19.67ms max=1.49s p(90)=386.27ms p(95)=533.96ms
  { expected_response:true }............................................: avg=120.34ms min=706.32µs med=19.69ms max=1.49s p(90)=386.29ms p(95)=533.97ms
http_req_failed.........................................................: 0.00%  1 out of 12610
http_reqs...............................................................: 12610  480.871108/s

EXECUTION
iteration_duration......................................................: avg=1.12s    min=1s       med=1.02s   max=2.49s p(90)=1.38s    p(95)=1.53s   
iterations..............................................................: 12610  480.871108/s
vus.....................................................................: 257    min=44         max=900
vus_max.................................................................: 900    min=900        max=900

NETWORK
data_received...........................................................: 550 MB 21 MB/s
data_sent...............................................................: 3.2 MB 123 kB/s



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

  check(res, { 200: (r) => r.status === 200 });
  sleep(1);
};
