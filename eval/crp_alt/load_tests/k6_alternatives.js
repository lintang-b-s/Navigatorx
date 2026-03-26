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
http_req_duration.......................................................: avg=296.64ms min=883.75µs med=205.29ms max=1.72s p(90)=758.72ms p(95)=917.88ms
  { expected_response:true }............................................: avg=296.64ms min=883.75µs med=205.29ms max=1.72s p(90)=758.72ms p(95)=917.88ms
http_req_failed.........................................................: 0.00%  0 out of 10962
http_reqs...............................................................: 10962  418.556191/s

EXECUTION
iteration_duration......................................................: avg=1.29s    min=1s       med=1.2s     max=2.72s p(90)=1.75s    p(95)=1.91s   
iterations..............................................................: 10962  418.556191/s
vus.....................................................................: 416    min=44         max=900
vus_max.................................................................: 900    min=900        max=900

osrm-backend build from source quick start (laptop tidak sambil dicharge & tdk low battery):
success rate (dari semua query ada berapa query yang berhasil return alternative routes): 56%  (didapat dari script eval/osrm/alternative_routes/main.go)
900vus
HTTP
http_req_duration.......................................................: avg=144.36ms min=601.89µs med=102.27ms max=10.63s p(90)=196.93ms p(95)=230.31ms
  { expected_response:true }............................................: avg=144.36ms min=601.89µs med=102.27ms max=10.63s p(90)=196.93ms p(95)=230.31ms
http_req_failed.........................................................: 0.00%  0 out of 16173
http_reqs...............................................................: 16173  518.131749/s

EXECUTION
iteration_duration......................................................: avg=1.14s    min=1s       med=1.1s     max=11.64s p(90)=1.19s    p(95)=1.23s   
iterations..............................................................: 16173  518.131749/s
vus.....................................................................: 153    min=44         max=900
vus_max.................................................................: 900    min=900        max=900



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
