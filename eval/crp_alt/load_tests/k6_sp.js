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
    { duration: "10s", target: 900 },
  ],
};

/*
navigatorx:

900vus:
HTTP
http_req_duration.......................................................: avg=6.87ms min=324.78µs med=4.59ms max=221.94ms p(90)=9.92ms p(95)=17.17ms
  { expected_response:true }............................................: avg=6.87ms min=336.61µs med=4.59ms max=221.94ms p(90)=9.92ms p(95)=17.17ms
http_req_failed.........................................................: 0.00%  1 out of 18269
http_reqs...............................................................: 18269  589.217075/s

EXECUTION
iteration_duration......................................................: avg=1s     min=1s       med=1s     max=1.22s    p(90)=1.01s  p(95)=1.01s  
iterations..............................................................: 18269  589.217075/s
vus.....................................................................: 10     min=10         max=900
vus_max.................................................................: 900    min=900        max=900

NETWORK
data_received...........................................................: 495 MB 16 MB/s
data_sent...............................................................: 4.4 MB 142 kB/s

osrm-backend build from source quick start:
900vus
HTTP
http_req_duration.......................................................: avg=7.31ms min=432.5µs med=5.61ms max=214.28ms p(90)=12.51ms p(95)=19.71ms
{ expected_response:true }............................................: avg=7.31ms min=432.5µs med=5.61ms max=214.28ms p(90)=12.51ms p(95)=19.71ms
http_req_failed.........................................................: 0.00%  0 out of 18277
http_reqs...............................................................: 18277  588.968368/s

EXECUTION
iteration_duration......................................................: avg=1s     min=1s      med=1s     max=1.21s    p(90)=1.01s   p(95)=1.02s  
iterations..............................................................: 18277  588.968368/s
vus.....................................................................: 64     min=43         max=900
vus_max.................................................................: 900    min=900        max=900

NETWORK
data_received...........................................................: 1.2 GB 40 MB/s
data_sent...............................................................: 4.2 MB 136 kB/s




todo2: bikin repo baru, buat bandingin navigatorx,osrm,graphopper,valhalla

*/

// todo: kurangi call GetPriority di overlay graph search..

export default () => {
  const randomQuery = queryData[Math.floor(Math.random() * queryData.length)];

  const res = http.get(
    `http://localhost:6060/api/computeRoutes?origin_lat=${randomQuery.srcLat}&origin_lon=${randomQuery.srcLon}&destination_lat=${randomQuery.destLat}&destination_lon=${randomQuery.destLon}`,
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
