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
http_req_duration.......................................................: avg=7.69ms min=307.93µs med=5.56ms max=93.59ms p(90)=13.17ms p(95)=21.58ms
  { expected_response:true }............................................: avg=7.69ms min=327.66µs med=5.57ms max=93.59ms p(90)=13.17ms p(95)=21.58ms
http_req_failed.........................................................: 0.01%  2 out of 18291
http_reqs...............................................................: 18291  589.847111/s

EXECUTION
iteration_duration......................................................: avg=1s     min=1s       med=1s     max=1.09s   p(90)=1.01s   p(95)=1.02s  
iterations..............................................................: 18291  589.847111/s
vus.....................................................................: 42     min=42         max=900
vus_max.................................................................: 900    min=900        max=900

NETWORK
data_received...........................................................: 496 MB 16 MB/s
data_sent...............................................................: 4.4 MB 142 kB/s


osrm-backend build from source quick start:
900vus
HTTP
http_req_duration.......................................................: avg=6.05ms min=470.83µs med=5ms max=217.73ms p(90)=9.48ms p(95)=13.45ms
  { expected_response:true }............................................: avg=6.05ms min=470.83µs med=5ms max=217.73ms p(90)=9.48ms p(95)=13.45ms
http_req_failed.........................................................: 0.00%  0 out of 18328
http_reqs...............................................................: 18328  591.094216/s

EXECUTION
iteration_duration......................................................: avg=1s     min=1s       med=1s  max=1.21s    p(90)=1.01s  p(95)=1.01s  
iterations..............................................................: 18328  591.094216/s
vus.....................................................................: 21     min=21         max=900
vus_max.................................................................: 900    min=900        max=900

NETWORK
data_received...........................................................: 1.2 GB 40 MB/s
data_sent...............................................................: 4.2 MB 137 kB/s





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
