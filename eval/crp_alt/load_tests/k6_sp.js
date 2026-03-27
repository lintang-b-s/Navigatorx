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
    { duration: "15s", target: 900 },
    { duration: "15s", target: 900 },
  ],
};

/*
navigatorx:
setelah ganti cache ke https://github.com/dgraph-io/ristretto
900vus:
HTTP
http_req_duration.......................................................: avg=5.6ms min=267.87µs med=4.46ms max=142.06ms p(90)=8.76ms p(95)=12.1ms
{ expected_response:true }............................................: avg=5.6ms min=405.5µs  med=4.46ms max=142.06ms p(90)=8.76ms p(95)=12.1ms
http_req_failed.........................................................: 0.00%  1 out of 20565
http_reqs...............................................................: 20565  663.260696/s

EXECUTION
iteration_duration......................................................: avg=1s    min=1s       med=1s     max=1.14s    p(90)=1s     p(95)=1.01s 
iterations..............................................................: 20565  663.260696/s
vus.....................................................................: 36     min=36         max=900
vus_max.................................................................: 900    min=900        max=900

NETWORK
data_received...........................................................: 556 MB 18 MB/s
data_sent...............................................................: 5.0 MB 160 kB/s

gokilll

osrm-backend build from source quick start:
900vus
HTTP
http_req_duration.......................................................: avg=8.43ms min=482.98µs med=5.56ms max=219.71ms p(90)=14.93ms p(95)=25.99ms
{ expected_response:true }............................................: avg=8.43ms min=482.98µs med=5.56ms max=219.71ms p(90)=14.93ms p(95)=25.99ms
http_req_failed.........................................................: 0.00%  0 out of 20531
http_reqs...............................................................: 20531  661.579731/s

EXECUTION
iteration_duration......................................................: avg=1s     min=1s       med=1s     max=1.22s    p(90)=1.01s   p(95)=1.02s  
iterations..............................................................: 20531  661.579731/s
vus.....................................................................: 49     min=49         max=900
vus_max.................................................................: 900    min=900        max=900

NETWORK
data_received...........................................................: 1.4 GB 45 MB/s
data_sent...............................................................: 4.7 MB 153 kB/s


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
