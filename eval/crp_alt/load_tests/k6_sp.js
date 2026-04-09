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
    http_req_duration: ["p(95)<100"], // 95% of requests should be below 100ms
  },
  stages: [
    { duration: "15s", target: 1000 },
    { duration: "15s", target: 1000 },
  ],
};

/*
navigatorx:
900 vus (laptop tidak sambil dicharge & tdk low battery), setelah ganti cache ke https://github.com/dgraph-io/ristretto, pake ARRAY_STORAGE instead of TWO_LEVEL_STORAGE utk query heap,
dan add profile guided optimization .pgo file:

900vus:
HTTP
http_req_duration.......................................................: avg=4.2ms min=565.87µs med=3.64ms max=54.63ms p(90)=6.11ms p(95)=8.32ms
  { expected_response:true }............................................: avg=4.2ms min=565.87µs med=3.64ms max=54.63ms p(90)=6.11ms p(95)=8.32ms
http_req_failed.........................................................: 0.00%  0 out of 20612
http_reqs...............................................................: 20612  664.566751/s

EXECUTION
iteration_duration......................................................: avg=1s    min=1s       med=1s     max=1.05s   p(90)=1s     p(95)=1s    
iterations..............................................................: 20612  664.566751/s
vus.....................................................................: 43     min=43         max=900
vus_max.................................................................: 900    min=900        max=900

NETWORK
data_received...........................................................: 567 MB 18 MB/s
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

  check(res, {
    "Get status is 200": (r) => res.status === 200,
    "Get Content-Type header": (r) =>
      res.headers["Content-Type"] === "application/json",
    "Ignore no path from source to destination response": (r) => {
      if (r.status === 200) return true;

      if (r.status === 400) {
        try {
          const body = JSON.parse(r.body);
          const msg = body?.error?.message || "";
          return (
            msg.includes("no nearby road segments") ||
            msg.includes("no route found")
          );
        } catch (e) {
          return false;
        }
      }

      return false;
    },
  });
  sleep(1);
};
