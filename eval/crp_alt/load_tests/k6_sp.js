import http from "k6/http";
import { check } from "k6";
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
  discardResponseBodies: false,
  thresholds: {
    http_req_failed: ["rate<0.01"], // http errors should be less than 1%
    http_req_duration: ["p(95)<100"], // 95% of requests should be below 100ms
  },
  scenarios: {
    contacts: {
      executor: "ramping-arrival-rate",
      startRate: 0,
      timeUnit: "1s",
      preAllocatedVUs: 50,
      maxVUs: __ENV.MAXVUS || 150,

      stages: [
        { duration: "15s", target: parseInt(__ENV.RPS || "1000") },
        {
          duration: __ENV.DURATION || "15s",
          target: parseInt(__ENV.RPS || "1000"),
        },
      ],
    },
  },
};

/*
old result (maret 2026): 
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


todo2: bikin repo baru, buat bandingin navigatorx,osrm,graphopper,valhalla (DONE)

LATEST RESULTS pakai jateng_jabar.osm.pbf (2968484 nodes, 7117629 edges) (oktober 2026):
➜  navigatorx-crp git:(lintang-b-s/optimize-query) ✗ k6 run -e RPS=2000 -e MAXVUS=200   -e DURATION=4m --out web-dashboard  eval/crp_alt/load_tests/k6_sp.js

         /\      Grafana   /‾‾/
    /\  /  \     |\  __   /  /
   /  \/    \    | |/ /  /   ‾‾\
  /          \   |   (  |  (‾)  |
 / __________ \  |_|\_\  \_____/


     execution: local
        script: eval/crp_alt/load_tests/k6_sp.js
 web dashboard: http://127.0.0.1:5665
        output: -

     scenarios: (100.00%) 1 scenario, 200 max VUs, 4m45s max duration (incl. graceful stop):
              * contacts: Up to 2000.00 iterations/s for 4m15s over 2 stages (maxVUs: 50-200, gracefulStop: 30s)

WARN[0019] Insufficient VUs, reached 200 active VUs and cannot initialize more  executor=ramping-arrival-rate scenario=contacts
WARN[0019] The test has generated metrics with 100395 unique time series, which is higher than the suggested limit of 100000 and could cause high memory usage. Consider not using high-cardinality values like unique Ids as metric tags or, if you need them in the URL, use the name metric tag or URL grouping. See https://grafana.com/docs/k6/latest/using-k6/tags-and-groups/ for details.  component=metrics-engine-ingester
WARN[0026] The test has generated metrics with 200160 unique time series, which is higher than the suggested limit of 100000 and could cause high memory usage. Consider not using high-cardinality values like unique Ids as metric tags or, if you need them in the URL, use the name metric tag or URL grouping. See https://grafana.com/docs/k6/latest/using-k6/tags-and-groups/ for details.  component=metrics-engine-ingester
WARN[0040] The test has generated metrics with 400464 unique time series, which is higher than the suggested limit of 100000 and could cause high memory usage. Consider not using high-cardinality values like unique Ids as metric tags or, if you need them in the URL, use the name metric tag or URL grouping. See https://grafana.com/docs/k6/latest/using-k6/tags-and-groups/ for details.  component=metrics-engine-ingester
WARN[0067] The test has generated metrics with 800523 unique time series, which is higher than the suggested limit of 100000 and could cause high memory usage. Consider not using high-cardinality values like unique Ids as metric tags or, if you need them in the URL, use the name metric tag or URL grouping. See https://grafana.com/docs/k6/latest/using-k6/tags-and-groups/ for details.  component=metrics-engine-ingester
WARN[0127] The test has generated metrics with 1600146 unique time series, which is higher than the suggested limit of 100000 and could cause high memory usage. Consider not using high-cardinality values like unique Ids as metric tags or, if you need them in the URL, use the name metric tag or URL grouping. See https://grafana.com/docs/k6/latest/using-k6/tags-and-groups/ for details.  component=metrics-engine-ingester


  █ THRESHOLDS

    http_req_duration
    ✗ 'p(95)<100' p(95)=212.53ms

    http_req_failed
    ✓ 'rate<0.01' rate=0.00%


  █ TOTAL RESULTS

    checks_total.......: 839990 3293.064163/s
    checks_succeeded...: 99.99% 839989 out of 839990
    checks_failed......: 0.00%  1 out of 839990

    ✓ Get Content-Type header
    ✗ Ignore no path from source to destination response
      ↳  99% — ✓ 419994 / ✗ 1

    HTTP
    http_req_duration..............: avg=77.03ms min=418.89µs med=53.22ms max=1.57s p(90)=163.07ms p(95)=212.53ms
      { expected_response:true }...: avg=77.03ms min=418.89µs med=53.22ms max=1.57s p(90)=163.07ms p(95)=212.52ms
    http_req_failed................: 0.00%  1 out of 419995
    http_reqs......................: 419995 1646.532081/s

    EXECUTION
    dropped_iterations.............: 75003  294.038847/s
    iteration_duration.............: avg=77.27ms min=521.05µs med=53.44ms max=1.57s p(90)=163.4ms  p(95)=212.87ms
    iterations.....................: 419995 1646.532081/s
    vus............................: 170    min=2           max=200
    vus_max........................: 200    min=50          max=200

    NETWORK
    data_received..................: 11 GB  42 MB/s
    data_sent......................: 116 MB 453 kB/s




running (4m15.1s), 000/200 VUs, 419995 complete and 0 interrupted iterations
contacts ✓ [======================================] 000/200 VUs  4m15s  2000.00 iters/s
ERRO[0262] thresholds on metrics 'http_req_duration' have been crossed
➜  navigatorx-crp git:(lintang-b-s/optimize-query) ✗

*/

export default () => {
  const randomQuery = queryData[Math.floor(Math.random() * queryData.length)];

  const res = http.get(
    `http://localhost:6060/api/computeRoutes?origin_lat=${randomQuery.srcLat}&origin_lon=${randomQuery.srcLon}&destination_lat=${randomQuery.destLat}&destination_lon=${randomQuery.destLon}&useAnnotation=false&useSteps=true`,
    {
      headers: {
        "Content-Type": "application/json",
        Accept: "application/json",
      },
    },
  );

  check(res, {
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
          console.log("error: ", e);
          return false;
        }
      }

      if (res.status !== 200) {
        let message = res.body;
        try {
          const json = JSON.parse(res.body);
          message = json?.error?.message || json?.message || res.body;
        } catch (_) {}
        console.error(`[VU ${__VU}] Status ${res.status} error: ${message}`);
      }

      return false;
    },
  });
};
