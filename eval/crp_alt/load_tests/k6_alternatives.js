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
success rate (dari semua query ada berapa query yang berhasil return alternative routes): 84%  (didapat dari script eval/crp_alt/alternative_routes/main.go)
300 vus:
HTTP
http_req_duration.......................................................: avg=12.14ms min=300.43µs med=10.55ms max=102.66ms p(90)=19.54ms p(95)=25.58ms
{ expected_response:true }............................................: avg=12.14ms min=300.43µs med=10.55ms max=102.66ms p(90)=19.54ms p(95)=25.58ms
http_req_failed.........................................................: 0.00%  0 out of 4594
http_reqs...............................................................: 4594   176.58433/s

EXECUTION
iteration_duration......................................................: avg=1.01s   min=1s       med=1.01s   max=1.1s     p(90)=1.02s   p(95)=1.02s  
iterations..............................................................: 4594   176.58433/s
vus.....................................................................: 8      min=8         max=300
vus_max.................................................................: 300    min=300       max=300


900 vus:
HTTP
http_req_duration.......................................................: avg=314.57ms min=535.76µs med=234.34ms max=2.74s p(90)=751.73ms p(95)=891.9ms
  { expected_response:true }............................................: avg=314.57ms min=535.76µs med=234.34ms max=2.74s p(90)=751.73ms p(95)=891.9ms
http_req_failed.........................................................: 0.00%  0 out of 10762
http_reqs...............................................................: 10762  408.860522/s

EXECUTION
iteration_duration......................................................: avg=1.31s    min=1s       med=1.23s    max=3.74s p(90)=1.75s    p(95)=1.89s  
iterations..............................................................: 10762  408.860522/s
vus.....................................................................: 347    min=44         max=900
vus_max.................................................................: 900    min=900        max=900


osrm-backend (https://github.com/Project-OSRM/osrm-backend) pakai script di "eval/osrm/load_tests/k6_alternatives.js":
success rate (dari semua query ada berapa query yang berhasil return alternative routes): 56%  (didapat dari script eval/osrm/alternative_routes/main.go)

vus:300
HTTP
http_req_duration.......................................................: avg=20.08ms min=662.48µs med=13.33ms max=254.5ms p(90)=36.29ms p(95)=62.14ms
  { expected_response:true }............................................: avg=20.11ms min=662.48µs med=13.38ms max=254.5ms p(90)=36.31ms p(95)=62.31ms
http_req_failed.........................................................: 0.28%  17 out of 6045
http_reqs...............................................................: 6045   194.794051/s

EXECUTION
iteration_duration......................................................: avg=1.02s   min=1s       med=1.01s   max=1.25s   p(90)=1.03s   p(95)=1.06s  
iterations..............................................................: 6045   194.794051/s
vus.....................................................................: 13     min=13         max=300
vus_max.................................................................: 300    min=300        max=300


900 vus:
HTTP
http_req_duration.......................................................: avg=670.47ms min=697.96µs med=592.32ms max=2.04s p(90)=1.42s p(95)=1.61s
  { expected_response:true }............................................: avg=670.79ms min=697.96µs med=593.12ms max=2.04s p(90)=1.42s p(95)=1.61s
http_req_failed.........................................................: 0.16%  19 out of 11480
http_reqs...............................................................: 11480  360.930442/s

EXECUTION
iteration_duration......................................................: avg=1.67s    min=1s       med=1.59s    max=3.04s p(90)=2.42s p(95)=2.61s
iterations..............................................................: 11480  360.930442/s
vus.....................................................................: 697    min=43          max=900
vus_max.................................................................: 900    min=900         max=900

todo: pindahin hasil eksperimen di repo baru + bandingin juga dg graphopper , valhalla
todo2: add evaluasi online map matching pakai dataset dari https://www.microsoft.com/en-us/research/publication/hidden-markov-map-matching-noise-sparseness/

todo (setelah skripsi selesai): implement rute alternative finder pakai cara https://dl.acm.org/doi/10.1145/3567421  (mathnya very hard... belum ada yang implement di open source routing engine)
todo (setelah skripsi selesai): implement another online map matching algorithm https://dl.acm.org/doi/pdf/10.1145/2666310.2666383

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
