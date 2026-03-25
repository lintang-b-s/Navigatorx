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
http_req_duration.......................................................: avg=212.11ms min=503.19µs med=86.85ms max=2s p(90)=605.13ms p(95)=804.32ms
  { expected_response:true }............................................: avg=212.11ms min=503.19µs med=86.85ms max=2s p(90)=605.13ms p(95)=804.32ms
http_req_failed.........................................................: 0.00%  0 out of 11639
http_reqs...............................................................: 11639  442.898714/s

EXECUTION
iteration_duration......................................................: avg=1.21s    min=1s       med=1.08s   max=3s p(90)=1.6s     p(95)=1.8s    
iterations..............................................................: 11639  442.898714/s
vus.....................................................................: 289    min=43         max=900
vus_max.................................................................: 900    min=900        max=900




osrm-backend build from source quick start:
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


osrm-backend docker quick start(https://github.com/Project-OSRM/osrm-backend) pakai script di "eval/osrm/load_tests/k6_alternatives.js":
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
http_req_duration.......................................................: avg=795.78ms min=1.1ms med=755.2ms  max=3.19s p(90)=1.73s p(95)=1.88s
  { expected_response:true }............................................: avg=795.83ms min=1.1ms med=755.23ms max=3.19s p(90)=1.73s p(95)=1.88s
http_req_failed.........................................................: 0.17%  18 out of 10555
http_reqs...............................................................: 10555  335.041329/s

EXECUTION
iteration_duration......................................................: avg=1.79s    min=1s    med=1.75s    max=4.19s p(90)=2.73s p(95)=2.88s
iterations..............................................................: 10555  335.041329/s
vus.....................................................................: 546    min=41          max=900
vus_max.................................................................: 900    min=900         max=900

apa karena aku run osrm pake docker yak?

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
