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
setelah update partitioner:
300vus:
  HTTP
    http_req_duration.......................................................: avg=5.44ms min=589.67µs med=5.04ms max=32.22ms p(90)=8.41ms p(95)=9.57ms
      { expected_response:true }............................................: avg=5.44ms min=589.67µs med=5.04ms max=32.22ms p(90)=8.41ms p(95)=9.57ms
    http_req_failed.........................................................: 0.00%  0 out of 6115
    http_reqs...............................................................: 6115   197.243652/s

900vus:

 http_req_duration.......................................................: avg=30.48ms min=423.07µs med=9.4ms max=526.51ms p(90)=86.32ms p(95)=138.07ms
      { expected_response:true }............................................: avg=30.48ms min=423.07µs med=9.4ms max=526.51ms p(90)=86.32ms p(95)=138.07ms
    http_req_failed.........................................................: 0.00%  0 out of 17874
    http_reqs...............................................................: 17874  576.457418/s


osrm-backend (https://github.com/Project-OSRM/osrm-backend):
300vus:

http_req_duration.......................................................: avg=7.27ms min=533.26µs med=6.69ms max=43.05ms p(90)=11.71ms p(95)=13.94ms
  { expected_response:true }............................................: avg=7.27ms min=533.26µs med=6.7ms  max=43.05ms p(90)=11.71ms p(95)=13.95ms
http_req_failed.........................................................: 0.21%  13 out of 6105
http_reqs...............................................................: 6105   196.889472/s

900vus:
http_req_duration.......................................................: avg=298.32ms min=715.3µs med=205.88ms max=1.35s p(90)=690.78ms p(95)=824.75ms
  { expected_response:true }............................................: avg=298.31ms min=715.3µs med=205.64ms max=1.35s p(90)=690.78ms p(95)=824.69ms
http_req_failed.........................................................: 0.13%  20 out of 14297
http_reqs...............................................................: 14297  459.091542/s

gokilll

todo0: update inertial flow partitioner biar jumlah cut edges makin kecil (DONE)
todo1: coba coba parameter buat alternative route -> bikin success rate alternative routes lebih tinggi (minimal 60%? coba lihat alternative routesnya punya osrm mld dulu..)
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
