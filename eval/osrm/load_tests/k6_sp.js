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

export default () => {
  const randomQuery = queryData[Math.floor(Math.random() * queryData.length)];
  // http://localhost:5000/route/v1/driving/110.3521728515625,-7.754197163260652\;110.37775039672852,-7.770015394576607\?overview\=false\&alternatives\=true\&steps\=true;
  const res = http.get(
    `http://localhost:5000/route/v1/driving/${randomQuery.srcLon},${randomQuery.srcLat};${randomQuery.destLon},${randomQuery.destLat}?overview=false&alternatives=false&steps=true`,
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
