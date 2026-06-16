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
  discardResponseBodies: true,
  thresholds: {
    http_req_failed: ["rate<0.01"], // http errors should be less than 1%
    http_req_duration: ["p(95)<1000"], // 95% of requests should be below 1000ms
  },
  scenarios: {
    contacts: {
      executor: "ramping-arrival-rate",
      startRate: 0,
      timeUnit: "1s",
      preAllocatedVUs: 50,
      maxVUs: 150,
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

export default () => {
  const randomQuery = queryData[Math.floor(Math.random() * queryData.length)];

  const res = http.get(
    `http://localhost:5000/route/v1/driving/${randomQuery.srcLon},${randomQuery.srcLat};${randomQuery.destLon},${randomQuery.destLat}?overview=false&alternatives=true&steps=true`,
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
  });
};
