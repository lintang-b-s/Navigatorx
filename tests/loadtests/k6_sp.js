import http from "k6/http";
import { sleep, check } from "k6";
export const options = {
  stages: [
    { duration: "1m", target: 50 }, // ramp up
    { duration: "30s", target: 50 }, // peak
  ],
};

// Source - https://stackoverflow.com/a/6878845
// Posted by Sergey Metlov, modified by community. See post 'Timeline' for change history
// Retrieved 2026-02-14, License - CC BY-SA 3.0
function getRandomInRange(from, to, fixed) {
  return (Math.random() * (to - from) + from).toFixed(fixed) * 1;
  // .toFixed() returns string, so ' * 1' is a trick to convert to number
}

export default () => {
  const srcLon = getRandomInRange(110.20878415486057, 110.84218487543602, 6);
  const srcLat = getRandomInRange(-7.8629891244016274, -7.152809872985647, 6);

  const destLon = getRandomInRange(110.20878415486057, 110.84218487543602, 6);
  const destLat = getRandomInRange(-7.8629891244016274, -7.152809872985647, 6);

  const res = http.get(
    `http://localhost:6060/api/computeRoutes?origin_lat=${srcLat}&origin_lon=${srcLon}&destination_lat=${destLat}&destination_lon=${destLon}`,
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
