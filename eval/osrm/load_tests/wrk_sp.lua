-- wrk Lua script for load testing OSRM shortest path queries
-- Replicates the behavior of k6_sp.js using random coordinates from coords file.
--  MAX_QUERIES=0 wrk -t4 -c200 -d4m --latency -s eval/osrm/load_tests/wrk_sp.lua http://localhost:5000


local threads = {}
local thread_counter = 0

-- Global setup: assign unique thread ID to each thread
function setup(thread)
    thread_counter = thread_counter + 1
    thread:set("id", thread_counter)
    table.insert(threads, thread)
end

-- Per-thread initialization
function init(args)
    -- Determine file path from argument, env variable, or default relative paths
    local coords_file = (args and args[1]) 
        or os.getenv("COORDS_FILE") 
        or "data/random_queries_1mil_sp_crp_alt_coords.txt"

    local file = io.open(coords_file, "r")
    if not file then
        -- Fallback to relative path if run from eval/osrm/load_tests directory
        coords_file = "../../../data/random_queries_1mil_sp_crp_alt_coords.txt"
        file = io.open(coords_file, "r")
    end

    if not file then
        print(string.format("[Thread %d] Error: Could not open coordinates file", id))
        os.exit(1)
    end

    -- Optional limit on number of queries loaded per thread to save startup time & memory.
    -- Set MAX_QUERIES=0 or MAX_QUERIES=1000000 to load all lines.
    local max_queries = tonumber(os.getenv("MAX_QUERIES")) or 100000

    queries = {}
    local count = 0
    for line in file:lines() do
        local srcLat, srcLon, destLat, destLon = line:match("(%S+)%s+(%S+)%s+(%S+)%s+(%S+)")
        if srcLat and srcLon and destLat and destLon then
            count = count + 1
            -- OSRM expects coordinates in {lon},{lat};{lon},{lat} order
            queries[count] = string.format(
                "/route/v1/driving/%s,%s;%s,%s?overview=false&alternatives=false&steps=true",
                srcLon, srcLat, destLon, destLat
            )
            if max_queries > 0 and count >= max_queries then
                break
            end
        end
    end
    file:close()

    -- Unique PRNG seed per thread
    math.randomseed(os.time() + (id or 1) * 99991)
    for _ = 1, 5 do math.random() end -- discard initial values

    -- Set default headers
    wrk.headers["Accept"] = "application/json"

    -- Per-thread metrics
    status_2xx = 0
    status_4xx = 0
    status_other = 0
end

-- Called for every request
function request()
    local idx = math.random(1, #queries)
    return wrk.format("GET", queries[idx])
end

-- Called for every response received
function response(status, headers, body)
    if status >= 200 and status < 300 then
        status_2xx = status_2xx + 1
    elseif status >= 400 and status < 500 then
        status_4xx = status_4xx + 1
    else
        status_other = status_other + 1
    end
end

-- Called once at the end of the test
function done(summary, latency, requests)
    io.write("\n==========================================\n")
    io.write("         OSRM Load Test Summary           \n")
    io.write("==========================================\n")
    io.write(string.format("Duration:         %.2fs\n", summary.duration / 1000000))
    io.write(string.format("Total Requests:   %d\n", summary.requests))
    io.write(string.format("Total Bytes:      %.2f MB\n", summary.bytes / (1024 * 1024)))
    io.write(string.format("Avg Requests/sec: %.2f\n", (summary.requests / (summary.duration / 1000000))))
    io.write("------------------------------------------\n")
    io.write("Latency Percentiles:\n")
    io.write(string.format("  Avg:            %8.2f ms\n", latency.mean / 1000))
    io.write(string.format("  50th (p50):     %8.2f ms\n", latency:percentile(50) / 1000))
    io.write(string.format("  75th (p75):     %8.2f ms\n", latency:percentile(75) / 1000))
    io.write(string.format("  90th (p90):     %8.2f ms\n", latency:percentile(90) / 1000))
    io.write(string.format("  95th (p95):     %8.2f ms\n", latency:percentile(95) / 1000))
    io.write(string.format("  99th (p99):     %8.2f ms\n", latency:percentile(99) / 1000))
    io.write(string.format("  Max:            %8.2f ms\n", latency.max / 1000))
    io.write("------------------------------------------\n")
    io.write("Socket Errors:\n")
    io.write(string.format("  Connect:        %d\n", summary.errors.connect))
    io.write(string.format("  Read:           %d\n", summary.errors.read))
    io.write(string.format("  Write:          %d\n", summary.errors.write))
    io.write(string.format("  Timeout:        %d\n", summary.errors.timeout))
    io.write("==========================================\n")
end
