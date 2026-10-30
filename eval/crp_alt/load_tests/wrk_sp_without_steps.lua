-- wrk Lua script for load testing Navigatorx shortest path queries 
-- Replicates the behavior of Navigatorx/eval/crp_alt/load_tests/k6_sp.js
--  MAX_QUERIES=0  wrk -t4 -c200 -d4m --latency -s eval/crp_alt/load_tests/wrk_sp.lua http://localhost:6060

local threads = {}
local thread_counter = 0

function setup(thread)
    thread_counter = thread_counter + 1
    thread:set("id", thread_counter)
    table.insert(threads, thread)
end

function init(args)
    -- Per-thread Content-Length tracking globals
    cl_sum = 0
    cl_count = 0
    cl_min = -1
    cl_max = 0

    local coords_file = (args and args[1]) 
        or os.getenv("COORDS_FILE") 
        or "data/random_queries_1mil_sp_crp_alt_coords.txt"

    local file = io.open(coords_file, "r")
    if not file then
        coords_file = "../../../data/random_queries_1mil_sp_crp_alt_coords.txt"
        file = io.open(coords_file, "r")
    end

    if not file then
        print(string.format("[Thread %d] Error: Could not open coordinates file", id))
        os.exit(1)
    end

    local max_queries = tonumber(os.getenv("MAX_QUERIES")) or 100000

    queries = {}
    local count = 0
    for line in file:lines() do
        local srcLat, srcLon, destLat, destLon = line:match("(%S+)%s+(%S+)%s+(%S+)%s+(%S+)")
        if srcLat and srcLon and destLat and destLon then
            count = count + 1
            queries[count] = string.format(
                "/api/computeRoutes?origin_lat=%s&origin_lon=%s&destination_lat=%s&destination_lon=%s&useAnnotation=false&useSteps=false",
                srcLat, srcLon, destLat, destLon
            )
            if max_queries > 0 and count >= max_queries then
                break
            end
        end
    end
    file:close()

    math.randomseed(os.time() + (id or 1) * 99991)
    for _ = 1, 5 do math.random() end

    wrk.headers["Accept"] = "application/json"
end

function request()
    local idx = math.random(1, #queries)
    return wrk.format("GET", queries[idx])
end

-- Called for every HTTP response
function response(status, headers, body)
    -- Check both standard and lowercase header keys, fallback to #body
    local cl_header = headers["Content-Length"] or headers["content-length"]
    local len = tonumber(cl_header) or #body

    cl_sum = cl_sum + len
    cl_count = cl_count + 1
    if cl_min == -1 or len < cl_min then
        cl_min = len
    end
    if len > cl_max then
        cl_max = len
    end
end

function done(summary, latency, requests)
    -- Aggregate Content-Length stats across all worker threads
    local total_cl_bytes = 0
    local total_cl_count = 0
    local global_cl_min = -1
    local global_cl_max = 0

    for _, thread in ipairs(threads) do
        local t_sum = tonumber(thread:get("cl_sum")) or 0
        local t_count = tonumber(thread:get("cl_count")) or 0
        local t_min = tonumber(thread:get("cl_min")) or -1
        local t_max = tonumber(thread:get("cl_max")) or 0

        total_cl_bytes = total_cl_bytes + t_sum
        total_cl_count = total_cl_count + t_count
        if t_min ~= -1 and (global_cl_min == -1 or t_min < global_cl_min) then
            global_cl_min = t_min
        end
        if t_max > global_cl_max then
            global_cl_max = t_max
        end
    end

    local avg_cl = total_cl_count > 0 and (total_cl_bytes / total_cl_count) or 0
    if global_cl_min == -1 then global_cl_min = 0 end

    io.write("\n==========================================\n")
    io.write("       Navigatorx Load Test Summary       \n")
    io.write("==========================================\n")
    io.write(string.format("Duration:         %.2fs\n", summary.duration / 1000000))
    io.write(string.format("Total Requests:   %d\n", summary.requests))
    io.write(string.format("Total Bytes:      %.2f MB\n", summary.bytes / (1024 * 1024)))
    io.write(string.format("Avg Requests/sec: %.2f\n", (summary.requests / (summary.duration / 1000000))))
    io.write("------------------------------------------\n")
    io.write("Content-Length (Payload Size):\n")
    io.write(string.format("  Avg:            %8.2f bytes (%.2f KB)\n", avg_cl, avg_cl / 1024))
    io.write(string.format("  Min:            %8d bytes\n", global_cl_min))
    io.write(string.format("  Max:            %8d bytes (%.2f KB)\n", global_cl_max, global_cl_max / 1024))
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