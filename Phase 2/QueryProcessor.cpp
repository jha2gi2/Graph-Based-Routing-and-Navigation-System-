#include "QueryProcessor.hpp"
#include "k_shortest_paths.hpp"
#include "k_shortest_paths_heuristic.hpp"
#include "approx_shortest_paths.hpp"
#include "json.hpp"

std::vector<std::vector<double>> APSP;
bool precomputed = false;

json QueryProcessor::process_query(json &query)
{
    std::string query_type;
    try
    {
        query_type = query.at("type").get<std::string>();
    }
    catch (const json::exception &e)
    {
        return json{{"status", "error"}, {"message", "Query 'type' missing or invalid"}};
    }

    if (query_type == "k_shortest_paths")
    {
        return handle_k_shortest_paths(query);
    }
    else if (query_type == "k_shortest_paths_heuristic")
    {
        return handle_k_shortest_paths_heuristic(query);
    }
    else if (query_type == "approx_shortest_path")
    {
        return handle_approx_shortest_path(query);
    }
    return json{{"status", "error"}, {"message", "Unknown query type"}};
}

static int get_id_or_minus_one(const json &q)
{
    try
    {
        return q.at("id").get<int>();
    }
    catch (...)
    {
        return -1;
    }
}

json QueryProcessor::handle_k_shortest_paths(const json &query)
{
    int id = get_id_or_minus_one(query);
    try
    {
        int source = query.at("source").get<int>();
        int target = query.at("target").get<int>();
        int k = query.at("k").get<int>();
        std::string mode = "distance";
        if (query.contains("mode"))
            mode = query.at("mode").get<std::string>();
        json result;
        if (id != -1)
            result["id"] = id;
        std::vector<std::pair<std::vector<int>, double>> paths;
        if (mode == "distance")
        {
            paths = k_shortest_paths(graph, source, target, k, graph.APSP);
        }
        else
        {
            if (id != -1)
                return json{{"id", id}, {"status", "error"}, {"message", "k_shortest_paths: unknown mode"}};
            return json{{"status", "error"}, {"message", "k_shortest_paths: unknown mode"}};
        }

        // if (!paths.empty())
        // {
        json paths_json = json::array();
        for (const auto &p : paths)
        {
            json path_json;
            path_json["path"] = p.first;
            path_json["length"] = p.second;
            paths_json.push_back(path_json);
        }
        result["paths"] = paths_json;
        // }
        return result;
    }
    catch (const json::exception &e)
    {
        if (id != -1)
            return json{{"id", id}, {"status", "error"}, {"message", "K_shortest_paths: missing/invalid fields"}};
        return json{{"status", "error"}, {"message", "k_shortest_paths: missing/invalid fields"}};
    }
}

json QueryProcessor::handle_k_shortest_paths_heuristic(const json &query)
{
    int id = get_id_or_minus_one(query);
    try
    {
        int source = query.at("source").get<int>();
        int target = query.at("target").get<int>();
        int k = query.at("k").get<int>();
        double raw_threshold = query.at("overlap_threshold").get<double>();
        int overlap_threshold_pct;
        if (raw_threshold <= 1.0 && raw_threshold > 0) {
             overlap_threshold_pct = static_cast<int>(raw_threshold * 100.0);
        } else {
             overlap_threshold_pct = static_cast<int>(raw_threshold);
        }

        json result;
        if (id != -1)
            result["id"] = id;

        std::vector<std::pair<std::vector<int>, double>> paths;
        paths = k_shortest_paths_heuristic(graph, source, target, k, overlap_threshold_pct);

        json paths_json = json::array();
        for (const auto &p : paths)
        {
            json path_json;
            path_json["path"] = p.first;
            path_json["length"] = p.second;
            paths_json.push_back(path_json);
        }
        result["paths"] = paths_json;

        return result;
    }
    catch (const std::exception &e) 
    {
        if (id != -1)
            return json{{"id", id}, {"status", "error"}, {"message", e.what()}};
        return json{{"status", "error"}, {"message", "k_shortest_paths_heuristic: error processing"}};
    }
}

json QueryProcessor::handle_approx_shortest_path(const json &query)
{
    int id = get_id_or_minus_one(query);
    try
    {
        int time_budget_ms = query.at("time_budget_ms").get<int>();
        double acceptable_error_pct = query.at("acceptable_error_pct").get<double>();
        const json &queries_array = query.at("queries");
        auto start_time = std::chrono::high_resolution_clock::now();
        long long total_budget_ns = (long long)time_budget_ms * 950000;
        auto global_deadline = start_time + std::chrono::nanoseconds(total_budget_ns);

        json result;
        if (id != -1)
            result["id"] = id;

        json distances_json = json::array();

        for (const auto &sub_query : queries_array)
        {
            int source = sub_query.at("source").get<int>();
            int target = sub_query.at("target").get<int>();
            double distance = approximate_shortest_path(
                graph,
                source,
                target,
                global_deadline,
                acceptable_error_pct);
            json distance_entry;
            distance_entry["source"] = source;
            distance_entry["target"] = target;
            distance_entry["approx_shortest_distance"] = distance;
            distances_json.push_back(distance_entry);
        }

        result["distances"] = distances_json;
        return result;
    }
    catch (const json::exception &e)
    {
        if (id != -1)
            return json{{"id", id}, {"status", "error"}, {"message", "approx_shortest_path: missing/invalid fields"}};
        return json{{"status", "error"}, {"message", "approx_shortest_path: missing/invalid fields"}};
    }
}
