#include "QueryProcessor.hpp"
#include "ShortestPath.hpp"
#include "knn.hpp"
#include "json.hpp"

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

    if (query_type == "remove_edge")
    {
        return handle_remove_edge(query);
    }
    else if (query_type == "modify_edge")
    {
        return handle_modify_edge(query);
    }
    else if (query_type == "shortest_path")
    {
        return handle_shortest_path(query);
    }
    else if (query_type == "knn")
    {
        return handle_knn(query);
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

json QueryProcessor::handle_remove_edge(const json &query)
{
    int id = get_id_or_minus_one(query);
    try
    {
        int edge_id = query.at("edge_id").get<int>();
        bool success = graph.remove_edge(edge_id);
        json result;
        result["id"] = id;
        result["done"] = success;
        return result;
    }
    catch (const json::exception &e)
    {
        if (id != -1)
            return json{{"id", id}, {"status", "error"}, {"message", "remove_edge: missing/invalid fields"}};
        return json{{"status", "error"}, {"message", "remove_edge: missing/invalid fields"}};
    }
}

json QueryProcessor::handle_modify_edge(const json &query)
{
    int id = get_id_or_minus_one(query);
    try
    {
        int edge_id = query.at("edge_id").get<int>();
        // patch may be an object (possibly empty) per spec; use .at to require presence of the key
        json patch;
        if (query.contains("patch"))
        {
            patch = query.at("patch");
        }
        else
        {
            patch = json::object();
        }
        bool success = graph.modify_edge(edge_id, patch);
        json result;
        result["id"] = id;
        result["done"] = success;
        return result;
    }
    catch (const json::exception &e)
    {
        if (id != -1)
            return json{{"id", id}, {"status", "error"}, {"message", "modify_edge: missing/invalid fields"}};
        return json{{"status", "error"}, {"message", "modify_edge: missing/invalid fields"}};
    }
}

json QueryProcessor::handle_shortest_path(const json &query)
{
    int id = get_id_or_minus_one(query);
    try
    {
        int source = query.at("source").get<int>();
        int target = query.at("target").get<int>();
        std::string mode = query.at("mode").get<std::string>();

        std::vector<int> forbidden_nodes;
        std::vector<std::string> forbidden_road_types;
        if (query.contains("constraints") && query["constraints"].is_object())
        {
            auto q = query["constraints"];
            if (q.contains("forbidden_nodes") && q["forbidden_nodes"].is_array())
            {
                for (const auto &node : q["forbidden_nodes"])
                {
                    if (node.is_number_integer())
                        forbidden_nodes.push_back(node.get<int>());
                }
            }
            if (q.contains("forbidden_road_types") && q["forbidden_road_types"].is_array())
            {
                for (const auto &rt : q["forbidden_road_types"])
                {
                    if (rt.is_string())
                        forbidden_road_types.push_back(rt.get<std::string>());
                }
            }
        }
        json result;
        if (id != -1)
            result["id"] = id;
        std::vector<int> path;
        double cost = -1;
        if (mode == "time")
        {
            path = shortest_time(graph, source, target, forbidden_nodes, forbidden_road_types, &cost);
        }
        else if (mode == "distance")
        {
            path = shortest_distance(graph, source, target, forbidden_nodes, forbidden_road_types, &cost);
        }
        else
        {
            if (id != -1)
                return json{{"id", id}, {"status", "error"}, {"message", "shortest_path: unknown mode"}};
            return json{{"status", "error"}, {"message", "shortest_path: unknown mode"}};
        }

        result["possible"] = !path.empty();
        if (!path.empty())
            result["path"] = path;
        result["minimum_time/minimum_distance"] = cost;
        return result;
    }
    catch (const json::exception &e)
    {
        if (id != -1)
            return json{{"id", id}, {"status", "error"}, {"message", "shortest_path: missing/invalid fields"}};
        return json{{"status", "error"}, {"message", "shortest_path: missing/invalid fields"}};
    }
}

json QueryProcessor::handle_knn(const json &query)
{
    int id = get_id_or_minus_one(query);
    try
    {

        std::string poi_type;
        try
        {
            poi_type = query.at("poi").get<std::string>();
        }
        catch (...)
        {
            poi_type = query.at("poi_type").get<std::string>();
        }

        int k = query.at("k").get<int>();

        double query_lat, query_lon;
        try
        {
            query_lat = query.at("query_lat").get<double>();
            query_lon = query.at("query_lon").get<double>();
        }
        catch (...)
        {
            // try nested
            const json &qp = query.at("query_point");
            query_lat = qp.at("lat").get<double>();
            query_lon = qp.at("lon").get<double>();
        }

        std::string metric = query.at("metric").get<std::string>();

        std::vector<int> nodes_ids;
        if (metric == "shortest_path")
        {
            nodes_ids = knn_shortest_path(graph, query_lat, query_lon, poi_type, k);
        }
        else if (metric == "euclidean")
        {
            nodes_ids = knn_euclidean(graph, query_lat, query_lon, poi_type, k);
        }
        else
        {
            if (id != -1)
                return json{{"id", id}, {"status", "error"}, {"message", "knn: unknown metric"}};
            return json{{"status", "error"}, {"message", "knn: unknown metric"}};
        }

        json result;
        if (id != -1)
            result["id"] = id;
        result["nodes"] = nodes_ids;
        return result;
    }
    catch (const json::exception &e)
    {
        if (id != -1)
            return json{{"id", id}, {"status", "error"}, {"message", "knn: missing/invalid fields"}};
        return json{{"status", "error"}, {"message", "knn: missing/invalid fields"}};
    }
}
