#include "QueryProcessor.hpp"
#include "tsp.hpp"
#include "json.hpp"

std::vector<std::vector<double>> APSP;
bool precomputed = false;

json QueryProcessor::process_query(json &query)
{
    try
    {
        return handle_tsp(query);
    }
    catch (const json::exception &e)
    {
        return json{{"status", "error"}, {"message", "Unknown query type"}};
    }
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

json QueryProcessor::handle_tsp(const json &query)
{
    std::string id_str = "unknown";
    try
    {
        if (query.contains("id"))
        {
            if (query["id"].is_number())
            {
                id_str = std::to_string(query["id"].get<int>());
            }
            else if (query["id"].is_string())
            {
                id_str = query["id"].get<std::string>();
            }
        }
    }
    catch (const json::exception &e)
    {
        return json{{"status", "error"}, {"message", "TSP: missing/invalid 'id' field"}};
    }
    try
    {
        const auto &fleet = query.at("fleet");
        int depot_node = fleet.at("depot_node");
        int num_drivers = fleet.at("num_delivery_guys");

        std::vector<Order> orders;
        if (query.contains("order_id"))
        {
            const auto &orders_array = query.at("order_id");
            for (const auto &o : orders_array)
            {
                orders.push_back({o.at("order_id").get<int>(),
                                  o.at("pickup").get<int>(),
                                  o.at("dropoff").get<int>()});
            }
        }
        else
        {
            return json{{"status", "error"}, {"message", "TSP: missing order list (checked 'orders' and 'order_id')"}};
        }
        double total_time = 26;
        std::pair<std::vector<std::tuple<int, std::vector<int>, std::vector<int>>>, double> tsp_result = tsp(graph, depot_node, num_drivers, orders, total_time);

        double total_cost = tsp_result.second;
        const auto &routes_vec = tsp_result.first;

        json result;
        json assignments = json::array();

        for (const auto &t : routes_vec)
        {
            int driver_id;
            std::vector<int> route_nodes;
            std::vector<int> order_ids;

            std::tie(driver_id, route_nodes, order_ids) = t;

            json assgn;
            assgn["driver_id"] = driver_id;
            assgn["route"] = route_nodes;
            assgn["order_ids"] = order_ids;
            assignments.push_back(assgn);
        }

        result["assignments"] = assignments;
        result["metrics"] = {{"total_delivery_time_s", total_cost}};

        if (id_str != "unknown")
            result["id"] = id_str;

        return result;
    }
    catch (const json::exception &e)
    {
        return json{{"status", "error"}, {"message", std::string("TSP error: ") + e.what()}};
    }
    catch (const std::exception &e)
    {
        return json{{"status", "error"}, {"message", std::string("TSP runtime error: ") + e.what()}};
    }
}
