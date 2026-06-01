#include "Graph.hpp"
#include <cmath>
#include <algorithm>
#include <queue>
#include <functional>
#include <iostream>
#include <limits>

void Graph::load_graph(const json &graph)
{
    int nodes_expected = 0;
    if (graph.contains("meta") && graph["meta"].contains("nodes"))
    {
        nodes_expected = graph["meta"]["nodes"].get<int>();
    }
    if (graph.contains("nodes"))
    {
        for (auto entry : graph["nodes"])
        {
            nodes_expected = std::max(nodes_expected, entry["id"].get<int>());
        }
    }
    nodes.clear();
    adj.clear();
    nodes.resize(nodes_expected + 1);
    adj.resize(nodes_expected + 1);

    try
    {
        // handling the nodes
        if (graph.contains("nodes"))
        {
            for (auto entry : graph["nodes"])
            {
                int id = entry["id"].get<int>();
                Node new_node;
                new_node.id = id;
                if (entry.contains("lat"))
                    new_node.lat = entry["lat"];
                if (entry.contains("lon"))
                    new_node.lon = entry["lon"];
                // just for saftey
                new_node.pois.clear();
                if (entry.contains("pois") && entry["pois"].is_array())
                {
                    for (auto p : entry["pois"])
                    {
                        new_node.pois.push_back(p.get<std::string>());
                    }
                }
                nodes[id] = new_node;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << 1 << " " << e.what() << '\n';
    }
    try
    {
        if (graph.contains("edges"))
        {
            int cnt = 0;
            for (auto entry : graph["edges"])
            {
                Edge new_edge;
                if (entry.contains("id"))
                    new_edge.id = entry["id"].get<int>();
                else
                {
                    new_edge.id = cnt;
                    cnt++;
                }
                new_edge.u = entry["u"].get<int>();
                new_edge.v = entry["v"].get<int>();
                if (entry.contains("average_time"))
                    new_edge.length = entry["average_time"].get<double>();
                if (entry.contains("oneway"))
                    new_edge.one_way = entry["oneway"].get<bool>();
                if (entry.contains("road_type"))
                    new_edge.road_type = entry["road_type"].get<std::string>();
                new_edge.speed_profile.clear();
                if (entry.contains("speed_profile") && entry["speed_profile"].is_array())
                {
                    for (auto speeds : entry["speed_profile"])
                    {
                        new_edge.speed_profile.push_back(speeds.get<double>());
                    }
                }
                edges_by_id[new_edge.id] = new_edge;

                AdjEntry u_to_v;
                u_to_v.to = new_edge.v;
                u_to_v.edge_id = new_edge.id;
                u_to_v.length = new_edge.length; 
                adj[new_edge.u].push_back(u_to_v);

                if (!new_edge.one_way)
                {
                    AdjEntry v_to_u;
                    v_to_u.to = new_edge.u;
                    v_to_u.edge_id = new_edge.id;
                    v_to_u.length = new_edge.length;
                    adj[new_edge.v].push_back(v_to_u);
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
}

bool Graph::remove_edge(int edge_id)
{
    auto it = edges_by_id.find(edge_id);
    if (it != edges_by_id.end())
    {
        Edge to_remove = it->second;
        int size_adj_u = adj[to_remove.u].size();
        for (int i = 0; i < size_adj_u; i++)
        {
            auto entry = adj[to_remove.u][i];
            if (entry.edge_id == edge_id && entry.to == to_remove.v)
            {
                adj[to_remove.u].erase(adj[to_remove.u].begin() + i);
                break;
            }
        }
        if (!to_remove.one_way)
        {
            int size_adj_v = adj[to_remove.v].size();
            for (int i = 0; i < size_adj_v; i++)
            {
                auto entry = adj[to_remove.v][i];
                if (entry.edge_id == edge_id && entry.to == to_remove.u)
                {
                    adj[to_remove.v].erase(adj[to_remove.v].begin() + i);
                    break; 
                }
            }
        }
        disabled_edges_by_id[edge_id] = to_remove;
        edges_by_id.erase(it);
        return true;
    }
    if (disabled_edges_by_id.find(edge_id) != disabled_edges_by_id.end())
    {
        return false;
    }
    return false;
}

bool Graph::modify_edge(int edge_id, const json &patch)
{
    if (edges_by_id.find(edge_id) != edges_by_id.end())
    {
        auto &edge_to_modify = edges_by_id[edge_id];
        if (patch.contains("length"))
        {
            edge_to_modify.length = patch["length"].get<double>();
        }
        if (patch.contains("average_time"))
        {
            edge_to_modify.average_time = patch["average_time"].get<double>();
        }
        if (patch.contains("road_type"))
        {
            edge_to_modify.road_type = patch["road_type"].get<std::string>();
        }
        if (patch.contains("speed_profile") && patch["speed_profile"].is_array())
        {
            edge_to_modify.speed_profile.clear();
            for (auto speeds : patch["speed_profile"])
            {
                edge_to_modify.speed_profile.push_back(speeds.get<double>());
            }
        }
        return true;
    }
    if (disabled_edges_by_id.find(edge_id) != disabled_edges_by_id.end())
    {
        auto edge_to_modify = disabled_edges_by_id[edge_id];
        if (patch.contains("length"))
        {
            edge_to_modify.length = patch["length"].get<double>();
        }
        if (patch.contains("average_time"))
        {
            edge_to_modify.average_time = patch["average_time"].get<double>();
        }
        if (patch.contains("road_type"))
        {
            edge_to_modify.road_type = patch["road_type"].get<std::string>();
        }
        if (patch.contains("speed_profile") && patch["speed_profile"].is_array())
        {
            edge_to_modify.speed_profile.clear();
            for (auto speeds : patch["speed_profile"])
            {
                edge_to_modify.speed_profile.push_back(speeds.get<double>());
            }
        }
        edges_by_id[edge_id] = edge_to_modify;
        AdjEntry u_to_v;
        u_to_v.to = edge_to_modify.v;
        u_to_v.edge_id = edge_to_modify.id;
        adj[edge_to_modify.u].push_back(u_to_v);

        if (!edge_to_modify.one_way)
        {
            AdjEntry v_to_u;
            v_to_u.to = edge_to_modify.u;
            v_to_u.edge_id = edge_to_modify.id;
            adj[edge_to_modify.v].push_back(v_to_u);
        }
        disabled_edges_by_id.erase(edge_id);
        return true;
    }
    return false;
}

double Graph::edge_time(int edge_id, double start_time_seconds) const
{
    auto it = edges_by_id.find(edge_id);
    if (it == edges_by_id.end())
    {
        return std::numeric_limits<double>::infinity(); 
    }

    const Edge &edge = it->second;
    if (edge.speed_profile.empty())
    {
        return (edge.average_time > 0.0) ? edge.average_time : std::numeric_limits<double>::infinity();
    }

    const int num_slots = static_cast<int>(edge.speed_profile.size());
    if (num_slots == 0)
    {
        return (edge.average_time > 0.0) ? edge.average_time : std::numeric_limits<double>::infinity();
    }

    const double slot_duration_seconds = 15.0 * 60.0;

    double time_taken = 0.0;
    double length_remaining = edge.length;
    double current_time_in_day = start_time_seconds;

    if (length_remaining == 0.0)
    {
        return 0.0;
    }

    int iterations = 0;
    const int max_iterations = num_slots * 2 + 100;

    while (length_remaining > 1e-9 && iterations < max_iterations)
    {
        int current_slot = static_cast<int>(std::fmod(std::floor(current_time_in_day / slot_duration_seconds), static_cast<double>(num_slots)));

        double speed = edge.speed_profile[current_slot];

        if (speed <= 0.0)
        {
            if (edge.average_time > 0.0)
            {
                return edge.average_time;
            }
            return std::numeric_limits<double>::infinity();
        }
        double time_to_next_slot = slot_duration_seconds - std::fmod(current_time_in_day, slot_duration_seconds);
        double time_to_traverse_remaining = length_remaining / speed;

        if (time_to_traverse_remaining <= time_to_next_slot)
        {
            time_taken += time_to_traverse_remaining;
            length_remaining = 0.0;
        }
        else
        {
            time_taken += time_to_next_slot;
            length_remaining -= speed * time_to_next_slot;
            current_time_in_day += time_to_next_slot;
        }
        iterations++;
    }

    if (length_remaining > 1e-9)
    {
        return std::numeric_limits<double>::infinity();
    }

    return time_taken;
}

int Graph::nearest_node(double query_lat, double query_lon) const
{
    int nearest_id = -1;
    double nearest_dist = std::numeric_limits<double>::max();
    for (const auto &node : nodes)
    {
        if (node.id == 0 && node.lat == 0.0 && node.lon == 0.0 && nearest_id != -1)
        {
            if (node.id != 0)
                continue;
        }

        double dlat = node.lat - query_lat;
        double dlon = node.lon - query_lon;
        double dist = dlat * dlat + dlon * dlon;
        if (dist < nearest_dist)
        {
            nearest_dist = dist;
            nearest_id = node.id;
        }
    }
    return nearest_id;
}

void Graph::compute_APSP()
{
    const Graph &graph = *this;
    int n = graph.num_nodes();
    double inf = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> dist(n, std::vector<double>(n, inf));
    std::vector<std::vector<int>> preds(n, std::vector<int>(n, -1));

    for (int s = 0; s < n; s++)
    {
        std::priority_queue<std::pair<double, int>,
                            std::vector<std::pair<double, int>>,
                            std::greater<std::pair<double, int>>>
            pq;

        dist[s][s] = 0.0;
        pq.push({0.0, s});

        while (!pq.empty())
        {
            auto [d, u] = pq.top();
            pq.pop();

            if (d > dist[s][u])
                continue;

            if ((size_t)u >= graph.adj.size())
                continue;

            for (const auto &adj_entry : graph.adj[u])
            {
                int v = adj_entry.to;
                double w = adj_entry.length;

                if (dist[s][u] + w < dist[s][v])
                {
                    dist[s][v] = dist[s][u] + w;
                    preds[s][v] = u; 
                    pq.push({dist[s][v], v});
                }
            }
        }
    }
    APSP = std::move(dist);
    predecessors = std::move(preds);
}

std::vector<int> Graph::get_path(int u, int v) const
{
    if (u == v)
        return {u};
    if (u < 0 || v < 0 || (size_t)u >= predecessors.size() || (size_t)v >= predecessors.size())
        return {};
    if (predecessors[u][v] == -1)
        return {};

    std::vector<int> path;
    int curr = v;
    while (curr != -1)
    {
        path.push_back(curr);
        if (curr == u)
            break;
        curr = predecessors[u][curr];
    }

    std::reverse(path.begin(), path.end());
    if (path.empty() || path[0] != u)
        return {};

    return path;
}

int Graph::num_nodes() const
{
    return nodes.size();
}
