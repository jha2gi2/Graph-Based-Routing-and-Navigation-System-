#include "Graph.hpp"
#include <cmath>
#include <algorithm>
#include <queue>
#include <functional>
#include <iostream>
#include <chrono>
static const int NUM_LANDMARKS = 20;
void Graph::load_graph(const json &graph)
{
    int nodes_expected = 0;
    if (graph.contains("meta") && graph["meta"].contains("nodes"))
        nodes_expected = graph["meta"]["nodes"].get<int>();

    if (graph.contains("nodes"))
        for (auto entry : graph["nodes"])
            nodes_expected = std::max(nodes_expected, entry["id"].get<int>());

    int size = nodes_expected + 1;
    nodes.resize(size);
    adj.resize(size);
    adj_rev.resize(size);

    // Resize CH structures
    ch_adj.resize(size);
    ch_adj_rev.resize(size);

    rank.clear();
    w_dist.resize(size, std::numeric_limits<double>::infinity());
    w_visited_token.resize(size, 0);

    if (graph.contains("nodes"))
    {
        for (auto entry : graph["nodes"])
        {
            int id = entry["id"].get<int>();
            if (id >= nodes.size())
            {
                nodes.resize(id + 1);
                adj.resize(id + 1);
                adj_rev.resize(id + 1);
                ch_adj.resize(id + 1);
                ch_adj_rev.resize(id + 1);
            }
            Node new_node;
            new_node.id = id;
            if (entry.contains("lat"))
                new_node.lat = entry["lat"];
            if (entry.contains("lon"))
                new_node.lon = entry["lon"];
            nodes[id] = new_node;
        }
    }

    if (graph.contains("edges"))
    {
        for (auto entry : graph["edges"])
        {
            Edge new_edge;
            new_edge.id = entry["id"].get<int>();
            new_edge.u = entry["u"].get<int>();
            new_edge.v = entry["v"].get<int>();

            if (new_edge.u >= nodes.size() || new_edge.v >= nodes.size())
                continue;

            if (entry.contains("length"))
                new_edge.length = entry["length"].get<double>();
            if (entry.contains("average_time"))
                new_edge.average_time = entry["average_time"].get<double>();
            if (entry.contains("oneway"))
                new_edge.one_way = entry["oneway"].get<bool>();

            edges_by_id[new_edge.id] = new_edge;

            // Load ONLY into standard adjacency lists
            adj[new_edge.u].push_back({new_edge.v, new_edge.id, new_edge.length});
            adj_rev[new_edge.v].push_back({new_edge.u, new_edge.id, new_edge.length});

            if (!new_edge.one_way)
            {
                adj[new_edge.v].push_back({new_edge.u, new_edge.id, new_edge.length});
                adj_rev[new_edge.u].push_back({new_edge.v, new_edge.id, new_edge.length});
            }
        }
    }
}
bool Graph::remove_edge(int edge_id)
{
    auto it = edges_by_id.find(edge_id);
    if (it != edges_by_id.end())
    {
        Edge to_remove = it->second;
        // removing from adj
        int size_adj_u = adj[to_remove.u].size();
        for (int i = 0; i < size_adj_u; i++)
        {
            auto entry = adj[to_remove.u][i];
            if (entry.edge_id == edge_id && entry.to == to_remove.v)
            {
                adj[to_remove.u].erase(adj[to_remove.u].begin() + i);
                break; // check if this is correct can there be multiple same entries
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
                    break; // check if this is correct can there be multiple same entries
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
    // if (patch.empty())
    // {
    //     auto edge_to_modify = disabled_edges_by_id[edge_id];
    //     edges_by_id[edge_id] = edge_to_modify;
    //     AdjEntry u_to_v;
    //     u_to_v.to = edge_to_modify.v;
    //     u_to_v.edge_id = edge_to_modify.id;
    //     adj[edge_to_modify.u].push_back(u_to_v);

    //     if (!edge_to_modify.one_way)
    //     {
    //         AdjEntry v_to_u;
    //         v_to_u.to = edge_to_modify.u;
    //         v_to_u.edge_id = edge_to_modify.id;
    //         adj[edge_to_modify.v].push_back(v_to_u);
    //     }
    //     disabled_edges_by_id.erase(edge_id);
    //     return true;
    // }
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
        return std::numeric_limits<double>::infinity(); // Edge not found
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
        double dist = dlat * dlat + dlon * dlon; // squared distance
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
    std::vector<std::vector<double>> dist(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));

    for (int s = 0; s < n; s++)
    {
        // Dijkstra from source s
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
            if (d != dist[s][u])
                continue;

            for (const auto &adj : graph.adj[u])
            {
                int v = adj.to;
                double w = graph.edges_by_id.at(adj.edge_id).length;
                if (dist[s][u] + w < dist[s][v])
                {
                    dist[s][v] = dist[s][u] + w;
                    pq.push({dist[s][v], v});
                }
            }
        }
    }
    APSP_precomputed = true;
    APSP = std::move(dist);
}

static void run_dijkstra_for_landmark(const Graph &graph, int start_node, std::vector<double> &dist_out)
{
    int n = graph.nodes.size();
    dist_out.assign(n, std::numeric_limits<double>::infinity());

    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    dist_out[start_node] = 0.0;
    pq.push({0.0, start_node});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist_out[u])
            continue;

        for (const auto &entry : graph.adj[u])
        {
            if (dist_out[u] + entry.length < dist_out[entry.to])
            {
                dist_out[entry.to] = dist_out[u] + entry.length;
                pq.push({dist_out[entry.to], entry.to});
            }
        }
    }
}

void Graph::compute_landmarks()
{
    const Graph &graph = *this;
    int N = graph.nodes.size();
    if (N == 0)
        return;

    landmarks_dist.resize(NUM_LANDMARKS);
    for (int i = 0; i < NUM_LANDMARKS; ++i)
    {
        int landmark_id = (i * N) / NUM_LANDMARKS;
        if (landmark_id >= N)
            landmark_id = N - 1;

        run_dijkstra_for_landmark(graph, landmark_id, landmarks_dist[i]);
    }
}
bool Graph::witness_search(int start_node, int target_node, int ignore_node, double limit_dist)
{
    ++w_search_id;

    // [ACCURACY FIX] Reduce MAX_HOPS.
    // Forces shortcuts if the alternative is even slightly complex.
    const int MAX_HOPS = 2;

    using P = std::tuple<double, int, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    w_dist[start_node] = 0.0;
    w_visited_token[start_node] = w_search_id;
    pq.push({0.0, 0, start_node});

    int settled = 0;
    const int MAX_SETTLED = 10000;

    while (!pq.empty())
    {
        auto [d, hops, u] = pq.top();
        pq.pop();

        if (d > limit_dist)
            return false;
        if (u == target_node)
            return true; // Found a witness
        if (hops >= MAX_HOPS)
            continue; // Prune complex paths

        if (w_visited_token[u] == w_search_id && d > w_dist[u])
            continue;

        // Safety Break: If search is taking too long, assume NO witness -> ADD SHORTCUT.
        if (++settled > MAX_SETTLED)
            return false;

        // Helper to process neighbors
        auto process_neighbors = [&](const std::vector<AdjEntry> &neighbors)
        {
            for (const auto &e : neighbors)
            {
                int v = e.to;
                if (v == ignore_node)
                    continue;
                if (rank[v] != -1)
                    continue; // Uncontracted only

                double nd = d + e.length;
                if (nd > limit_dist)
                    continue;

                if (w_visited_token[v] != w_search_id || nd < w_dist[v])
                {
                    w_dist[v] = nd;
                    w_visited_token[v] = w_search_id;
                    pq.push({nd, hops + 1, v});
                }
            }
        };

        // Check BOTH original edges AND shortcuts
        process_neighbors(adj[u]);
        process_neighbors(ch_adj[u]);
    }
    return false;
}

void Graph::contract_node(int x, int &shortcut_id_counter)
{
    struct Neighbor
    {
        int u;
        double len;
        int edge_id;
    };
    std::vector<Neighbor> incoming;
    std::vector<Neighbor> outgoing;

    // Helper to gather active neighbors from both lists
    auto gather_in = [&](const std::vector<AdjEntry> &list)
    {
        for (const auto &ae : list)
            if (rank[ae.to] == -1)
                incoming.push_back({ae.to, ae.length, ae.edge_id});
    };
    auto gather_out = [&](const std::vector<AdjEntry> &list)
    {
        for (const auto &ae : list)
            if (rank[ae.to] == -1)
                outgoing.push_back({ae.to, ae.length, ae.edge_id});
    };

    gather_in(adj_rev[x]);
    gather_in(ch_adj_rev[x]);
    gather_out(adj[x]);
    gather_out(ch_adj[x]);

    for (const auto &in_p : incoming)
    {
        for (const auto &out_p : outgoing)
        {
            if (in_p.u == out_p.u)
                continue;

            double total = in_p.len + out_p.len;

            // Search for a witness (alternative path)
            // If witness search returns TRUE, we SKIP the shortcut.
            // If FALSE, we ADD the shortcut.
            if (!witness_search(in_p.u, out_p.u, x, total))
            {
                int sc_id = shortcut_id_counter++;
                Edge sc;
                sc.id = sc_id;
                sc.u = in_p.u;
                sc.v = out_p.u;
                sc.length = total;
                sc.one_way = true;
                sc.shortcut_e1 = in_p.edge_id;
                sc.shortcut_e2 = out_p.edge_id;

                edges_by_id[sc_id] = sc;

                ch_adj[in_p.u].push_back({out_p.u, sc_id, total});
                ch_adj_rev[out_p.u].push_back({in_p.u, sc_id, total});
            }
        }
    }
}

void Graph::preprocess_ch()
{
    auto start_time = std::chrono::high_resolution_clock::now();
    const double TIME_LIMIT_SECONDS = 4.5 * 60.0; 

    int N = nodes.size();
    rank.assign(N, -1);
    ch_adj.assign(N, {});
    ch_adj_rev.assign(N, {});

    std::vector<int> active_nodes;
    for (int i = 0; i < N; ++i)
        if (nodes[i].id == i)
            active_nodes.push_back(i);

    std::cout << "[CH] Preprocessing " << active_nodes.size() << " nodes..." << std::endl;

    int max_eid = 0;
    for (const auto &p : edges_by_id)
        max_eid = std::max(max_eid, p.first);
    int shortcut_id = max_eid + 1;

    auto get_priority = [&](int u)
    {
        int in = 0, out = 0;
        for (auto &e : adj_rev[u])
            if (rank[e.to] == -1)
                in++;
        for (auto &e : adj[u])
            if (rank[e.to] == -1)
                out++;
        for (auto &e : ch_adj_rev[u])
            if (rank[e.to] == -1)
                in++;
        for (auto &e : ch_adj[u])
            if (rank[e.to] == -1)
                out++;

        return (in * out) - (in + out);
    };

    using P = std::pair<int, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    for (int u : active_nodes)
        pq.push({get_priority(u), u});

    int contracted = 0;
    int cur_rank = 0;

    while (!pq.empty())
    {
        // --- TIME LIMIT CHECK START ---
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = current_time - start_time;
        if (elapsed.count() >= TIME_LIMIT_SECONDS)
        {
            std::cout << "[CH] Time limit of " << TIME_LIMIT_SECONDS / 60.0 << " minutes reached. Stopping early." << std::endl;
            break;
        }
        // --- TIME LIMIT CHECK END ---

        auto [prio, u] = pq.top();
        pq.pop();

        if (rank[u] != -1)
            continue;

        int cur_prio = get_priority(u);
        if (cur_prio > prio)
        {
            pq.push({cur_prio, u});
            continue;
        }

        rank[u] = cur_rank++;
        contract_node(u, shortcut_id);

        if (++contracted % 5000 == 0)
            std::cout << "Contracted: " << contracted << std::endl;
    }
    std::cout << "[CH] Done. Contracted " << contracted << " nodes. Added " << (shortcut_id - max_eid - 1) << " shortcuts." << std::endl;
}
int Graph::num_nodes() const { return nodes.size(); }
