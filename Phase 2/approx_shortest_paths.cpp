#include "approx_shortest_paths.hpp"
#include "Graph.hpp"
#include <queue>
#include <vector>
#include <limits>
#include <algorithm>
#include <chrono>

static std::vector<double> g_fwd;
static std::vector<double> g_bwd;
static std::vector<int> visited_fwd;
static std::vector<int> visited_bwd;
static std::vector<int> parent_edge_fwd;
static std::vector<int> parent_edge_bwd;
static int query_id = 0;
std::vector<int> path_nodes;

void unpack_edge(const Graph &graph, int start_edge_id, int start_node, std::vector<int> &out_path)
{
    struct Task
    {
        int edge_id;
        int current_u;
    };
    std::vector<Task> stack;
    stack.reserve(64);
    stack.push_back({start_edge_id, start_node});

    while (!stack.empty())
    {
        Task t = stack.back();
        stack.pop_back();
        const auto &edge = graph.edges_by_id.at(t.edge_id);

        if (edge.shortcut_e1 == -1)
        {
            int next_node = (edge.u == t.current_u) ? edge.v : edge.u;
            out_path.push_back(next_node);
        }
        else
        {
            const auto &e1_obj = graph.edges_by_id.at(edge.shortcut_e1);
            int middle_node = (e1_obj.u == t.current_u) ? e1_obj.v : e1_obj.u;
            stack.push_back({edge.shortcut_e2, middle_node});
            stack.push_back({edge.shortcut_e1, t.current_u});
        }
    }
}

double approximate_shortest_path(const Graph &graph, int source, int target,
                                 std::chrono::high_resolution_clock::time_point global_deadline,
                                 double acceptable_error_pct)
{
    path_nodes.clear();
    if (source < 0 || source >= graph.num_nodes() || target < 0 || target >= graph.num_nodes())
        return -1.0;
    if (source == target)
    {
        path_nodes.push_back(source);
        return 0.0;
    }
    if (graph.rank.empty())
        return -1.0;

    query_id++;
    size_t N = graph.nodes.size();
    if (visited_fwd.size() < N)
    {
        visited_fwd.assign(N, 0);
        visited_bwd.assign(N, 0);
        g_fwd.assign(N, std::numeric_limits<double>::infinity());
        g_bwd.assign(N, std::numeric_limits<double>::infinity());
        parent_edge_fwd.assign(N, -1);
        parent_edge_bwd.assign(N, -1);
    }

    double epsilon = 1.0 + 0 * (acceptable_error_pct / 100.0);
    using Pair = std::pair<double, int>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq_fwd;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq_bwd;

    g_fwd[source] = 0.0;
    visited_fwd[source] = query_id;
    pq_fwd.push({0.0, source});
    g_bwd[target] = 0.0;
    visited_bwd[target] = query_id;
    pq_bwd.push({0.0, target});

    double mu = std::numeric_limits<double>::infinity();
    int meet_node = -1;

    while (!pq_fwd.empty() || !pq_bwd.empty())
    {
        if (query_id % 256 == 0 && std::chrono::high_resolution_clock::now() > global_deadline)
            return -1.0;

        double min_f = pq_fwd.empty() ? std::numeric_limits<double>::infinity() : pq_fwd.top().first;
        double min_b = pq_bwd.empty() ? std::numeric_limits<double>::infinity() : pq_bwd.top().first;
        if (mu != std::numeric_limits<double>::infinity() && min_f + min_b > mu / epsilon)
            break;

        // FORWARD SEARCH
        if (!pq_fwd.empty())
        {
            auto [d_u, u] = pq_fwd.top();
            pq_fwd.pop();
            if (d_u <= g_fwd[u] && d_u < mu)
            {

                auto process_fwd = [&](const std::vector<AdjEntry> &list)
                {
                    for (const auto &e : list)
                    {
                        if (graph.rank[e.to] <= graph.rank[u])
                            continue;
                        double new_dist = d_u + e.length;
                        if (visited_fwd[e.to] != query_id || new_dist < g_fwd[e.to])
                        {
                            g_fwd[e.to] = new_dist;
                            visited_fwd[e.to] = query_id;
                            parent_edge_fwd[e.to] = e.edge_id;
                            pq_fwd.push({new_dist, e.to});
                            if (visited_bwd[e.to] == query_id)
                            {
                                double path = new_dist + g_bwd[e.to];
                                if (path < mu)
                                {
                                    mu = path;
                                    meet_node = e.to;
                                }
                            }
                        }
                    }
                };

                process_fwd(graph.adj[u]);
                process_fwd(graph.ch_adj[u]);
            }
        }

        // BACKWARD SEARCH
        if (!pq_bwd.empty())
        {
            auto [d_u, u] = pq_bwd.top();
            pq_bwd.pop();
            if (d_u <= g_bwd[u] && d_u < mu)
            {

                auto process_bwd = [&](const std::vector<AdjEntry> &list)
                {
                    for (const auto &e : list)
                    {
                        if (graph.rank[e.to] <= graph.rank[u])
                            continue;
                        double new_dist = d_u + e.length;
                        if (visited_bwd[e.to] != query_id || new_dist < g_bwd[e.to])
                        {
                            g_bwd[e.to] = new_dist;
                            visited_bwd[e.to] = query_id;
                            parent_edge_bwd[e.to] = e.edge_id;
                            pq_bwd.push({new_dist, e.to});
                            if (visited_fwd[e.to] == query_id)
                            {
                                double path = new_dist + g_fwd[e.to];
                                if (path < mu)
                                {
                                    mu = path;
                                    meet_node = e.to;
                                }
                            }
                        }
                    }
                };

                process_bwd(graph.adj_rev[u]);
                process_bwd(graph.ch_adj_rev[u]);
            }
        }
    }

    if (meet_node == -1)
        return -1.0;

    std::vector<int> fwd_edges;
    int curr = meet_node;
    while (curr != source)
    {
        int e_id = parent_edge_fwd[curr];
        if (e_id == -1)
            break;
        fwd_edges.push_back(e_id);
        const auto &edge = graph.edges_by_id.at(e_id);
        curr = (edge.v == curr) ? edge.u : edge.v;
    }
    std::reverse(fwd_edges.begin(), fwd_edges.end());

    std::vector<int> bwd_edges;
    curr = meet_node;
    while (curr != target)
    {
        int e_id = parent_edge_bwd[curr];
        if (e_id == -1)
            break;
        bwd_edges.push_back(e_id);
        const auto &edge = graph.edges_by_id.at(e_id);
        curr = (edge.u == curr) ? edge.v : edge.u;
    }

    path_nodes.push_back(source);
    int current_node = source;
    for (int eid : fwd_edges)
    {
        unpack_edge(graph, eid, current_node, path_nodes);
        current_node = path_nodes.back();
    }
    for (int eid : bwd_edges)
    {
        unpack_edge(graph, eid, current_node, path_nodes);
        current_node = path_nodes.back();
    }

    return mu;
}
