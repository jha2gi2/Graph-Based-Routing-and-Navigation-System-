#include "Graph.hpp"
#include <queue>
#include <unordered_set>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <set>
#include <map>

using namespace std;

const double INF = 1e18;

struct PathMeta
{
    int id;
    vector<int> path;
    double length;
    double dist_penalty_val;
    set<pair<int, int>> edges;
};
double get_overlap_ratio(const PathMeta &A, const PathMeta &B)
{
    if (A.edges.empty() || B.edges.empty())
        return 0.0;

    int common = 0;
    if (A.edges.size() < B.edges.size())
    {
        for (const auto &e : A.edges)
            if (B.edges.count(e))
                common++;
    }
    else
    {
        for (const auto &e : B.edges)
            if (A.edges.count(e))
                common++;
    }

    double min_len = (double)min(A.edges.size(), B.edges.size());
    return (min_len > 0) ? common / min_len : 0.0;
}
double calculate_total_system_penalty(const vector<const PathMeta *> &selected, double theta)
{
    double total_penalty = 0;
    int k = selected.size();

    for (int i = 0; i < k; ++i)
    {
        int overlap_count = 0;
        for (int j = 0; j < k; ++j)
        {
            if (i == j)
            {
                overlap_count++;
            }
            else
            {
                if (get_overlap_ratio(*selected[i], *selected[j]) > theta)
                {
                    overlap_count++;
                }
            }
        }
        double path_penalty = overlap_count * selected[i]->dist_penalty_val;
        total_penalty += path_penalty;
    }
    return total_penalty;
}

struct DNode
{
    double cost;
    int u;
    bool operator>(const DNode &other) const { return cost > other.cost; }
};
pair<double, vector<int>> dijkstra_with_penalty(
    const Graph &g,
    int s,
    int t,
    const map<pair<int, int>, int> &edge_counts,
    double penalty_factor)
{
    int n = g.num_nodes();
    vector<double> dist(n, INF);
    vector<int> parent(n, -1);
    priority_queue<DNode, vector<DNode>, greater<DNode>> pq;

    dist[s] = 0;
    pq.push({0, s});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[u])
            continue;
        if (u == t)
            break; 

        for (const auto &edge : g.adj[u])
        {
            int v = edge.to;
            double multiplier = 1.0;
            auto it = edge_counts.find({u, v});
            if (it != edge_counts.end())
            {
                multiplier += (penalty_factor * it->second);
            }

            double new_dist = d + (edge.length * multiplier);

            if (new_dist < dist[v])
            {
                dist[v] = new_dist;
                parent[v] = u;
                pq.push({new_dist, v});
            }
        }
    }

    if (dist[t] >= INF)
        return {-1.0, {}};

    vector<int> path;
    int curr = t;
    while (curr != -1)
    {
        path.push_back(curr);
        curr = parent[curr];
    }
    reverse(path.begin(), path.end());

    return {dist[t], path};
}

double get_real_path_length(const Graph &g, const vector<int> &path)
{
    double len = 0;
    if (path.empty())
        return INF;
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        int u = path[i];
        int v = path[i + 1];
        bool found = false;
        double min_edge = INF;
        for (const auto &e : g.adj[u])
        {
            if (e.to == v)
            {
                min_edge = min(min_edge, e.length);
                found = true;
            }
        }
        len += min_edge;
        if (!found)
            return INF;
    }
    return len;
}

std::vector<std::pair<std::vector<int>, double>> k_shortest_paths_heuristic(
    const Graph &graph,
    int source,
    int target,
    int k,
    double overlap_threshold_pct)
{
    double theta = overlap_threshold_pct / 100.0;
    double penalty_factor = 0.5;
    int pool_size = max(k * 3, 15);
    vector<PathMeta> pool;
    set<vector<int>> seen_paths_set; 
    map<pair<int, int>, int> edge_usage_counts;

    double true_shortest_len = -1.0;

    for (int iter = 0; iter < pool_size; ++iter)
    {
        auto [penalized_len, p_vec] = dijkstra_with_penalty(graph, source, target, edge_usage_counts, penalty_factor);

        if (p_vec.empty())
            break;
        for (size_t i = 0; i < p_vec.size() - 1; ++i)
        {
            edge_usage_counts[{p_vec[i], p_vec[i + 1]}]++;
        }

        if (seen_paths_set.find(p_vec) == seen_paths_set.end())
        {
            seen_paths_set.insert(p_vec);

            double real_len = get_real_path_length(graph, p_vec);

            if (pool.empty())
                true_shortest_len = real_len;
            double dist_pen = 0.1;
            if (true_shortest_len > 1e-9)
            {
                dist_pen = ((real_len - true_shortest_len) / true_shortest_len) + 0.1;
            }

            set<pair<int, int>> p_edges;
            for (size_t i = 0; i < p_vec.size() - 1; ++i)
                p_edges.insert({p_vec[i], p_vec[i + 1]});

            pool.push_back({(int)pool.size(), p_vec, real_len, dist_pen, p_edges});
        }
    }

    if (pool.empty())
        return {};
    if ((int)pool.size() < k)
        k = pool.size();
    vector<const PathMeta *> selected;
    vector<bool> in_selection(pool.size(), false);

    selected.push_back(&pool[0]);
    in_selection[0] = true;
    while (selected.size() < (size_t)k)
    {
        double best_system_score = std::numeric_limits<double>::max();
        int best_candidate_idx = -1;

        for (size_t i = 1; i < pool.size(); ++i)
        {
            if (in_selection[i])
                continue;
            vector<const PathMeta *> trial_set = selected;
            trial_set.push_back(&pool[i]);
            double score = calculate_total_system_penalty(trial_set, theta);

            if (score < best_system_score)
            {
                best_system_score = score;
                best_candidate_idx = i;
            }
        }

        if (best_candidate_idx != -1)
        {
            selected.push_back(&pool[best_candidate_idx]);
            in_selection[best_candidate_idx] = true;
        }
        else
        {
            break; 
        }
    }
    vector<pair<vector<int>, double>> result;
    result.push_back({selected[0]->path, selected[0]->length});
    vector<const PathMeta *> remainder;
    for (size_t i = 1; i < selected.size(); ++i)
        remainder.push_back(selected[i]);

    sort(remainder.begin(), remainder.end(), [](const PathMeta *a, const PathMeta *b)
         { return a->length < b->length; });

    for (auto *pm : remainder)
    {
        result.push_back({pm->path, pm->length});
    }

    return result;
}
