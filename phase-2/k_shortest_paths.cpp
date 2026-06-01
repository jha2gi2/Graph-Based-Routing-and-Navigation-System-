#include "Graph.hpp"
#include <queue>
#include <unordered_set>
#include <map>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <set>
#include <functional>
#include <unordered_map>

using namespace std;
//heuristic
double h(int u, int v, const vector<vector<double>> &APSP)
{
    if (!APSP.empty() && u < (int)APSP.size() && v < (int)APSP.size())
    {
        double dist = APSP[u][v];
        if (dist != numeric_limits<double>::infinity())
        {
            return dist;
        }
    }
    return 0.0;
}

//A*
pair<vector<int>, double> astar(const Graph &graph, int source, int target,
                                const unordered_set<int> &forbidden_nodes,
                                const set<pair<int, int>> &forbidden_edges,
                                const vector<vector<double>> &APSP)
{
    int n = graph.num_nodes();
    if (forbidden_nodes.count(source) || forbidden_nodes.count(target))
        return {{}, numeric_limits<double>::infinity()};

    vector<double> g(n, numeric_limits<double>::infinity());
    vector<double> f(n, numeric_limits<double>::infinity());
    vector<int> parent(n, -1);

    using P = pair<double, int>;
    priority_queue<P, vector<P>, greater<P>> openSet;

    g[source] = 0;
    f[source] = h(source, target, APSP);
    openSet.emplace(f[source], source);

    while (!openSet.empty())
    {
        auto [current_f, u] = openSet.top();
        openSet.pop();
        if (current_f > f[u]) continue;

        if (u == target)
        {
            vector<int> path;
            for (int v = u; v != -1; v = parent[v])
                path.push_back(v);
            reverse(path.begin(), path.end());
            return {path, g[u]};
        }

        for (const auto &adj_entry : graph.adj[u])
        {
            int v = adj_entry.to;

            //checking constraint
            if (forbidden_nodes.count(v)) continue;
            if (forbidden_edges.count({u, v})) continue;

            const Edge &e = graph.edges_by_id.at(adj_entry.edge_id);
            double tentative_g = g[u] + e.length;
            if (tentative_g < g[v])
            {
                parent[v] = u;
                g[v] = tentative_g;
                f[v] = tentative_g + h(v, target, APSP);
                openSet.emplace(f[v], v);
            }
        }
    }

    return {{}, numeric_limits<double>::infinity()};
}
vector<pair<vector<int>, double>> k_shortest_paths(const Graph &graph, int source, int target, int k, const vector<vector<double>> &APSP)
{
    vector<pair<vector<int>, double>> A;
    auto first = astar(graph, source, target, {}, {}, APSP);
    if (first.first.empty())
        return A;

    A.push_back(first);
    set<vector<int>> generated_paths;
    generated_paths.insert(first.first);
    using PQEntry = pair<double, vector<int>>;
    auto cmp = [](const PQEntry &a, const PQEntry &b) { 
        return a.first > b.first; 
    };
    priority_queue<PQEntry, vector<PQEntry>, decltype(cmp)> B(cmp);
    for (int i = 1; i < k; ++i)
    {
        const auto &prev_path = A[i - 1].first;
        double current_root_cost = 0.0;
        for (size_t j = 0; j < prev_path.size() - 1; j++)
        {
            int spur_node = prev_path[j];
            vector<int> root_path(prev_path.begin(), prev_path.begin() + j + 1);

            if (j > 0) {
                 int u = prev_path[j-1];
                 int v = prev_path[j];
                 for (const auto &adj : graph.adj[u]) {
                    if (adj.to == v) {
                        current_root_cost += graph.edges_by_id.at(adj.edge_id).length;
                        break; 
                    }
                 }
            }

            // 1. Forbidden Nodes: 
            unordered_set<int> forbidden_nodes(root_path.begin(), root_path.end() - 1);
            
            // 2. Forbidden Edges: 
            set<pair<int, int>> forbidden_edges;

            for (const auto &p : A)
            {
                const auto &path = p.first;
                if (path.size() > j && equal(root_path.begin(), root_path.end(), path.begin()))
                {
                    forbidden_edges.insert({path[j], path[j + 1]});
                }
            }

            auto spur_result = astar(graph, spur_node, target, forbidden_nodes, forbidden_edges, APSP);
            auto &spur_path = spur_result.first;
            double spur_cost = spur_result.second;

            if (spur_path.empty())
                continue;
            vector<int> total_path = root_path;
            total_path.insert(total_path.end(), spur_path.begin() + 1, spur_path.end());

            if (generated_paths.count(total_path))
                continue;

            double total_cost = current_root_cost + spur_cost;

            B.push({total_cost, total_path});
            generated_paths.insert(total_path);
        }

        if (B.empty())
            break;

        // Move best candidate to A
        A.push_back({B.top().second, B.top().first});
        B.pop();
    }

    return A;
}