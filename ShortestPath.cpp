#include "Graph.hpp"
#include <queue>
#include <unordered_set>
#include <map>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <set>

using namespace std;

double heuristic(const Graph &graph, int a, int b)
{
    double dx = graph.nodes[a].lat - graph.nodes[b].lat;
    double dy = graph.nodes[a].lon - graph.nodes[b].lon;
    return sqrt(dx * dx + dy * dy);
}

vector<int> shortest_distance(const Graph &graph, int source, int target, const vector<int> &forbidden_nodes, const vector<string> &forbidden_road_types, double *cost)
{
    int n = graph.nodes.size();
    *cost = -1;
    unordered_set<int> forbiddenNodeSet(forbidden_nodes.begin(), forbidden_nodes.end());
    unordered_set<string> forbiddenRoadSet(forbidden_road_types.begin(), forbidden_road_types.end());
    if (forbiddenNodeSet.count(source) || forbiddenNodeSet.count(target))
        return {};
    using P = pair<double, int>;
    priority_queue<P, vector<P>, greater<P>> openSet;
    vector<double> g(n, numeric_limits<double>::infinity());
    vector<double> f(n, numeric_limits<double>::infinity());
    vector<int> parent(n, -1);
    g[source] = 0;
    f[source] = heuristic(graph, source, target);
    openSet.emplace(f[source], source);

    while (!openSet.empty())
    {
        auto [current_f, u] = openSet.top();
        openSet.pop();
        if (u == target)
        {
            *cost = g[u];
            vector<int> path;
            for (int v = u; v != -1; v = parent[v])
                path.push_back(v);
            reverse(path.begin(), path.end());
            return path;
        }

        if (forbiddenNodeSet.count(u))
            continue;

        for (const auto &adj_entry : graph.adj[u])
        {
            int v = adj_entry.to;
            const Edge &e = graph.edges_by_id.at(adj_entry.edge_id);

            if (forbiddenNodeSet.count(v))
                continue;
            if (forbiddenRoadSet.count(e.road_type))
                continue;

            double tentative_g = g[u] + e.length;
            if (tentative_g < g[v])
            {
                parent[v] = u;
                g[v] = tentative_g;
                f[v] = tentative_g + heuristic(graph, v, target);
                openSet.emplace(f[v], v);
            }
        }
    }
    *cost = -1;
    return {};
}

vector<int> shortest_time(const Graph &graph, int source, int target, const vector<int> &forbidden_nodes, const vector<string> &forbidden_road_types, double *cost)
{
    int n = graph.num_nodes();
    if (n == 0)
        return {};
    unordered_set<int> forbidden_nodes_set(forbidden_nodes.begin(), forbidden_nodes.end());
    unordered_set<string> forbidden_types_set(forbidden_road_types.begin(), forbidden_road_types.end());
    if (forbidden_nodes_set.count(source) || forbidden_nodes_set.count(target))
    {
        *cost = -1;
        return {};
    }
    vector<double> g(n, numeric_limits<double>::infinity()); 
    map<int, int> parent;
    using PQEntry = pair<double, int>;
    priority_queue<PQEntry, vector<PQEntry>, greater<PQEntry>> pq;
    g[source] = 0.0;
    pq.push({heuristic(graph, source, target), source});

    while (!pq.empty())
    {
        auto [f_score, u] = pq.top();
        pq.pop();
        double current_time = g[u];
        if (u == target)
        {
            *cost = g[target];
            vector<int> path;
            int curr = target;
            while (curr != source)
            {
                path.push_back(curr);
                curr = parent[curr];
            }
            path.push_back(source);
            reverse(path.begin(), path.end());
            return path;
        }

        for (const auto &adj_entry : graph.adj[u])
        {
            int v = adj_entry.to;
            int edge_id = adj_entry.edge_id;
            if (forbidden_nodes_set.count(v))
                continue;
            auto it = graph.edges_by_id.find(edge_id);
            if (it == graph.edges_by_id.end())
                continue;
            const Edge &edge = it->second;
            if (forbidden_types_set.count(edge.road_type))
                continue;
            double travel_time = graph.edge_time(edge_id, current_time);
            if (travel_time == numeric_limits<double>::infinity())
                continue;

            double tentative_g = g[u] + travel_time;
            if (tentative_g < g[v])
            {
                g[v] = tentative_g;
                parent[v] = u;
                double f_v = tentative_g + heuristic(graph, v, target);
                pq.push({f_v, v});
            }
        }
    }
    *cost = -1;
    return {};
}