#include "knn.hpp"
#include "ShortestPath.hpp" // For running SSSP
#include <cmath>
#include <vector>
#include <string>
#include <queue>
#include <limits>
#include <algorithm>
#include <set>
#include <map>

using namespace std;
double squared_euclidean_dist(double lat1, double lon1, double lat2, double lon2)
{
    return (lat1 - lat2) * (lat1 - lat2) + (lon1 - lon2) * (lon1 - lon2);
}

bool has_poi(const Node &node, const string &poi_type)
{
    for (const auto &poi : node.pois)
    {
        if (poi == poi_type)
            return true;
    }
    return false;
}

vector<int> knn_euclidean(const Graph &graph, double query_lat, double query_lon, const string &poi_type, int k)
{
    priority_queue<pair<double, int>> pq;
    for (const auto &node : graph.nodes)
    {
        if (has_poi(node, poi_type))
        {
            double dist = squared_euclidean_dist(query_lat, query_lon, node.lat, node.lon);
            if (static_cast<int>(pq.size()) < k)
            {
                pq.push({dist, node.id});
            }
            else if (dist < pq.top().first)
            {
                pq.pop();
                pq.push({dist, node.id});
            }
        }
    }
    vector<int> result;
    while (!pq.empty())
    {
        result.push_back(pq.top().second);
        pq.pop();
    }
    reverse(result.begin(), result.end());
    return result;
}

vector<int> knn_shortest_path(const Graph &graph, double query_lat, double query_lon, const string &poi_type, int k)
{
    int start_node = graph.nearest_node(query_lat, query_lon);
    if (start_node == -1)
        return {};
    int n = graph.num_nodes();
    vector<double> dist(n, numeric_limits<double>::infinity());
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    dist[start_node] = 0.0;
    pq.push({0.0, start_node});
    vector<int> found_pois;
    found_pois.reserve(k);
    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u])
            continue;
        if (has_poi(graph.nodes[u], poi_type))
        {
            found_pois.push_back(u);
            if ((int)found_pois.size() == k)
                return found_pois;
        }

        for (const auto &adj_entry : graph.adj[u])
        {
            int v = adj_entry.to;
            int edge_id = adj_entry.edge_id;
            auto it = graph.edges_by_id.find(edge_id);
            if (it == graph.edges_by_id.end())
                continue;

            double w = it->second.length;
            if (dist[u] + w < dist[v])
            {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
    }
    return found_pois;
}