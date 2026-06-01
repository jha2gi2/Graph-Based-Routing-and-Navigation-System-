#pragma once
#include "json.hpp"
#include <vector>
#include <string>
#include <unordered_map>

using json = nlohmann::json;

struct Edge
{
    int id;
    int u, v;
    double length = 0.0;
    double average_time = 0.0;
    std::vector<double> speed_profile;
    bool one_way = false;
    std::string road_type = "";
};

struct Node
{
    int id;
    double lat = 0.0;
    double lon = 0.0;
    std::vector<std::string> pois;
};

struct AdjEntry
{
    int to;
    int edge_id;
    double length;
};

class Graph
{
public:
    Graph() = default;

    void load_graph(const json &graph);

    bool remove_edge(int edge_id);

    bool modify_edge(int edge_id, const json &patch);

    double edge_time(int edge_id, double start_time_seconds) const;

    int nearest_node(double query_lat, double query_lon) const;

    int num_nodes() const;

    void compute_APSP();

    // Helper to reconstruct path between two nodes
    std::vector<int> get_path(int u, int v) const;

    std::vector<std::vector<double>> APSP;
    // Stores the predecessor node for path reconstruction: predecessors[s][v] = u
    std::vector<std::vector<int>> predecessors;

    std::vector<Node> nodes;
    std::unordered_map<int, Edge> edges_by_id;
    std::unordered_map<int, Edge> disabled_edges_by_id;
    std::vector<std::vector<AdjEntry>> adj;
};