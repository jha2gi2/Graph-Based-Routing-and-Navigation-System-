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
    
    // Shortcuts specific fields
    int shortcut_e1 = -1; 
    int shortcut_e2 = -1; 
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
    void compute_landmarks();
    void preprocess_ch();

    // CH Specific Data
    std::vector<int> rank;
    std::vector<std::vector<AdjEntry>> ch_adj;      // ONLY Shortcuts
    std::vector<std::vector<AdjEntry>> ch_adj_rev;  // ONLY Shortcuts (Reverse)

    // Standard Data (Original Graph)
    std::vector<std::vector<AdjEntry>> adj;         // ONLY Original Edges
    std::vector<std::vector<AdjEntry>> adj_rev;     // ONLY Original Edges (Reverse)

    // Scratchpads
    std::vector<double> w_dist;
    std::vector<int> w_visited_token;
    int w_search_id = 0; 

    bool APSP_precomputed = false;
    std::vector<std::vector<double>> APSP;
    std::vector<std::vector<double>> landmarks_dist;
    std::vector<Node> nodes;
    std::unordered_map<int, Edge> edges_by_id;
    std::unordered_map<int, Edge> disabled_edges_by_id;

private:
    void contract_node(int u, int &shortcut_id_counter);
    bool witness_search(int u, int v, int ignore_node, double max_dist);
};