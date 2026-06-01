#pragma once
#include "Graph.hpp"
#include <iostream>
using json = nlohmann::json;
class QueryProcessor
{
public:
    QueryProcessor(Graph &graph) : graph(graph)
    {
        if (graph.num_nodes() <= 5000)
            graph.compute_APSP();
        auto start = std::chrono::high_resolution_clock::now();
        std::cout << "CH start: " << std::endl;
        graph.preprocess_ch();
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "CH Precomputation took: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << "ms" << std::endl;
        graph.compute_landmarks();
    }
    json process_query(json &query);

private:
    Graph &graph;
    json handle_k_shortest_paths(const json &query);
    json handle_k_shortest_paths_heuristic(const json &query);
    json handle_approx_shortest_path(const json &query);
};
