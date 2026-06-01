#pragma once
#include "Graph.hpp"
using json = nlohmann::json;
class QueryProcessor
{
public:
    QueryProcessor(Graph &graph) : graph(graph)
    {
        graph.compute_APSP();
    }
    json process_query(json &query);

private:
    Graph &graph;
    json handle_tsp(const json &query);
};