#pragma once
#include "Graph.hpp"
using json = nlohmann::json;
class QueryProcessor
{
public:
    QueryProcessor(Graph &graph) : graph(graph) {}
    json process_query(json &query);

private:
    Graph & graph;
    json handle_remove_edge(const json &query);
    json handle_modify_edge(const json &query);
    json handle_shortest_path(const json &query);
    json handle_knn(const json &query);
};
