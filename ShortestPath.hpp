#pragma once
#include "Graph.hpp"
#include <vector>

std::vector<int> shortest_distance(const Graph &graph, int source, int target, const std::vector<int> &forbidden_nodes, const std::vector<std::string> &forbidden_road_types, double *cost);
std::vector<int> shortest_time(const Graph &graph, int source, int target, const std::vector<int> &forbidden_nodes, const std::vector<std::string> &forbidden_road_types, double *cost);