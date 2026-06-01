#pragma once
#include "Graph.hpp"
#include <vector>

std::vector<std::pair<std::vector<int>, double>> k_shortest_paths(const Graph &graph, int source, int target, int k, const std::vector<std::vector<double>> &APSP);
