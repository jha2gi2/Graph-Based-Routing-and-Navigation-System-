#pragma once
#include "Graph.hpp"
#include <vector>
std::vector<std::pair<std::vector<int>, double>> k_shortest_paths_heuristic(const Graph &graph, int source, int target, int k, double overlap_threshold);
