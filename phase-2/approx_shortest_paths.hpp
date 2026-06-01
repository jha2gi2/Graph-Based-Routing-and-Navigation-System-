#pragma once
#include "Graph.hpp"
#include <vector>
#include <chrono>

double approximate_shortest_path(const Graph &graph, int source, int target, std::chrono::high_resolution_clock::time_point global_deadline, double acceptable_error_pct);