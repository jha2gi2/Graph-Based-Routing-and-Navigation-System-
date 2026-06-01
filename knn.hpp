#pragma once
#include "Graph.hpp"
#include <vector>

std::vector<int> knn_euclidean(const Graph &graph, double query_lat, double query_lon, const std::string &poi_type, int k);
std::vector<int> knn_shortest_path(const Graph &graph, double query_lat, double query_lon, const std::string &poi_type, int k);