#pragma once
#include "Graph.hpp"
#include <vector>
struct Order
{
    int order_id;
    int pickup;
    int dropoff;
};
struct Order4D {
    int order_idx;
    double v[4];
};

std::pair<std::vector<std::tuple<int, std::vector<int>, std::vector<int>>>, double> tsp(const Graph &graph, int depot_node, int num_drivers, const std::vector<struct Order> &orders, double time_limit_seconds);
//tried multithreading but it didnt work out
// std::pair<std::vector<std::tuple<int, std::vector<int>, std::vector<int>>>, double>
// tsp_parallel_multistart(
//     const Graph &graph,
//     int depot_node,
//     int num_drivers,
//     const std::vector<Order> &orders,
//     double total_time_seconds,
//     int num_threads = 0
// );