#include "tsp.hpp"
#include "Graph.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
#include <array>
#include <limits>
#include <random>
#include <chrono>
#include <iostream>

struct AssignmentPair
{
    double dist_sq;
    int point_idx;
    int cluster_idx;
    bool operator<(const AssignmentPair &other) const
    {
        return dist_sq < other.dist_sq;
    }
};
#include <tuple>
#include <random>
#include <chrono>
#include <cmath>
#include <numeric>

std::pair<std::vector<std::tuple<int, std::vector<int>, std::vector<int>>>, double>
tsp(
    const Graph &graph,
    int depot_node,
    int num_drivers,
    const std::vector<Order> &orders,
    double time_limit_seconds)
{
    using namespace std::chrono;
    using std::vector;

    const int M_orders = (int)orders.size();
    if (M_orders == 0)
    {
        std::vector<std::tuple<int, std::vector<int>, std::vector<int>>> res;
        for (int d = 0; d < num_drivers; ++d)
            res.push_back({d, {depot_node}, {}});
        return {res, 0.0};
    }
    const int E = 2 * M_orders;
    vector<int> event_to_node(E);
    for (int i = 0; i < M_orders; ++i)
    {
        event_to_node[2 * i] = orders[i].pickup;
        event_to_node[2 * i + 1] = orders[i].dropoff;
    }
    auto event_dist = [&](int e1, int e2) -> double
    {
        int n1 = event_to_node[e1];
        int n2 = event_to_node[e2];
        if (n1 < (int)graph.APSP.size() && n2 < (int)graph.APSP.size())
            return graph.APSP[n1][n2];
        return std::numeric_limits<double>::infinity();
    };

    auto depot_to_event_dist = [&](int e) -> double
    {
        int n = event_to_node[e];
        if (n < (int)graph.APSP.size())
            return graph.APSP[depot_node][n];
        return std::numeric_limits<double>::infinity();
    };

    auto event_to_depot = [&](int e) -> double
    {
        int n = event_to_node[e];
        if (n < (int)graph.APSP.size())
            return graph.APSP[n][depot_node];
        return std::numeric_limits<double>::infinity();
    };

    vector<vector<int>> routes(num_drivers);
    vector<int> ord_idx(M_orders);
    for (int i = 0; i < M_orders; ++i)
        ord_idx[i] = i;
    std::sort(ord_idx.begin(), ord_idx.end(), [&](int a, int b)
              {
        double da = depot_to_event_dist(2*a);
        double db = depot_to_event_dist(2*b);
        return da < db; });

    for (int i = 0; i < M_orders; ++i)
    {
        int driver = i % num_drivers;
        int oi = ord_idx[i];
        routes[driver].push_back(2 * oi);
        routes[driver].push_back(2 * oi + 1);
    }
    auto route_cost = [&](const vector<int> &r) -> double
    {
        double cum = 0.0;
        double tot = 0.0;
        int prev_node = depot_node;
        for (size_t i = 0; i < r.size(); ++i)
        {
            int ev = r[i];
            int cur_node = event_to_node[ev];
            double step;
            if (prev_node < (int)graph.APSP.size() && cur_node < (int)graph.APSP.size())
                step = graph.APSP[prev_node][cur_node];
            else
                step = std::numeric_limits<double>::infinity();
            if (step >= 1e15)
                step = 1e9;
            cum += step;
            if ((ev & 1) == 1)
            {
                tot += cum;
            }
            prev_node = cur_node;
        }
        return tot;
    };

    auto is_route_pd_feasible = [&](const vector<int> &r) -> bool
    {
        std::vector<char> seen(M_orders, 0);
        for (int ev : r)
        {
            int ord = ev >> 1; 
            if ((ev & 1) == 0)
            { 
                seen[ord] = 1;
            }
            else
            { 
                if (!seen[ord])
                    return false;
            }
        }
        return true;
    };

    auto repair_route_feasible = [&](vector<int> &r)
    {
        std::vector<int> pos_pick(M_orders, -1);
        for (size_t i = 0; i < r.size(); ++i)
        {
            int ev = r[i];
            if ((ev & 1) == 0)
            { 
                pos_pick[ev >> 1] = (int)i;
            }
        }
        for (size_t i = 0; i < r.size(); ++i)
        {
            int ev = r[i];
            if ((ev & 1) == 1)
            {
                int ord = ev >> 1;
                if (pos_pick[ord] == -1 || pos_pick[ord] > (int)i)
                {
                    r.erase(r.begin() + i);
                    int ins = (pos_pick[ord] == -1) ? (int)r.size() : pos_pick[ord] + 1;
                    r.insert(r.begin() + ins, ev);
                    pos_pick.assign(M_orders, -1);
                    for (size_t j = 0; j < r.size(); ++j)
                    {
                        int ev2 = r[j];
                        if ((ev2 & 1) == 0)
                            pos_pick[ev2 >> 1] = (int)j;
                    }
                }
            }
        }
    };

    for (auto &rt : routes)
    {
        if (!is_route_pd_feasible(rt))
            repair_route_feasible(rt);
    }
    vector<double> route_costs(num_drivers, 0.0);
    double global_cost = 0.0;
    for (int d = 0; d < num_drivers; ++d)
    {
        route_costs[d] = route_cost(routes[d]);
        global_cost += route_costs[d];
    }
    std::mt19937 rng((unsigned)std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> ur01(0.0, 1.0);
    double avg_cost_per_order = (global_cost > 0.0) ? (global_cost / (double)M_orders) : 1000.0;
    double T = std::max(10000.0, avg_cost_per_order * 0.6 * M_orders / std::max(1, num_drivers)); 
    double T_min = 1e-4;
    double cooling_factor = 0.999999; 
    double best_global_cost = global_cost;

    double p_relocate = 0.5;
    double p_swap = 0.3;
    double p_intra_2opt = 0.2;


    auto time_start = high_resolution_clock::now();
    auto time_limit = std::chrono::duration<double>(time_limit_seconds);

    vector<vector<int>> best_routes = routes;
    vector<double> best_route_costs = route_costs;

    auto compute_route_cost = [&](int d)
    {
        return route_cost(routes[d]);
    };

    auto pd_reverse_feasible = [&](const vector<int> &r, int l, int rr) -> bool
    {

        int len = (int)r.size();
        if (l < 0 || rr >= len || l >= rr)
            return false;

        std::vector<char> inside(M_orders, 0);
        for (int i = l; i <= rr; ++i)
        {
            int ev = r[i];
            inside[ev >> 1] ^= 1;
            if (inside[ev >> 1] == 0)
            {
                return false;
            }
        }
        std::vector<char> seen(M_orders, 0);
        for (int i = l; i <= rr; ++i)
        {
            int ev = r[i];
            int ord = ev >> 1;
            if (seen[ord])
                return false; 
            seen[ord] = 1;
        }
        return true;
    };

    auto compute_cost_delta_two_routes = [&](int dA, const vector<int> &candA, int dB, const vector<int> &candB)
    {
        double oldsum = route_costs[dA] + route_costs[dB];
        double newsum = route_cost(candA) + route_cost(candB);
        return newsum - oldsum;
    };

    auto insert_order_into_route = [&](vector<int> &r, int order_idx, int pos_p, int pos_d)
    {
        int evp = 2 * order_idx, evd = 2 * order_idx + 1;
        if (pos_p < 0)
            pos_p = 0;
        if (pos_p > (int)r.size())
            pos_p = (int)r.size();
        if (pos_d < pos_p)
            pos_d = pos_p;
        if (pos_d > (int)r.size())
            pos_d = (int)r.size();
        r.insert(r.begin() + pos_p, evp);
        r.insert(r.begin() + (pos_d + (pos_d >= pos_p ? 1 : 0)), evd);
    };

    while (std::chrono::high_resolution_clock::now() - time_start < time_limit)
    {
        double p = ur01(rng);

        if (p < p_relocate)
        {
            int order_idx = rng() % M_orders;
            int driver_from = -1;
            int pos_pick = -1, pos_del = -1;
            for (int d = 0; d < num_drivers; ++d)
            {
                auto &r = routes[d];
                for (size_t i = 0; i < r.size(); ++i)
                {
                    if (r[i] == 2 * order_idx)
                    {
                        driver_from = d;
                        pos_pick = (int)i;
                    }
                    if (r[i] == 2 * order_idx + 1)
                    {
                        pos_del = (int)i;
                    }
                }
                if (driver_from != -1 && pos_del != -1)
                    break;
            }
            if (driver_from == -1)
                continue; 
            vector<int> rA = routes[driver_from];
            if (pos_del > pos_pick)
            {
                rA.erase(rA.begin() + pos_del);
                rA.erase(rA.begin() + pos_pick);
            }
            else
            {
                rA.erase(rA.begin() + pos_pick);
                rA.erase(rA.begin() + pos_del);
            }
            int driver_to = rng() % num_drivers;
            vector<int> rB = routes[driver_to];
            int pos_p = (rB.empty()) ? 0 : (rng() % (rB.size() + 1));
            int pos_d = pos_p + (rng() % ((rB.size() - pos_p) + 1));
            if (driver_from == driver_to)
            {
                vector<int> cand = rA;
                insert_order_into_route(cand, order_idx, pos_p, pos_d);
                if (!is_route_pd_feasible(cand))
                    continue;
                double oldc = route_costs[driver_from];
                double newc = route_cost(cand);
                double delta = newc - oldc;
                if (delta < 0 || ur01(rng) < std::exp(-delta / T))
                {
                    routes[driver_from] = std::move(cand);
                    route_costs[driver_from] = newc;
                    global_cost += delta;
                    if (global_cost < best_global_cost)
                    {
                        best_global_cost = global_cost;
                        best_routes = routes;
                        best_route_costs = route_costs;
                    }
                }
            }
            else
            {
                insert_order_into_route(rB, order_idx, pos_p, pos_d);
                if (!is_route_pd_feasible(rA) || !is_route_pd_feasible(rB))
                    continue;
                double oldpair = route_costs[driver_from] + route_costs[driver_to];
                double newpair = route_cost(rA) + route_cost(rB);
                double delta = newpair - oldpair;
                if (delta < 0 || ur01(rng) < std::exp(-delta / T))
                {
                    routes[driver_from] = std::move(rA);
                    routes[driver_to] = std::move(rB);
                    route_costs[driver_from] = route_cost(routes[driver_from]);
                    route_costs[driver_to] = route_cost(routes[driver_to]);
                    global_cost += delta;
                    if (global_cost < best_global_cost)
                    {
                        best_global_cost = global_cost;
                        best_routes = routes;
                        best_route_costs = route_costs;
                    }
                }
            }
        }
        else if (p < p_relocate + p_swap)
        {
            int oa = rng() % M_orders;
            int ob = rng() % M_orders;
            if (oa == ob)
                continue;
            int da = -1, db = -1, a_pick = -1, a_del = -1, b_pick = -1, b_del = -1;
            for (int d = 0; d < num_drivers; ++d)
            {
                auto &r = routes[d];
                for (size_t i = 0; i < r.size(); ++i)
                {
                    if (r[i] == 2 * oa)
                    {
                        da = d;
                        a_pick = (int)i;
                    }
                    if (r[i] == 2 * oa + 1)
                    {
                        a_del = (int)i;
                    }
                    if (r[i] == 2 * ob)
                    {
                        db = d;
                        b_pick = (int)i;
                    }
                    if (r[i] == 2 * ob + 1)
                    {
                        b_del = (int)i;
                    }
                }
            }
            if (da == -1 || db == -1 || da == db)
                continue;
            vector<int> rA = routes[da];
            vector<int> rB = routes[db];
            int a_hi = std::max(a_pick, a_del), a_lo = std::min(a_pick, a_del);
            rA.erase(rA.begin() + a_hi);
            rA.erase(rA.begin() + a_lo);
            int b_hi = std::max(b_pick, b_del), b_lo = std::min(b_pick, b_del);
            rB.erase(rB.begin() + b_hi);
            rB.erase(rB.begin() + b_lo);
            int pos_pa = (rA.empty()) ? 0 : (rng() % (rA.size() + 1));
            int pos_da = pos_pa + (rng() % ((rA.size() - pos_pa) + 1));
            int pos_pb = (rB.empty()) ? 0 : (rng() % (rB.size() + 1));
            int pos_db = pos_pb + (rng() % ((rB.size() - pos_pb) + 1));
            insert_order_into_route(rA, ob, pos_pa, pos_da);
            insert_order_into_route(rB, oa, pos_pb, pos_db);
            if (!is_route_pd_feasible(rA) || !is_route_pd_feasible(rB))
                continue;
            double oldpair = route_costs[da] + route_costs[db];
            double newpair = route_cost(rA) + route_cost(rB);
            double delta = newpair - oldpair;
            if (delta < 0 || ur01(rng) < std::exp(-delta / T))
            {
                routes[da] = std::move(rA);
                routes[db] = std::move(rB);
                route_costs[da] = route_cost(routes[da]);
                route_costs[db] = route_cost(routes[db]);
                global_cost += delta;
                if (global_cost < best_global_cost)
                {
                    best_global_cost = global_cost;
                    best_routes = routes;
                    best_route_costs = route_costs;
                }
            }
        }
        else
        {
            int d = rng() % num_drivers;
            auto &r = routes[d];
            int n = (int)r.size();
            if (n < 4)
            {
                T *= cooling_factor;
                continue;
            }
            int i = 0, j = 0;
            i = rng() % (n - 2);
            j = i + 1 + (rng() % (n - i - 1));
            if (!pd_reverse_feasible(r, i, j))
            {
                T *= cooling_factor;
                continue;
            }
            vector<int> cand = r;
            std::reverse(cand.begin() + i, cand.begin() + j + 1);
            if (!is_route_pd_feasible(cand))
            {
                T *= cooling_factor;
                continue;
            }
            double oldc = route_costs[d];
            double newc = route_cost(cand);
            double delta = newc - oldc;
            if (delta < 0 || ur01(rng) < std::exp(-delta / T))
            {
                routes[d] = std::move(cand);
                route_costs[d] = newc;
                global_cost += delta;
                if (global_cost < best_global_cost)
                {
                    best_global_cost = global_cost;
                    best_routes = routes;
                    best_route_costs = route_costs;
                }
            }
        }
        T = std::max(T_min, T * cooling_factor);
    } 
    routes = best_routes;
    route_costs = best_route_costs;
    global_cost = best_global_cost;
    std::vector<std::tuple<int, std::vector<int>, std::vector<int>>> results;
    results.reserve(num_drivers);

    for (int d = 0; d < num_drivers; ++d)
    {
        auto &r = routes[d];
        std::vector<int> full_nodes;
        full_nodes.push_back(depot_node);
        int prev_node = depot_node;
        for (size_t i = 0; i < r.size(); ++i)
        {
            int ev = r[i];
            int cur_node = event_to_node[ev];
            std::vector<int> segment = graph.get_path(prev_node, cur_node);
            if (segment.empty())
            {
                full_nodes.push_back(cur_node);
            }
            else
            {
                for (size_t s = 1; s < segment.size(); ++s)
                    full_nodes.push_back(segment[s]);
            }
            prev_node = cur_node;
        }
        std::vector<int> order_seq;
        order_seq.reserve(r.size() / 2);
        for (size_t i = 0; i < r.size(); ++i)
        {
            int ev = r[i];
            if ((ev & 1) == 1)
            {
                order_seq.push_back(orders[ev >> 1].order_id);
            }
        }
        if (r.empty())
        {
            results.push_back({d, {depot_node}, {}});
        }
        else
        {
            results.push_back({d, full_nodes, order_seq});
        }
    }

    return {results, global_cost};
}
