# Graph-Based Routing and Navigation System

A scalable graph-based routing engine inspired by real-world navigation and delivery systems such as Google Maps and Zomato.

This project implements efficient graph algorithms for shortest path computation, dynamic routing, K-shortest paths, approximate routing acceleration, and delivery scheduling optimization on large-scale road networks.

---

# Features

## Phase 1 — Core Routing Engine
- Dynamic graph updates
- Shortest path computation
- Time-dependent routing with traffic profiles
- Constraint-based routing
- K nearest neighbor (KNN) queries

## Phase 2 — Advanced Routing
- Exact K shortest paths
- Heuristic diversified routing
- Approximate shortest path acceleration
- Fast query processing on large graphs

## Phase 3 — Delivery Scheduling
- Multi-order delivery optimization
- Pickup-before-dropoff constraints
- Fleet route assignment
- Delivery completion time minimization

---

# Algorithms Used

| Module | Algorithm |
|---|---|
| Shortest Path | A* Search |
| Time Routing | Dynamic Weighted Graph Traversal |
| KNN Queries | Heap + Dijkstra |
| Exact K Shortest Paths | Yen’s Algorithm |
| Heuristic Routing | Overlap Penalization Heuristics |
| Approximate Routing | Contraction-Hierarchy Inspired Optimization |
| Delivery Scheduling | Greedy + Local Search Heuristics |

---

# Project Structure

```text
Graph-Based-Routing-System/
│
├── Phase-1/
├── Phase-2/
├── Phase-3/
├── testcases/
├── README.md
└── Makefile
```

---

# Build Instructions

---

# Build Instructions

Compile executables using:

bash make phase1 make phase2 make phase3 

---

# Running the Project

## Phase 1

bash ./phase1 graph.json queries.json output.json 

## Phase 2

bash ./phase2 graph.json queries.json output.json 

## Phase 3

bash ./phase3 graph.json queries.json output.json 

---

# Query Types

## Phase 1
- Shortest Path
- Dynamic Edge Updates
- KNN Queries

## Phase 2
- Exact K Shortest Paths
- Heuristic K Shortest Paths
- Approximate Shortest Path

## Phase 3
- Delivery Scheduling
- Fleet Assignment
- Route Optimization

---

# Graph Model

The road network is modeled as a weighted graph:

- Nodes represent intersections or locations
- Edges represent roads
- Edge weights represent:
  - Distance
  - Average travel time
  - Dynamic traffic conditions

The system supports:
- One-way roads
- Dynamic road closures
- Time-varying speed profiles
- Road-type constraints

---

# Performance Optimizations

- A* heuristic acceleration
- Bidirectional search
- Shortcut edge preprocessing
- Efficient adjacency-list graph representation
- Heuristic route diversification

---

# Sample Functionalities

- Find shortest routes between locations
- Handle road closures dynamically
- Generate alternate navigation routes
- Find nearest hospitals/restaurants
- Accelerate routing queries using preprocessing
- Optimize delivery scheduling for multiple drivers

---

# Complexity Overview

| Operation | Complexity |
|---|---|
| A* Shortest Path | O(E log V) |
| KNN Query | O(V log k) |
| Yen’s K Shortest Paths | O(kV(E+V)logV) |
| Approximate Queries | Substantially faster after preprocessing |

---

# Testcases

The repository includes:
- Sample graph inputs
- Query JSON files
- Output JSON files
- Python testcase generation scripts

---

# Future Improvements

- OpenStreetMap integration
- Real-time traffic APIs
- Turn penalties
- Machine learning-based ETA prediction
- Multi-agent fleet optimization
- GPU-accelerated preprocessing

---

# Inspiration

Inspired by large-scale routing and delivery systems such as:
- Google Maps
- Zomato
- Swiggy
- Blinkit
- Uber

---

# Technologies Used

- C++
- STL
- Graph Algorithms
- Heuristic Optimization
- JSON Parsing
- Python (for testcase generation)

---

# Authors

Developed as part of a graph-based routing and optimization systems project.
