import osmnx as ox
import networkx as nx
import json
import random
import collections

# --- CONFIGURATION ---

# 1. Location & Size
# Bandra West, Mumbai coordinates
# CENTER_POINT = (19.0596, 72.8295) 
# INITIAL_RADIUS = 50000

# SIngapore
# CENTER_POINT = (1.290270, 103.851959)   
# INITIAL_RADIUS = 45000

#Newyork
CENTER_POINT = (40.7128, -74.0060)
INITIAL_RADIUS = 4000

TARGET_NODE_COUNT = 10000

# 2. Query Counts
NUM_K_SHORTEST_PATHS = 0
NUM_K_SHORTEST_HEURISTIC = 200
NUM_APPROX_BATCHES = 200
BATCH_SIZE_APPROX = 50

# 3. Constants
POIS = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
DEFAULT_SPEEDS = {
    "motorway": 80, "trunk": 60, "primary": 50, 
    "secondary": 40, "tertiary": 30, "residential": 20, 
    "unclassified": 20
}

# --- HELPER FUNCTIONS ---

def limit_graph_size(G, target_count):
    """
    Reduces the graph G to exactly target_count nodes
    by keeping the cluster around the center/busiest node.
    """
    total_nodes = len(G.nodes)
    if total_nodes <= target_count:
        print(f"Graph is already small ({total_nodes} nodes). No trimming needed.")
        return G

    print(f"Trimming graph from {total_nodes} to {target_count} nodes...")

    # 1. Find a start node (highest degree)
    start_node = max(dict(G.degree()).items(), key=lambda x: x[1])[0]

    # 2. BFS Traversal to ensure we keep a connected chunk
    kept_nodes = set()
    kept_nodes.add(start_node)
    queue = collections.deque([start_node])

    while len(kept_nodes) < target_count and queue:
        u = queue.popleft()
        
        # Successors
        for v in G.successors(u): 
            if v not in kept_nodes:
                kept_nodes.add(v)
                queue.append(v)
                if len(kept_nodes) >= target_count: break
        
        if len(kept_nodes) < target_count:
            # Predecessors (to ensure we don't get stuck in one-way traps)
             for v in G.predecessors(u):
                if v not in kept_nodes:
                    kept_nodes.add(v)
                    queue.append(v)
                    if len(kept_nodes) >= target_count: break

    # 3. Create Subgraph
    H = G.subgraph(kept_nodes).copy()
    
    # 4. Clean up: Ensure Strongly Connected Component
    # This guarantees that for every node u, v selected, path u->v exists (if graph structure allows)
    try:
        if hasattr(ox, 'truncate'):
             H = ox.truncate.largest_component(H, strongly=True)
        else:
             H = ox.utils_graph.get_largest_component(H, strongly=True)
    except AttributeError:
        pass
    
    print(f"Trimmed graph size: {len(H.nodes)} nodes, {len(H.edges)} edges.")
    return H

def get_speed(edge_data):
    maxspeed = edge_data.get("maxspeed")
    highway = edge_data.get("highway")
    if isinstance(maxspeed, list): maxspeed = maxspeed[0]
    if isinstance(highway, list): highway = highway[0]
    try:
        if maxspeed:
            speed_kmh = float(str(maxspeed).split()[0])
        else:
            speed_kmh = DEFAULT_SPEEDS.get(highway, 20)
    except (ValueError, AttributeError):
        speed_kmh = 20
    
    # Convert km/h to m/s
    return speed_kmh * (5.0 / 18.0)

def generate_queries(num_nodes, num_ksp, num_ksp_h, num_approx):
    print(f"Generating queries for {num_nodes} nodes...")
    events = []
    query_id_counter = 1
    
    def get_st():
        s = random.randint(0, num_nodes - 1)
        t = random.randint(0, num_nodes - 1)
        while s == t:
            t = random.randint(0, num_nodes - 1)
        return s, t

    # 1. K-Shortest Paths
    for _ in range(num_ksp):
        s, t = get_st()
        events.append({
            "type": "k_shortest_paths", "id": query_id_counter,
            "source": s, "target": t, "k": random.randint(3, 10), "mode": "distance"
        })
        query_id_counter += 1

    # 2. Heuristic K-SP
    for _ in range(num_ksp_h):
        s, t = get_st()
        events.append({
            "type": "k_shortest_paths_heuristic", "id": query_id_counter,
            "source": s, "target": t, "k": random.randint(3, 8), "overlap_threshold": random.randint(40, 80)
        })
        query_id_counter += 1

    # 3. Approx Batches
    for _ in range(num_approx):
        sub_queries = []
        for _ in range(BATCH_SIZE_APPROX):
            s, t = get_st()
            sub_queries.append({"source": s, "target": t})
        events.append({
            "type": "approx_shortest_path", "id": query_id_counter,
            "queries": sub_queries, "time_budget_ms": 100, 
            "acceptable_error_pct": random.choice([5.0, 10.0, 15.0])
        })
        query_id_counter += 1
        
    random.shuffle(events)
    return {"meta": {"id": "queries_limited"}, "events": events}

def main():
    print(f"1. Downloading map data around {CENTER_POINT}...")
    
    G = ox.graph_from_point(CENTER_POINT, dist=INITIAL_RADIUS, network_type="drive", simplify=True)
    if not isinstance(G, nx.MultiDiGraph): G = nx.MultiDiGraph(G)
    
    # 2. Limit Graph Size
    G = limit_graph_size(G, TARGET_NODE_COUNT)
    
    # 3. Re-index Nodes
    osm_id_to_index = {}
    nodes_list = []
    
    for idx, (osmid, data) in enumerate(G.nodes(data=True)):
        osm_id_to_index[osmid] = idx
        node_pois = random.sample(POIS, random.randint(0, 2)) if random.random() < 0.1 else []
        
        nodes_list.append({
            "id": idx,
            "lat": data.get("y", 0.0), "lon": data.get("x", 0.0),
            "pois": node_pois
        })

    # --- STEP 4: STRICT UNIQUE EDGE GENERATION ---
    # We explicitly generate every directed edge first, then deduplicate.
    # This resolves conflicts between 'bidirectional' attributes and explicit 'return' lanes.
    
    final_directed_edges = {} # Key: (u_index, v_index), Value: edge_data_dict
    
    print("Processing edges to ensure strict uniqueness...")

    for u, v, key, data in G.edges(keys=True, data=True):
        if u not in osm_id_to_index or v not in osm_id_to_index: 
            continue
        if u == v: 
            continue # Remove self-loops

        new_u = osm_id_to_index[u]
        new_v = osm_id_to_index[v]
        
        # 1. Analyze Directionality
        oneway_raw = data.get("oneway", False)
        if isinstance(oneway_raw, list): oneway_raw = oneway_raw[0]
        is_oneway = str(oneway_raw).lower() in ['true', 'yes', '1']

        # 2. Define the specific directed edges this OSM entry creates
        # If it's oneway, it creates u->v.
        # If it's bidirectional, it creates u->v AND v->u.
        pairs_to_add = [(new_u, new_v)]
        if not is_oneway:
            pairs_to_add.append((new_v, new_u))

        # 3. Add to map, keeping ONLY the shortest for that specific direction
        length = data.get("length", float('inf'))
        
        for (src, dst) in pairs_to_add:
            if (src, dst) not in final_directed_edges:
                final_directed_edges[(src, dst)] = data
            else:
                # COLLISION! We already have an edge src->dst.
                # Only keep this new one if it is strictly shorter.
                existing_len = final_directed_edges[(src, dst)].get("length", float('inf'))
                if length < existing_len:
                    final_directed_edges[(src, dst)] = data

    # --- Generate Final Edge List for JSON ---
    edges_list = []
    edge_id_counter = 1001

    for (u, v), data in final_directed_edges.items():
        
        length = data.get("length", 0.0)
        speed_ms = get_speed(data)
        highway = data.get("highway", "unclassified")
        if isinstance(highway, list): highway = highway[0]

        # IMPORTANT: We explicitly write EVERY edge as oneway=True 
        # because we have already manually handled the bidirectional split above.
        # The C++ parser should just read u->v and add a single directed edge.
        edges_list.append({
            "id": edge_id_counter,
            "u": u, "v": v,
            "length": round(length, 2),
            "average_time": round(length / speed_ms, 2) if speed_ms > 0 else 0,
            "oneway": True, 
            "road_type": str(highway)
        })
        edge_id_counter += 1

    # 5. Save Files
    graph_json = {
        "meta": {
            "id": "osm_test_strict_unique", 
            "nodes": len(nodes_list),
            "description": "Generated map with strict unique directed edges (no parallels)"
        },
        "nodes": nodes_list,
        "edges": edges_list
    }

    with open("graph.json", "w") as f:
        json.dump(graph_json, f, indent=2)
    print("Saved graph.json")

    queries_json = generate_queries(len(nodes_list), NUM_K_SHORTEST_PATHS, NUM_K_SHORTEST_HEURISTIC, NUM_APPROX_BATCHES)
    with open("queries.json", "w") as f:
        json.dump(queries_json, f, indent=2)
    print("Saved queries.json")
    print(f"\nDone! Generated {len(nodes_list)} nodes and {len(edges_list)} unique directed edges.")

if __name__ == "__main__":
    main()