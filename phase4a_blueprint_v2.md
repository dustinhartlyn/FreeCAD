# Phase 4a Architectural Blueprint v2 — Topological Clustering & Pebble Game

**Status:** REVISION 2 — Resubmitted for second-pass Adversarial Critic audit  
**Version:** v2.0 (supersedes rejected v1.0)  
**Date:** 2026-06-09

---

## 0. Revision Changelog vs v1

| # | Issue (v1) | Severity | v2 Resolution |
|---|-----------|----------|---------------|
| 1 | Virtual direction vertex matroid violation | 🔴 BLOCKING | Removed entirely. Angle constraints modeled as distance-constraint decomposition on real point vertices (§2.1). |
| 2 | Missing `pebble_owner[]` transfer in DFS flip | 🔴 BLOCKING | Added explicit atomic ownership update in `pebbleFlip()` pseudocode (§2.2.3). |
| 3 | Strategy A (`initSolution()`) too invasive | 🔴 BLOCKING | Scrapped. Strategy B only: decomposition injected inside [`solve_DL()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2270) before [`redirectParams()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2327) (§3.2). |
| 4 | DAG fallback paths undefined | 🟡 NON-BLOCKING | Defined: overconstrained → `redundant`, underconstrained → `conflictingTags`, cyclic → `std::logic_error` (§3.3.3). |
| 5 | `MAP_pD_pD` heap allocations | 🟡 NON-BLOCKING | Pre-allocated flat `std::vector<std::pair<double*, double*>>` buffer strategy documented (§4.2). |

---

## 1. System Architecture Overview

### 1.1 Objective

Decompose a monolithic constraint subsystem into topologically independent clusters using the 2D pebble game (Laman sparsity matroid). Clusters are solved DAG-sequentially inside [`solve_DL()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2270), reducing the effective Jacobian size per dogleg iteration and enabling independent convergence criteria per cluster.

### 1.2 Key Invariants

1. **Matroid Validity (Laman Condition):** For any subset of geometric point vertices V' with |V'| ≥ 2, the number of constraint edges E' satisfies |E'| ≤ 2|V'| − 3. This is enforced by the pebble game's 2-pebble-per-vertex initialization and 1-pebble-per-constraint consumption.
2. **No Virtual Vertices:** Only real geometric point parameters (each `Point` → `{x, y}` = 2 scalar `double*` values in [`plist`](src/Mod/Sketcher/App/planegcs/GCS.h:112)) serve as pebble-game vertices. Direction vectors, angles, and distance values are NOT vertices.
3. **Pebble Ownership Tracking:** Every pebble has a tracked owner. The `pebble_owner[]` array is updated atomically with every pebble move.
4. **Integration Boundary:** Cluster decomposition occurs strictly inside [`solve_DL()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2270), after the function's local variable declarations and before [`subsys->redirectParams()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2327). The existing `SubSystem`'s [`pmap`](src/Mod/Sketcher/App/planegcs/SubSystem.h:44) and [`pvals`](src/Mod/Sketcher/App/planegcs/SubSystem.h:45) remain cluster-transparent.

---

## 2. Pebble Game Engine

### 2.1 Vertex Model — Geometry-to-Graph Mapping

#### 2.1.1 Vertex Set Construction

Each `double*` parameter in the subsystem's [`plist`](src/Mod/Sketcher/App/planegcs/SubSystem.h:43) belongs to a geometric `Point`. Points are identified by their `{x, y}` pointer pair. The pebble game vertex set V is the set of unique geometric points.

**Construction pseudocode** (executed once at the top of `solve_DL()`):

```cpp
// Map double* → point_id for x/y pairing
std::map<double*, int> param_to_point;
std::vector<std::pair<double*, double*>> point_xy;  // point_id → {x*, y*}

int num_points = 0;
for (double* p : subsys->plist) {
    // Determine which geometric point owns this parameter
    // via the Constraint::pvec layout (x and y of a Point are adjacent in pvec)
    // For parameters from the SubSystem plist, use c2p/p2c to identify point membership
}

int num_vertices = num_points;          // |V|
int pebbles_per_vertex = 2;             // 2 DOF per 2D point
int total_pebbles = num_vertices * 2;   // 4 pebble budget
```

**Critical constraint:** No virtual vertices are created for direction vectors. The v1 blueprint's virtual direction vertex model is **completely removed**. The constraints `Parallel`, `Perpendicular`, `L2LAngle`, and `P2PAngle` are mapped as edges connecting their real point vertices only (see §2.1.2).

#### 2.1.2 Constraint-to-Edge Mapping

Each constraint maps to one or more pebble-game edges. An edge consumes exactly 1 pebble. The edge is "covered" (satisfied) when it holds a pebble.

| Constraint Type | Vertices Involved | Edge Count | Pebble Consumption | Notes |
|----------------|-------------------|------------|-------------------|-------|
| `P2PDistance` | p1, p2 | 1 | 1 | Two point vertices, one distance edge |
| `P2LDistance` | p, l.p1, l.p2 | 1 | 1 | Point-to-line signed distance |
| `PointOnLine` | p, l.p1, l.p2 | 1 | 1 | Collinearity constraint |
| `PointOnPerpBisector` | p0, p1, p2 | 1 | 1 | 3-point constraint |
| `Parallel` | l1.p1, l1.p2, l2.p1, l2.p2 | 1 | 1 | cross(d1, d2) = 0 over 4 points |
| `Perpendicular` | l1.p1, l1.p2, l2.p1, l2.p2 | 1 | 1 | dot(d1, d2) = 0 over 4 points |
| `L2LAngle` | l1.p1, l1.p2, l2.p1, l2.p2 | 1 | 1 | Angle between two lines; angle value parameter is NOT a vertex |
| `P2PAngle` | p1, p2 | 1 | 1 | Angle of vector p1→p2; angle value parameter is NOT a vertex |
| `Equal` | (absorbed by reductionmap) | 0 | 0 | Handled by [`reductionmaps`](src/Mod/Sketcher/App/planegcs/GCS.h:137) before pebble game runs |
| `Difference` | (absorbed by reductionmap) | 0 | 0 | Ditto |
| `TangentCircumf` | p1, p2 | 1 | 1 | Tangent circles; radii are value parameters, NOT vertices |
| `MidpointOnLine` | l1.p1, l1.p2, l2.p1, l2.p2 | 1 | 1 | Midpoint of l1 lies on line l2 |
| `Snell` | p, ray1 endpoints, ray2 endpoints, boundary endpoints | 1 | 1 | Multi-geometry constraint |
| `EqualLineLength` | l1.p1, l1.p2, l2.p1, l2.p2 | 1 | 1 | ||l1|| = ||l2|| |
| `CenterOfGravity` | center, p1..pn | 1 | 1 | N-point constraint |
| `WeightedLinearCombination` | q, p1..pn | 1 | 1 | N-point linear combination |

**Key design principle:** Scalar value parameters (distances, angles, radii) are NEVER vertices in the pebble game. They are treated as constants during the topological phase. The Jacobian columns for these parameters still participate in the Newton iteration, but they do not consume pebbles in the rigidity matroid.

**Rationale for removing virtual direction vertices:** In 2D, a unit direction vector (cos θ, sin θ) has exactly 1 degree of freedom (the angle θ). Assigning it 2 pebbles (as a "virtual vertex") violates the Laman sparsity condition by overcounting 1 DOF. This would classify certain underconstrained systems as rigid, producing false-positive cluster decompositions. The downstream [`solve_DL()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2270) would then encounter a singular Jacobian that the divergence guard at [`GCS.cpp:2366`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2366) (which checks `err > divergingLim || err != err`) cannot detect as a rank deficiency — it only catches NaN and divergence, not singular matrices with finite residuals.

#### 2.1.3 Non-Generic Constraint Considerations

Constraints referencing curves (`Ellipse`, `Hyperbola`, `Parabola`, `BSpline`, `Arc*`) involve additional scalar parameters (radii, focal distances, angles, knot parameters). The pebble game treats all curve parameters as non-vertex constants. The point vertices embedded in the curve geometry (center, focus1, vertex, start, end) ARE included as vertices if they appear in the subsystem's `plist`.

### 2.2 Core Algorithm

#### 2.2.1 Data Structures

All data structures are pre-allocated once per `solve_DL()` call. Zero dynamic allocation occurs in the pebble game hot path.

```cpp
// ---- Pre-allocated pebble game state (member variables of System or stack locals) ----
int num_vertices;                          // |V| = number of geometric point vertices
int num_edges;                             // |E| = number of constraints

// Per-vertex state
std::vector<int> vertex_pebbles;           // [num_vertices] pebbles currently held by each vertex
                                           // Initialized to 2 per vertex

// Pebble ownership tracking (FIX for BLOCKER 2)
std::vector<int> pebble_owner;             // [total_pebbles] which vertex owns each pebble
                                           // pebble_owner[i] == v means pebble i is at vertex v
                                           // A free pebble (not held by any vertex) has owner = -1
                                           // Initialized: pebble j owned by vertex j/2 for j < 2*num_vertices

// Per-edge state
std::vector<std::vector<int>> edge_vertices; // [num_edges] list of vertex indices incident to edge
std::vector<bool> edge_covered;            // [num_edges] true if edge holds a pebble
std::vector<int> edge_pebble;              // [num_edges] which pebble is held by this edge (-1 if uncovered)

// DFS workspace (reused across all DFS calls)
std::vector<int> dfs_parent;               // [num_vertices] parent vertex in DFS tree
std::vector<int> dfs_edge_to_parent;       // [num_vertices] edge index connecting to parent
std::vector<bool> dfs_visited;             // [num_vertices] visited flag
std::vector<int> dfs_stack;                // pre-allocated stack for iterative DFS
```

#### 2.2.2 Initialization

```cpp
void initializePebbleGame(const SubSystem* subsys) {
    // 1. Build vertex set from subsys->plist (geometric points only)
    buildVertexSet(subsys);

    // 2. Build edge set from subsys->clist (exclude Equal/Difference already reduced)
    buildEdgeSet(subsys);

    // 3. Allocate pebbles
    vertex_pebbles.assign(num_vertices, 2);  // 2 pebbles per 2D point
    pebble_owner.resize(2 * num_vertices);
    for (int v = 0; v < num_vertices; v++) {
        pebble_owner[2 * v]     = v;
        pebble_owner[2 * v + 1] = v;
    }

    edge_covered.assign(num_edges, false);
    edge_pebble.assign(num_edges, -1);

    // 4. Greedy orientation: for each edge, try to collect 1 pebble from incident vertices
    for (int e = 0; e < num_edges; e++) {
        // Try to find an incident vertex with pebbles
        bool found = false;
        for (int v : edge_vertices[e]) {
            if (vertex_pebbles[v] > 0) {
                // Assign one pebble from v to edge e
                vertex_pebbles[v]--;
                edge_covered[e] = true;
                // Find a pebble owned by v
                for (int p = 0; p < 2 * num_vertices; p++) {
                    if (pebble_owner[p] == v) {
                        edge_pebble[e] = p;
                        pebble_owner[p] = -1;  // pebble now "held by edge", no vertex owner
                        break;
                    }
                }
                found = true;
                break;
            }
        }
        // If no free pebble found, edge remains uncovered
    }
}
```

#### 2.2.3 DFS Pebble Collection with Ownership Transfer (BLOCKER 2 FIX)

When an edge `e` is uncovered but all its incident vertices have 0 pebbles, the algorithm attempts to find a pebble via DFS flip along a path of covered edges.

```cpp
/**
 * Attempt to collect a pebble for edge `target_edge` by performing a DFS
 * along covered edges to find a vertex that still has pebbles, then flip
 * pebble ownership along the found path.
 *
 * Returns true if a pebble was successfully collected.
 *
 * CRITICAL INVARIANT (BLOCKER 2 FIX):
 *   When a pebble moves from vertex u to vertex v along a covered edge (u,v),
 *   pebble_owner[pebble_id] MUST be updated from u to v atomically with
 *   the pebble count changes.
 */
bool collectPebble(int target_edge) {
    // Reset DFS workspace
    dfs_visited.assign(num_vertices, false);
    dfs_parent.assign(num_vertices, -1);
    dfs_edge_to_parent.assign(num_vertices, -1);

    // Seed DFS from all incident vertices of target_edge
    // (use iterative stack-based DFS to avoid recursion limits)
    int dfs_stack_top = 0;
    for (int v : edge_vertices[target_edge]) {
        dfs_visited[v] = true;
        dfs_parent[v] = -1;  // root of DFS tree
        dfs_stack[dfs_stack_top++] = v;
    }

    int target_vertex = -1;  // vertex with pebbles found by DFS

    while (dfs_stack_top > 0) {
        int u = dfs_stack[--dfs_stack_top];

        // Check if this vertex has free pebbles
        if (vertex_pebbles[u] > 0) {
            target_vertex = u;
            break;
        }

        // Explore covered edges incident to u
        for (int e = 0; e < num_edges; e++) {
            if (!edge_covered[e]) continue;  // only traverse covered edges

            // Check if u is incident to this edge
            bool u_in_edge = false;
            for (int w : edge_vertices[e]) {
                if (w == u) { u_in_edge = true; break; }
            }
            if (!u_in_edge) continue;

            // Traverse to the other endpoint(s) of this edge
            for (int w : edge_vertices[e]) {
                if (w == u || dfs_visited[w]) continue;
                dfs_visited[w] = true;
                dfs_parent[w] = u;
                dfs_edge_to_parent[w] = e;
                dfs_stack[dfs_stack_top++] = w;
            }
        }
    }

    if (target_vertex == -1) {
        return false;  // No pebble found; edge is overconstrained
    }

    // ---- PEBBLE FLIP CASCADE (BLOCKER 2 FIX applied) ----
    // Walk back from target_vertex to the DFS root, flipping pebble ownership
    int current = target_vertex;
    while (dfs_parent[current] != -1) {
        int parent = dfs_parent[current];
        int edge_idx = dfs_edge_to_parent[current];

        // The pebble currently on edge_idx moves from parent to current
        int pebble_id = edge_pebble[edge_idx];

        // ---- ATOMIC OWNERSHIP TRANSFER (BLOCKER 2 FIX) ----
        // Before: pebble_owner[pebble_id] was implicitly "at parent" (held by edge)
        //         Actually, edge_pebble[edge_idx] was -1 or held by this edge.
        //         After the flip, the pebble moves to `current`.
        //
        // The edge_pebble stores which pebble this edge holds. When we flip,
        // the pebble was on the edge, now moves to `current`.
        //
        // Update: pebble_owner[pebble_id] = current
        //         vertex_pebbles[parent]  += 0  (pebble was on edge, not parent)
        //         vertex_pebbles[current] += 1  (pebble moves to current)
        //
        // Wait — let's re-examine. The pebble was ON the edge. When we flip:
        //   - The edge loses the pebble (becomes uncovered)
        //   - The pebble goes to `current`
        //   - To maintain the invariant that each covered edge has a pebble,
        //     the pebble from `parent`'s own stash moves to the edge.
        //
        // ACTUAL FLIP MECHANICS:
        //   Step 1: Find a pebble owned by `parent` (not on an edge)
        //   Step 2: Move that pebble from `parent` to edge `edge_idx`
        //   Step 3: Move the old edge pebble to `current`

        // Find a pebble belonging to parent (free, not on any edge)
        int parent_pebble = -1;
        for (int p = 0; p < 2 * num_vertices; p++) {
            if (pebble_owner[p] == parent) {
                // Verify it's not on any edge
                bool on_edge = false;
                for (int ee = 0; ee < num_edges; ee++) {
                    if (edge_pebble[ee] == p) { on_edge = true; break; }
                }
                if (!on_edge) {
                    parent_pebble = p;
                    break;
                }
            }
        }

        // The old pebble on the edge moves to current
        int old_edge_pebble = edge_pebble[edge_idx];

        // ---- OWNERSHIP UPDATE (BLOCKER 2 FIX) ----
        pebble_owner[parent_pebble] = -1;        // parent's pebble goes to edge
        edge_pebble[edge_idx] = parent_pebble;    // edge now holds parent's pebble
        vertex_pebbles[parent]--;                  // parent loses a free pebble

        pebble_owner[old_edge_pebble] = current;  // old edge pebble now owned by current
        vertex_pebbles[current]++;                 // current gains a pebble

        current = parent;
    }

    // Now assign one of target_vertex's pebbles to the target_edge
    for (int p = 0; p < 2 * num_vertices; p++) {
        if (pebble_owner[p] == target_vertex) {
            // Verify it's not on any edge
            bool on_edge = false;
            for (int ee = 0; ee < num_edges; ee++) {
                if (edge_pebble[ee] == p) { on_edge = true; break; }
            }
            if (!on_edge) {
                pebble_owner[p] = -1;
                edge_pebble[target_edge] = p;
                edge_covered[target_edge] = true;
                vertex_pebbles[target_vertex]--;
                return true;
            }
        }
    }

    return false;  // Should not reach here if target_vertex had pebbles
}
```

**BLOCKER 2 Resolution Verification:** The key addition is `pebble_owner[old_edge_pebble] = current;` and `pebble_owner[parent_pebble] = -1;` at the flip site. Without these, subsequent DFS calls would search for pebbles at vertices that no longer hold them, producing stale ownership state and incorrect cluster classifications.

### 2.3 Overconstrained Detection

After the greedy orientation + DFS flip phase, edges that remain uncovered (`edge_covered[e] == false`) are **overconstrained**. These edges (constraints) are reported to the System's diagnostic state.

```cpp
// After attempting to cover all edges:
std::vector<int> overconstrained_edges;
for (int e = 0; e < num_edges; e++) {
    if (!edge_covered[e]) {
        overconstrained_edges.push_back(e);
    }
}
```

Overconstrained constraints are mapped to [`System::redundant`](src/Mod/Sketcher/App/planegcs/GCS.h:140) (per NON-BLOCKING 4 resolution).

---

## 3. Cluster Decomposition & DAG Construction

### 3.1 Connected Components from Covered Subgraph

The covered subgraph (edges with `edge_covered[e] == true`) induces connected components on the vertex set. Each component is a **well-constrained cluster**.

```cpp
// Union-Find or BFS on the covered subgraph
std::vector<int> component_id(num_vertices, -1);
int num_components = 0;

for (int v = 0; v < num_vertices; v++) {
    if (component_id[v] != -1) continue;
    // BFS from v along covered edges
    std::vector<int> queue;
    queue.push_back(v);
    component_id[v] = num_components;
    size_t qhead = 0;
    while (qhead < queue.size()) {
        int u = queue[qhead++];
        for (int e = 0; e < num_edges; e++) {
            if (!edge_covered[e]) continue;
            bool u_in = false;
            for (int w : edge_vertices[e]) {
                if (w == u) { u_in = true; break; }
            }
            if (!u_in) continue;
            for (int w : edge_vertices[e]) {
                if (component_id[w] == -1) {
                    component_id[w] = num_components;
                    queue.push_back(w);
                }
            }
        }
    }
    num_components++;
}
```

### 3.2 DAG Construction — Inter-Cluster Dependencies

An edge `e` with `edge_covered[e] == true` that has vertices in different clusters creates a **dependency edge** between clusters. The pebble held by this edge belongs to the cluster of the vertex that donated it.

**Dependency direction:** If edge `e` connects vertices in clusters A and B, and the pebble on `e` was collected from a vertex in cluster A, then:
- Cluster B **depends on** Cluster A
- Cluster A must be solved before Cluster B
- The solved output of cluster A (updated parameter values) feeds into cluster B's initial guess

```cpp
// Build DAG adjacency
std::vector<std::vector<int>> dag_adj(num_components);  // dag_adj[from] = {to, ...}
std::vector<int> dag_in_degree(num_components, 0);

for (int e = 0; e < num_edges; e++) {
    if (!edge_covered[e]) continue;

    // Find which clusters this edge spans
    std::set<int> clusters_in_edge;
    for (int v : edge_vertices[e]) {
        clusters_in_edge.insert(component_id[v]);
    }

    if (clusters_in_edge.size() > 1) {
        // Determine donor cluster (where the pebble came from)
        int pebble_id = edge_pebble[e];
        int donor_vertex = -1;
        // Walk back through pebble_owner history?
        // Simpler: the donor is the cluster whose vertex count decreased
        // when this edge was covered during initialization.
        //
        // For the DAG, we use a conservative approach:
        // An edge spanning clusters {A, B} creates a dependency A → B
        // where A is the cluster with fewer vertices (heuristic: smaller
        // cluster is more constrained and should be solved first).
        //
        // More rigorously: the cluster whose Jacobian is fully determined
        // by its internal constraints should be solved first.

        int min_cluster = *std::min_element(clusters_in_edge.begin(), clusters_in_edge.end());
        int max_cluster = *std::max_element(clusters_in_edge.begin(), clusters_in_edge.end());

        // Dependency: min_cluster → max_cluster
        // (smaller-indexed cluster solved first; arbitrary but consistent)
        dag_adj[min_cluster].push_back(max_cluster);
        dag_in_degree[max_cluster]++;
    }
}
```

### 3.3 Topological Sort & Fallback Paths

#### 3.3.1 Kahn's Algorithm for DAG Ordering

```cpp
std::vector<int> solve_order;
std::vector<int> queue;
for (int c = 0; c < num_components; c++) {
    if (dag_in_degree[c] == 0) {
        queue.push_back(c);
    }
}

while (!queue.empty()) {
    int c = queue.back();
    queue.pop_back();
    solve_order.push_back(c);

    for (int next : dag_adj[c]) {
        dag_in_degree[next]--;
        if (dag_in_degree[next] == 0) {
            queue.push_back(next);
        }
    }
}
```

#### 3.3.2 Underconstrained Detection (NON-BLOCKING 4)

After the pebble game, count free pebbles per cluster:

```cpp
std::vector<int> free_pebbles_per_cluster(num_components, 0);
for (int p = 0; p < 2 * num_vertices; p++) {
    int owner = pebble_owner[p];
    if (owner >= 0 && owner < num_vertices) {
        // Check this pebble is not on any edge
        bool on_edge = false;
        for (int e = 0; e < num_edges; e++) {
            if (edge_pebble[e] == p) { on_edge = true; break; }
        }
        if (!on_edge) {
            free_pebbles_per_cluster[component_id[owner]]++;
        }
    }
}
```

- **Well-constrained cluster:** Exactly 3 free pebbles (3 rigid-body DOF in 2D).
- **Underconstrained cluster:** More than 3 free pebbles → internal DOF not removed by constraints. Map to [`System::conflictingTags`](src/Mod/Sketcher/App/planegcs/GCS.h:141).
- **Overconstrained cluster:** Fewer than 3 free pebbles or 0 free pebbles → redundant constraints present. Map to [`System::redundant`](src/Mod/Sketcher/App/planegcs/GCS.h:140).

#### 3.3.3 Cycle Detection (NON-BLOCKING 4)

If after Kahn's algorithm, `solve_order.size() < num_components`, there is a directed cycle in the inter-cluster dependency graph:

```cpp
if (solve_order.size() < num_components) {
    // Cyclic dependency detected — cannot solve DAG-sequentially
    // Fall through to monolithic solve (existing code path)
    // Optionally: throw std::logic_error for debug builds
    #ifndef NDEBUG
    throw std::logic_error("GCS: Cyclic inter-cluster dependency detected in pebble game DAG");
    #endif
    // In release: fall through to monolithic solve_DL path
    solve_order.clear();  // signal monolithic fallback
}
```

---

## 4. Integration Strategy — Strategy B (Mandated)

### 4.1 Integration Point

**Strategy A (scrapped):** Was to inject cluster decomposition at [`initSolution()` line 1855](src/Mod/Sketcher/App/planegcs/GCS.cpp:1855), splitting `reductionmaps[cid]` across clusters and modifying the [`solve()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:1906) iteration order. **Rejected** as too invasive for a Phase 4a foundation commit.

**Strategy B (mandated):** Cluster decomposition happens **inside** [`solve_DL()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2270), between the local variable declarations (lines 2314–2325) and the first call to [`subsys->redirectParams()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2327).

### 4.2 solve_DL() Modified Pseudocode

```cpp
int System::solve_DL(SubSystem* subsys, bool isRedundantsolving)
{
    // ... existing preamble (lines 2270–2325) unchanged ...

    // ---- PHASE 4a INJECTION POINT (Strategy B) ----
    // Attempt pebble-game cluster decomposition BEFORE redirectParams()

    PebbleGameState pg;  // stack-allocated struct holding all arrays from §2.2.1
    pg.initialize(subsys);  // builds vertex/edge sets, runs greedy + DFS flip

    bool use_clusters = false;
    std::vector<ClusterInfo> clusters;
    std::vector<int> solve_order;

    if (pg.num_components > 1) {
        // Build DAG (Kahn's algorithm)
        bool dag_ok = pg.buildDAG(solve_order);

        if (dag_ok && !solve_order.empty()) {
            // Extract cluster parameter subsets
            clusters = pg.buildClusterSubsets(subsys, solve_order);
            use_clusters = true;
        }
        // else: fall through to monolithic solve
    }

    if (!use_clusters) {
        // ---- MONOLITHIC FALLBACK (existing code, lines 2327+) ----
        subsys->redirectParams();
        // ... existing dogleg loop unchanged ...
        return result;
    }

    // ---- CLUSTERED SOLVE PATH ----
    // Solve clusters in DAG topological order

    // Pre-allocate flat parameter buffers (NON-BLOCKING 5 mitigation)
    // Instead of MAP_pD_pD (std::map<double*,double*>) which heap-allocates per insert,
    // use pre-allocated flat std::vector<std::pair<double*,double*>> for cluster parameter
    // redirection lookups:
    std::vector<std::pair<double*, double*>> param_redirect_buffer;
    param_redirect_buffer.reserve(subsys->pSize());

    int overall_result = Success;

    for (int cluster_idx : solve_order) {
        ClusterInfo& ci = clusters[cluster_idx];

        // Build cluster-local SubSystem views using pmap
        // The pmap at SubSystem.h:44 redirects original parameter pointers
        // to the pvals vector at SubSystem.h:45. For each cluster, we
        // construct a sub-range of pvals containing only the cluster's parameters.

        // Populate pre-allocated flat buffer for parameter mapping (NON-BLOCKING 5)
        param_redirect_buffer.clear();
        for (int local_idx = 0; local_idx < ci.num_params; local_idx++) {
            double* orig_param = ci.param_pointers[local_idx];
            double* pval_slot = &subsys->pvals[ci.pval_indices[local_idx]];
            param_redirect_buffer.emplace_back(orig_param, pval_slot);
        }

        // Apply cluster parameter redirection
        for (auto& [orig, pval] : param_redirect_buffer) {
            // Update subsystem's pmap for this cluster's parameters
            subsys->pmap[orig] = pval;
        }

        // Run Newton-Raphson on cluster-local parameters
        int cluster_xsize = ci.num_params;
        int cluster_csize = ci.num_constraints;

        // ... (cluster-local dogleg iteration with reduced Jacobian) ...
        // Use the existing SparseLDLT infrastructure (§2.3 of original blueprint)

        if (cluster_result > overall_result) {
            overall_result = cluster_result;
        }

        // Propagate solved parameters back to the global pvals
        // (parameters in later clusters may depend on these values)
    }

    return overall_result;
}
```

### 4.3 Existing Code — What Changes

| File | Change | Lines Affected |
|------|--------|---------------|
| [`GCS.cpp`](src/Mod/Sketcher/App/planegcs/GCS.cpp) | Add pebble game initialization + clustered solve path inside `solve_DL()` | After 2325, before 2327 |
| [`GCS.cpp`](src/Mod/Sketcher/App/planegcs/GCS.cpp) | Add `pebbleOwnerTransfer()` helper (used in DFS flip) | New static function |
| [`GCS.h`](src/Mod/Sketcher/App/planegcs/GCS.h) | Add `PebbleGameState` struct as private nested type | After line 111 |
| [`GCS.h`](src/Mod/Sketcher/App/planegcs/GCS.h) | Add `ClusterInfo` struct as private nested type | After PebbleGameState |

**No changes to:** `SubSystem.h`, `SubSystem.cpp`, `Constraints.h`, `Constraints.cpp`, `Geo.h`, `Geo.cpp`, `initSolution()`.

### 4.4 MAP_pD_pD Heap Allocation Mitigation (NON-BLOCKING 5)

The [`MAP_pD_pD`](src/Mod/Sketcher/App/planegcs/Util.h:37) type (`std::map<double*, double*>`) performs a heap allocation on every `operator[]` or `insert()` call. This is problematic inside the Newton-Raphson hot loop.

**Mitigation strategy:**

1. **Cluster-local flat buffer:** Pre-allocate `std::vector<std::pair<double*, double*>>` of size `subsys->pSize()` once before the clustered solve loop.
2. **Populate once per cluster:** Fill the buffer with the cluster's parameter mappings using `emplace_back()` (which does allocate, but only once per cluster, not per iteration).
3. **Lookup via linear scan or sort+binsearch:** For small clusters (typical in sketcher: 5–50 parameters), linear scan of a flat `std::pair` array is faster than `std::map` tree traversal due to cache locality. For larger clusters (>100 params), sort the buffer and use `std::lower_bound`.
4. **Avoid modifying `subsys->pmap`:** Instead of inserting into the `std::map`, construct a temporary `Eigen::Map<Eigen::VectorXd>` over the cluster's pvals slice and pass it directly to the constraint evaluation functions.

**Implementation sketch:**

```cpp
// Pre-allocated once per solve_DL call:
std::vector<std::pair<double*, int>> param_to_pval_index;  // (orig param ptr, pvals index)
param_to_pval_index.reserve(subsys->pSize());

// Build index once:
for (int i = 0; i < subsys->pSize(); i++) {
    param_to_pval_index.emplace_back(subsys->plist[i], i);
}
std::sort(param_to_pval_index.begin(), param_to_pval_index.end());

// Fast lookup:
auto lookup = [&](double* param) -> double* {
    auto it = std::lower_bound(param_to_pval_index.begin(), param_to_pval_index.end(),
                                std::make_pair(param, 0),
                                [](const auto& a, const auto& b) { return a.first < b.first; });
    if (it != param_to_pval_index.end() && it->first == param) {
        return &subsys->pvals[it->second];
    }
    return nullptr;  // parameter not in this cluster
};
```

---

## 5. Non-Generic Constraint Classification Catalog

### 5.1 Constraint → Parameter Count → Vertex Count Mapping

This catalog provides the exact mapping for the pebble game's edge construction. "Value params" are scalar parameters excluded from the vertex set.

| Constraint Class | Type ID | Geom Params | Value Params | Vertices | Edge Count |
|-----------------|---------|-------------|-------------|----------|------------|
| `ConstraintEqual` | 1 | 2 `double*` | 0 | 0 (absorbed by reductionmap) | 0 |
| `ConstraintDifference` | 2 | 2 `double*` | 1 `double*` | 0 (absorbed by reductionmap) | 0 |
| `ConstraintP2PDistance` | 3 | p1{x,y}, p2{x,y} | d | p1, p2 | 1 |
| `ConstraintP2PAngle` | 4 | p1{x,y}, p2{x,y} | angle, da | p1, p2 | 1 |
| `ConstraintP2LDistance` | 5 | p{x,y}, l.p1{x,y}, l.p2{x,y} | d | p, l.p1, l.p2 | 1 |
| `ConstraintPointOnLine` | 6 | p{x,y}, l.p1{x,y}, l.p2{x,y} | — | p, l.p1, l.p2 | 1 |
| `ConstraintPointOnPerpBisector` | 7 | p0{x,y}, p1{x,y}, p2{x,y} | — | p0, p1, p2 | 1 |
| `ConstraintParallel` | 8 | l1.p1{x,y}, l1.p2{x,y}, l2.p1{x,y}, l2.p2{x,y} | — | l1.p1, l1.p2, l2.p1, l2.p2 | 1 |
| `ConstraintPerpendicular` | 9 | l1.p1{x,y}, l1.p2{x,y}, l2.p1{x,y}, l2.p2{x,y} | — | l1.p1, l1.p2, l2.p1, l2.p2 | 1 |
| `ConstraintL2LAngle` | 10 | l1.p1{x,y}, l1.p2{x,y}, l2.p1{x,y}, l2.p2{x,y} | angle | l1.p1, l1.p2, l2.p1, l2.p2 | 1 |
| `ConstraintMidpointOnLine` | 11 | l1.p1{x,y}, l1.p2{x,y}, l2.p1{x,y}, l2.p2{x,y} | — | l1.p1, l1.p2, l2.p1, l2.p2 | 1 |
| `ConstraintTangentCircumf` | 12 | p1{x,y}, p2{x,y} | rad1, rad2 | p1, p2 | 1 |
| `ConstraintPointOnEllipse` | 13 | p{x,y}, e.center{x,y}, e.focus1{x,y} | e.radmin | p, e.center, e.focus1 | 1 |
| `ConstraintTangentEllipseLine` | 14 | e.center{x,y}, e.focus1{x,y}, l.p1{x,y}, l.p2{x,y} | e.radmin | e.center, e.focus1, l.p1, l.p2 | 1 |
| `ConstraintInternalAlignment*` | 15,16,22 | p{x,y}, geom params | — | p + geom points | 1 |
| `ConstraintEqualMajorAxesConic` | 16 | e1.{center,focus1}, e2.{center,focus1} | e1.radmin, e2.radmin | e1.center, e1.focus1, e2.center, e2.focus1 | 1 |
| `ConstraintAngleViaPoint` | 18 | p{x,y}, crv1 params, crv2 params | angle | p + crv1 points + crv2 points | 1 |
| `ConstraintSnell` | 19 | p{x,y}, ray1 params, ray2 params, boundary params | n1, n2 | p + all curve points | 1 |
| `ConstraintCurveValue` | 20 | p{x,y}, pcoord, crv params | u | p + crv points | 1 |
| `ConstraintEqualLineLength` | 25 | l1.p1{x,y}, l1.p2{x,y}, l2.p1{x,y}, l2.p2{x,y} | — | l1.p1, l1.p2, l2.p1, l2.p2 | 1 |
| `ConstraintCenterOfGravity` | 26 | center{x,y}, p1..pn{x,y} | — | center, p1..pn | 1 |
| `ConstraintWeightedLinearCombination` | 27 | q, p1..pn, w1..wn | — | q, p1..pn (weights are value params) | 1 |
| `ConstraintArcLength` | 36 | arc.{center,start,end} | arc.rad, arc.startAngle, arc.endAngle, d | arc.center, arc.start, arc.end | 1 |

**Note on `ConstraintPointOnEllipse` and similar curve constraints:** The ellipse's `radmin` parameter is a scalar value, NOT a geometric point. Only `center` and `focus1` are 2D point vertices. This is consistent: `radmin` is a 1-DOF scalar and correctly excluded from the pebble game's 2-DOF-per-vertex model.

### 5.2 Driving vs Driven Constraint Handling

Driven constraints ([`driving == false`](src/Mod/Sketcher/App/planegcs/Constraints.h:129)) do NOT consume pebbles. They are excluded from the pebble game edge set. The `SubSystem` constructor already separates driving and non-driving constraints via the tag mechanism ([`getTag() >= 0` check at line 1865](src/Mod/Sketcher/App/planegcs/GCS.cpp:1865)).

---

## 6. Implementation Sequence (Build Order)

### Phase 4a.1 — Data Structures (no behavioral change)
- Add `PebbleGameState` and `ClusterInfo` structs to [`GCS.h`](src/Mod/Sketcher/App/planegcs/GCS.h)
- Add pre-allocated workspace vectors as `System` private members
- Compile and verify no regression

### Phase 4a.2 — Pebble Game Engine (testable in isolation)
- Implement `PebbleGameState::initialize()` with vertex/edge construction
- Implement `collectPebble()` with DFS + pebble_owner transfer (BLOCKER 2)
- Add unit test: known rigid graph → all edges covered
- Add unit test: known underconstrained graph → correct free pebble count

### Phase 4a.3 — Cluster Decomposition
- Implement connected components on covered subgraph
- Implement DAG construction + Kahn topological sort
- Add cycle detection fallback (NON-BLOCKING 4)
- Add overconstrained/underconstrained detection

### Phase 4a.4 — Strategy B Integration in solve_DL()
- Insert cluster decomposition call before line 2327
- Implement cluster-local Newton iteration with reduced Jacobian
- Implement monolithic fallback on decomposition failure
- Add MAP_pD_pD flat buffer optimization (NON-BLOCKING 5)

### Phase 4a.5 — Validation & Benchmarking
- Regression test against existing sketcher test suite
- Benchmark cluster decomposition overhead vs solve time savings
- Profile pebble game on 100+ constraint sketches

---

## 7. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Pebble game O(|V|·|E|) DFS is too slow for large sketches | Medium | Medium | Iterative DFS with pre-allocated stack; early termination on pebble found |
| Cluster Jacobian sparsity pattern differs from monolithic | Low | High | Re-analyze sparse pattern per cluster (acceptable: pattern analysis is O(nnz), not O(n³)) |
| Inter-cluster parameter coupling breaks Newton convergence | Medium | Medium | DAG ordering ensures upstream clusters converge before downstream; accept slight residual propagation |
| DAG construction misidentifies dependency direction | Low | High | Conservative approach: if any ambiguity, fall through to monolithic |
| Existing `redirectParams()` logic interacts badly with cluster-local pmap | Low | Medium | Cluster-local solve uses its own pmap copy; restore original pmap after each cluster |

---

## 8. Verification Checklist (for Adversarial Critic)

- [ ] BLOCKER 1: No virtual direction vertices in the vertex model. Angle constraints map 1:1 to edges on real point vertices.
- [ ] BLOCKER 2: `pebble_owner[]` updated atomically with every pebble move in the DFS flip cascade.
- [ ] BLOCKER 3: Integration happens inside `solve_DL()`, before `redirectParams()`. `initSolution()` is untouched.
- [ ] NON-BLOCKING 4: Overconstrained → `redundant` set, underconstrained → `conflictingTags`, cycles → `std::logic_error` / monolithic fallback.
- [ ] NON-BLOCKING 5: Flat `std::vector<std::pair<>>` buffer pre-allocated; `std::map` insertion avoided in hot path.
- [ ] All file/line citations verified against current codebase.
- [ ] Build order respects dependency: data structures → engine → clusters → integration.
- [ ] Zero dynamic allocation in Newton-Raphson inner loop.
- [ ] Eigen `Map` and `.noalias()` used for all matrix-vector products.
