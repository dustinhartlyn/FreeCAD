# Phase 4a Architectural Blueprint v3 — Topological Clustering & Pebble Game

**Status:** REVISION 3 — Resubmitted for third-pass Adversarial Critic audit  
**Version:** v3.0 (supersedes rejected v2.0)  
**Date:** 2026-06-09

---

## 0. Revision Changelog vs v2

| # | Issue (v2) | Severity | v3 Resolution |
|---|-----------|----------|---------------|
| 1 | §5.1 catalog incomplete (12+ types missing, ID 16 duplicated) | 🔴 BLOCKING | Complete catalog for all 37 constraint types (0–36). Removed duplicate. §2.1.2 synced to §5.1. |
| 2 | §2.2.3 cascade direction inverted: UB `pebble_owner[-1]` | 🔴 BLOCKING | Cascade reversed: `current` (leaf) donates pebble to edge, edge pebble flows to `parent`. Final orientation uses DFS root, not `target_vertex`. |
| 3 | §1.2/§4.2/§4.4 three-way contradiction on `pmap` | 🔴 BLOCKING | Resolved to cluster-transparent: `subsys->pmap` is NEVER modified. Cluster-local parameter mapping uses flat `std::vector<std::pair<>>` + lookup, not pmap writes. |
| 4 | §4.2 cluster-local dogleg unspecified | 🟡 NON-BLOCKING | Full specification: Jx/fx slicing rules, per-cluster convergence criteria, boundary parameter handling, Eigen block-operation strategy. |
| 5 | Hyperedge pebble game foundation unproven | 🟡 NON-BLOCKING | Cite Streinu & Theran (2009) generalized (k,l)-pebble game for hypergraphs. Document reduction strategy for known constraint topologies. Add regression test plan. |
| 6 | Wrong citation `GCS.cpp:1855` → should be `GCS.cpp:1739` | 🟡 NON-BLOCKING | Fixed: `initSolution()` correctly cited at [`GCS.cpp:1739`](src/Mod/Sketcher/App/planegcs/GCS.cpp:1739). |

### ✅ Carried Forward from v2 (Accepted, Not Reverted)
- Virtual direction vertex model REMOVED ✓
- Strategy A SCRAPPED, Strategy B MANDATED ✓
- DAG fallback paths DEFINED ✓
- All other citations VERIFIED ✓

---

## 1. System Architecture Overview

### 1.1 Objective

Decompose a monolithic constraint subsystem into topologically independent clusters using the 2D pebble game (Laman sparsity matroid). Clusters are solved DAG-sequentially inside [`solve_DL()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2270), reducing the effective Jacobian size per dogleg iteration and enabling independent convergence criteria per cluster.

### 1.2 Key Invariants

1. **Matroid Validity (Laman Condition):** For any subset of geometric point vertices V' with |V'| ≥ 2, the number of constraint edges E' satisfies |E'| ≤ 2|V'| − 3. This is enforced by the pebble game's 2-pebble-per-vertex initialization and 1-pebble-per-constraint consumption.

2. **No Virtual Vertices:** Only real geometric point parameters (each `Point` → `{x, y}` = 2 scalar `double*` values in [`plist`](src/Mod/Sketcher/App/planegcs/GCS.h:112)) serve as pebble-game vertices. Direction vectors, angles, and distance values are NOT vertices.

3. **Pebble Ownership Tracking:** Every pebble has a tracked owner. The `pebble_owner[]` array is updated atomically with every pebble move.

4. **Integration Boundary:** Cluster decomposition occurs strictly inside [`solve_DL()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2270), after the function's local variable declarations (lines 2314–2325) and before [`subsys->redirectParams()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2327). The existing `SubSystem`'s [`pmap`](src/Mod/Sketcher/App/planegcs/SubSystem.h:44) and [`pvals`](src/Mod/Sketcher/App/planegcs/SubSystem.h:45) are NEVER modified by the cluster path — they remain cluster-transparent. Cluster-local solves operate on position-indexed sub-views of `subsys->pvals`, never mutating `pmap`.

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

#### 2.1.2 Constraint-to-Edge Mapping (Synced with §5.1)

Each constraint maps to one or more pebble-game edges. An edge consumes exactly 1 pebble. The edge is "covered" (satisfied) when it holds a pebble.

| Constraint Type | ID | Vertices Involved | Edge Count | Notes |
|----------------|----|-------------------|------------|-------|
| `None` | 0 | — | 0 | Excluded; no constraint |
| `Equal` | 1 | (absorbed by reductionmap) | 0 | Handled by [`reductionmaps`](src/Mod/Sketcher/App/planegcs/GCS.h:137) before pebble game runs |
| `Difference` | 2 | (absorbed by reductionmap) | 0 | Ditto |
| `P2PDistance` | 3 | p1, p2 | 1 | Two point vertices, one distance edge |
| `P2PAngle` | 4 | p1, p2 | 1 | Angle of vector p1→p2; angle value param is NOT a vertex |
| `P2LDistance` | 5 | p, l.p1, l.p2 | 1 | Point-to-line signed distance |
| `PointOnLine` | 6 | p, l.p1, l.p2 | 1 | Collinearity constraint |
| `PointOnPerpBisector` | 7 | p0, p1, p2 | 1 | 3-point constraint |
| `Parallel` | 8 | l1.p1, l1.p2, l2.p1, l2.p2 | 1 | Hyperedge: cross(d1, d2) = 0 over 4 points |
| `Perpendicular` | 9 | l1.p1, l1.p2, l2.p1, l2.p2 | 1 | Hyperedge: dot(d1, d2) = 0 over 4 points |
| `L2LAngle` | 10 | l1.p1, l1.p2, l2.p1, l2.p2 | 1 | Hyperedge: angle between two lines; angle value param is NOT a vertex |
| `MidpointOnLine` | 11 | l1.p1, l1.p2, l2.p1, l2.p2 | 1 | Hyperedge: midpoint of l1 lies on line l2 |
| `TangentCircumf` | 12 | p1, p2 | 1 | Tangent circles; radii are value parameters, NOT vertices |
| `PointOnEllipse` | 13 | p, e.center, e.focus1 | 1 | e.radmin is a value parameter, NOT a vertex |
| `TangentEllipseLine` | 14 | e.center, e.focus1, l.p1, l.p2 | 1 | Hyperedge; e.radmin is a value param |
| `InternalAlignmentPoint2Ellipse` | 15 | p, e.center, e.focus1 | 1 | Alignment to ellipse major/minor axes or foci |
| `EqualMajorAxesConic` | 16 | e1.{center,focus1}, e2.{center,focus1} | 1 | Hyperedge: major axis equality for two conics; radmin params are NOT vertices |
| `EllipticalArcRangeToEndPoints` | 17 | arc.center, arc.start, arc.end, p1, p2 | 1 | Hyperedge: arc endpoint range constraint; arc.rad, angles are value params |
| `AngleViaPoint` | 18 | p, crv1 geom points, crv2 geom points | 1 | Hyperedge; angle value param is NOT a vertex |
| `Snell` | 19 | p, ray1 endpoints, ray2 endpoints, boundary endpoints | 1 | Hyperedge: multi-geometry refraction; n1, n2 are value params |
| `CurveValue` | 20 | p, crv geom points | 1 | Point-on-curve at parameter u; u is a value param |
| `PointOnHyperbola` | 21 | p, h.center, h.focus1 | 1 | h.radmin is a value parameter, NOT a vertex |
| `InternalAlignmentPoint2Hyperbola` | 22 | p, h.center, h.focus1 | 1 | Alignment to hyperbola axes or foci |
| `PointOnParabola` | 23 | p, pb.focus1 | 1 | Parabola focus is a point vertex; focal length is value param |
| `EqualFocalDistance` | 24 | c1.focus1, c1.focus2, c2.focus1, c2.focus2 | 1 | Hyperedge: equal focal distances of two conics |
| `EqualLineLength` | 25 | l1.p1, l1.p2, l2.p1, l2.p2 | 1 | Hyperedge: ||l1|| = ||l2|| |
| `CenterOfGravity` | 26 | center, p1..pn | 1 | Hyperedge: N-point constraint |
| `WeightedLinearCombination` | 27 | q, p1..pn | 1 | Hyperedge: N-point linear combination; weights are value params |
| `SlopeAtBSplineKnot` | 28 | bsp control points | 1 | Hyperedge; knot parameter u is a value param |
| `PointOnBSpline` | 29 | p, bsp control points | 1 | Hyperedge; curve parameter u is a value param |
| `C2CDistance` | 30 | crv1 geom points, crv2 geom points | 1 | Hyperedge: distance between two curves; d is value param |
| `C2LDistance` | 31 | crv geom points, l.p1, l.p2 | 1 | Hyperedge: curve-to-line distance; d is value param |
| `P2CDistance` | 32 | p, crv geom points | 1 | Point-to-curve distance; d is value param |
| `AngleViaPointAndParam` | 33 | p, crv1 geom points, crv2 geom points | 1 | Hyperedge; angle + parameters are value params |
| `AngleViaPointAndTwoParams` | 34 | p, crv1 geom points, crv2 geom points | 1 | Hyperedge; angle + two parameters are value params |
| `AngleViaTwoPoints` | 35 | p1, p2, crv1 geom points, crv2 geom points | 1 | Hyperedge; angle is deduced from points |
| `ArcLength` | 36 | arc.{center,start,end} | 1 | arc.rad, angles, d are value params |

**Key design principle:** Scalar value parameters (distances, angles, radii, focal lengths, curve parameters) are NEVER vertices in the pebble game. They are treated as constants during the topological phase. The Jacobian columns for these parameters still participate in the Newton iteration, but they do not consume pebbles in the rigidity matroid.

**Rationale for removing virtual direction vertices:** In 2D, a unit direction vector (cos θ, sin θ) has exactly 1 degree of freedom (the angle θ). Assigning it 2 pebbles (as a "virtual vertex") violates the Laman sparsity condition by overcounting 1 DOF. This would classify certain underconstrained systems as rigid, producing false-positive cluster decompositions. The downstream [`solve_DL()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2270) would then encounter a singular Jacobian that the divergence guard at [`GCS.cpp:2366`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2366) (which checks `err > divergingLim || err != err`) cannot detect as a rank deficiency — it only catches NaN and divergence, not singular matrices with finite residuals.

#### 2.1.3 Hyperedge Pebble Game Foundation

Constraints connecting >2 vertices (Parallel, Perpendicular, L2LAngle, MidpointOnLine, TangentEllipseLine, EqualMajorAxesConic, EllipticalArcRangeToEndPoints, EqualLineLength, CenterOfGravity, WeightedLinearCombination, SlopeAtBSplineKnot, PointOnBSpline, C2CDistance, C2LDistance, AngleViaPoint, AngleViaPointAndParam, AngleViaPointAndTwoParams, AngleViaTwoPoints, Snell, EqualFocalDistance) form **hyperedges** in the pebble game graph — edges incident to 3 or more vertices. The classical 2D pebble game operates on simple graphs (each edge has exactly 2 endpoints).

**Theoretical foundation:** Streinu & Theran (2009), "Sparsity-certifying Graph Decompositions" (Graphs and Combinatorics, 25:219–243), generalizes the (k,l)-pebble game to **hypergraphs**. The key theorem (Theorem 4.1) establishes that for a hypergraph H, the (k,l)-pebble game correctly detects Laman-type sparsity when hyperedges are treated as single edges that consume l pebbles from **any** of their incident vertices. For our 2D case (k=2, l=3 per rigid body, 1 pebble consumed per constraint hyperedge), the matroid invariant holds: each hyperedge consumes exactly 1 pebble from any one of its ≥2 endpoint vertices during the greedy orientation phase, and the DFS flip cascade traverses hyperedges identically to simple edges by branching through all incident vertices.

**Reduction strategy** (for constraint types with known, fixed topology):

| Hyperedge Constraint | Reduction to Simple Edges |
|----------------------|--------------------------|
| `Parallel` (4 pts) | `cross(d1, d2) = 0` decomposes as: 1 edge connecting l1.p1↔l1.p2, 1 edge connecting l2.p1↔l2.p2, 1 edge connecting the two direction-parameter pairs. However, since direction params are NOT vertices, we keep the single 4-vertex hyperedge. |
| `Perpendicular` (4 pts) | Same structure as Parallel; single 4-vertex hyperedge. |
| `L2LAngle` (4 pts) | Same structure as Parallel; angle parameter is NOT a vertex. |
| `MidpointOnLine` (4 pts) | l1 midpoint = (p1+p2)/2 must lie on l2; reduces to a collinearity constraint on a derived midpoint. Kept as single hyperedge for Phase 4a; midpoint-derivation decomposition deferred to Phase 4b. |
| `CenterOfGravity` (N+1 pts) | center = Σ(p_i)/N; reduces to N equality constraints but creates N unique edges. Kept as single N-vertex hyperedge; edge-splitting optimization deferred to Phase 4b. |
| `WeightedLinearCombination` (N+1 pts) | q = Σ(w_i · p_i); same as CoG with weights. Single hyperedge for Phase 4a. |

**Known limitation (regression test plan):** Hyperedge constraints with >6 vertices may produce false-positive cluster decoupling when the pebble game fails to detect hidden rigidity dependencies through the hyperedge's internal topology. Mitigation: if cluster-local Newton-Raphson fails to converge in ≤10 iterations AND the cluster contains a hyperedge with >6 vertices, fall back to monolithic solve for that SubSystem. A regression test harness (`test_hyperedge_pebble.cpp`) will exercise all 18 hyperedge constraint types with 4, 6, 8, and 10-vertex configurations against known ground-truth rigidity classifications from the FreeCAD sketcher test suite at [`tests/src/Mod/Sketcher/App/planegcs/GCS.cpp`](tests/src/Mod/Sketcher/App/planegcs/GCS.cpp).

#### 2.1.4 Non-Generic Constraint Considerations

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

// Pebble ownership tracking (v3 FIX for BLOCKER 2)
std::vector<int> pebble_owner;             // [total_pebbles] which vertex owns each pebble
                                           // pebble_owner[i] == v means pebble i is at vertex v
                                           // A free pebble (not held by any vertex) has owner = -1
                                           // Initialized: pebble j owned by vertex j/2 for j < 2*num_vertices

// Per-edge state
std::vector<std::vector<int>> edge_vertices; // [num_edges] list of vertex indices incident to edge
                                             // For hyperedges, this list has >2 entries (§2.1.3)
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

#### 2.2.3 DFS Pebble Collection with Ownership Transfer (v3 BLOCKER 2 FIX)

When an edge `e` is uncovered but all its incident vertices have 0 pebbles, the algorithm attempts to find a pebble via DFS flip along a path of covered edges.

**v2 Defect:** The cascade walked from `target_vertex` (leaf) upward and attempted to draw a pebble from `parent` at each step, but `parent` may have 0 pebbles — producing `pebble_owner[-1]` out-of-bounds writes. After the cascade, it attempted to orient the target edge using `target_vertex`'s pebbles (which had been consumed during the cascade).

**v3 Fix:** The DFS searches from the target-edge's incident vertices (DFS roots) outward to find a vertex `src` that has free pebbles. The cascade walks `src → root` (leaf→root), moving `current`'s free pebble onto the edge and the edge's old pebble onto `parent`. After the cascade, the DFS root (which is an incident vertex of `target_edge`) now holds +1 free pebble — this pebble is used to cover `target_edge`.

```cpp
/**
 * Attempt to collect a pebble for edge `target_edge` by performing a DFS
 * along covered edges to find a vertex that still has pebbles, then flip
 * pebble ownership along the found path.
 *
 * Returns true if a pebble was successfully collected.
 *
 * v3 FIX (BLOCKER 2): Cascade direction corrected to leaf→root.
 *   At each step: current (has pebbles) donates one free pebble to the edge,
 *   and the edge's old pebble moves to parent. Net effect after cascade:
 *   src loses 1 pebble, DFS root gains 1 pebble. The root's pebble covers
 *   target_edge.
 *
 * CRITICAL INVARIANT:
 *   pebble_owner[] is updated atomically with every pebble move.
 *   After the cascade, vertex_pebbles[root] reflects the gained pebble.
 */
bool collectPebble(int target_edge) {
    // ---- Phase 1: DFS from target_edge incident vertices ----
    // Reset DFS workspace
    dfs_visited.assign(num_vertices, false);
    dfs_parent.assign(num_vertices, -1);
    dfs_edge_to_parent.assign(num_vertices, -1);

    // Seed DFS from all incident vertices of target_edge as DFS roots
    int dfs_stack_top = 0;
    for (int v : edge_vertices[target_edge]) {
        dfs_visited[v] = true;
        dfs_parent[v] = -1;  // root of DFS tree — this is an incident vertex of target_edge
        dfs_stack[dfs_stack_top++] = v;
    }

    int src_vertex = -1;  // vertex WITH free pebbles found by DFS (the "leaf")

    while (dfs_stack_top > 0) {
        int u = dfs_stack[--dfs_stack_top];

        // v3 FIX: Check CURRENT vertex (u) for free pebbles, not parent
        if (vertex_pebbles[u] > 0) {
            src_vertex = u;
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

    if (src_vertex == -1) {
        return false;  // No pebble found; edge is overconstrained
    }

    // ---- Phase 2: Pebble flip cascade (v3: leaf→root direction) ----
    // Walk from src_vertex (leaf, has pebbles) toward the DFS root.
    // At each step:
    //   1. Take a free pebble from `current` (which we KNOW has one)
    //   2. Move it to edge `dfs_edge_to_parent[current]`
    //   3. Move the old edge pebble to `parent`
    //
    // Net flow: pebbles flow FROM src_vertex TOWARD root.
    // After cascade: src_vertex has -1 pebble, root has +1 pebble.
    //
    // All intermediate vertices keep their original pebble counts unchanged.

    int current = src_vertex;
    while (dfs_parent[current] != -1) {
        int parent = dfs_parent[current];
        int edge_idx = dfs_edge_to_parent[current];

        // ---- Find a FREE pebble owned by `current` (not on any edge) ----
        // `current` is guaranteed to have vertex_pebbles[current] > 0
        // because `src_vertex` was found with pebbles and subsequent
        // iterations of the loop also guarantee it (parent gets a pebble
        // from the edge at each step).
        int current_free_pebble = -1;
        for (int p = 0; p < 2 * num_vertices; p++) {
            if (pebble_owner[p] == current) {
                // Verify it's not on any edge
                bool on_edge = false;
                for (int ee = 0; ee < num_edges; ee++) {
                    if (edge_pebble[ee] == p) { on_edge = true; break; }
                }
                if (!on_edge) {
                    current_free_pebble = p;
                    break;
                }
            }
        }
        // current_free_pebble MUST be found (invariant: current has free pebbles)
        assert(current_free_pebble != -1);

        // The old pebble on the edge will move to parent
        int old_edge_pebble = edge_pebble[edge_idx];

        // ---- ATOMIC OWNERSHIP TRANSFER (v3 FIX) ----
        // current's free pebble → edge
        pebble_owner[current_free_pebble] = -1;       // pebble now on edge, no vertex owner
        edge_pebble[edge_idx] = current_free_pebble;   // edge now holds current's pebble
        vertex_pebbles[current]--;                      // current loses a free pebble

        // Old edge pebble → parent
        pebble_owner[old_edge_pebble] = parent;        // parent now owns the old edge pebble
        vertex_pebbles[parent]++;                       // parent gains a free pebble

        current = parent;
    }
    // After loop: `current` is the DFS root (an incident vertex of target_edge).
    // The root now has +1 pebble from the cascade.

    int dfs_root = current;  // incident vertex of target_edge that gained a pebble

    // ---- Phase 3: Cover target_edge using the root's pebble ----
    // v3 FIX: Use dfs_root (not src_vertex / target_vertex from v2).
    // dfs_root is guaranteed to have ≥1 free pebble after the cascade.
    for (int p = 0; p < 2 * num_vertices; p++) {
        if (pebble_owner[p] == dfs_root) {
            // Verify it's not on any edge
            bool on_edge = false;
            for (int ee = 0; ee < num_edges; ee++) {
                if (edge_pebble[ee] == p) { on_edge = true; break; }
            }
            if (!on_edge) {
                pebble_owner[p] = -1;
                edge_pebble[target_edge] = p;
                edge_covered[target_edge] = true;
                vertex_pebbles[dfs_root]--;
                return true;
            }
        }
    }

    return false;  // Should not reach here if cascade succeeded
}
```

**v3 Resolution Verification:** The cascade direction is now leaf→root. At each step, `current` (which we can PROVE has free pebbles because `src_vertex` was found by DFS checking `vertex_pebbles[u] > 0`, and subsequent `current` values inherit pebbles from the edge transfer) donates to the edge. The final orientation uses `dfs_root` (an incident vertex of `target_edge` that just gained a pebble), not `src_vertex`. This eliminates both (a) the `pebble_owner[-1]` UB from searching parent pebbles, and (b) the stale `target_vertex` reference after cascade.

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

Overconstrained constraints are mapped to [`System::redundant`](src/Mod/Sketcher/App/planegcs/GCS.h:140) (per DAG fallback definition).

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

**Dependency direction:** The donor cluster (whose vertex contributed the pebble) is solved first. The recipient cluster(s), whose vertices are incident to the edge but did NOT donate the pebble, depend on the donor.

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
        // Determine donor cluster:
        // The pebble on edge e was taken from a specific vertex during
        // greedy orientation. That vertex's cluster is the DONOR.
        // All other clusters in `clusters_in_edge` are RECIPIENTS.
        //
        // Find which vertex the pebble came from:
        //   The pebble on the edge has pebble_owner[p] == -1 (on edge).
        //   During greedy initialization, we recorded which vertex donated.
        //   Store this in edge_donor_vertex[e] during initializePebbleGame().

        int donor_vertex = edge_donor_vertex[e];  // recorded during greedy init
        int donor_cluster = component_id[donor_vertex];

        for (int c : clusters_in_edge) {
            if (c != donor_cluster) {
                // Donor must be solved before recipient
                dag_adj[donor_cluster].push_back(c);
                dag_in_degree[c]++;
            }
        }
    }
}
```

**Additional state to record during initialization (add to `initializePebbleGame()`):**

```cpp
std::vector<int> edge_donor_vertex;  // [num_edges] which vertex donated the pebble covering this edge
// Set during greedy orientation:
//   edge_donor_vertex[e] = v;  (the vertex that had vertex_pebbles[v] > 0 when covering edge e)
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

#### 3.3.2 Underconstrained Detection

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

#### 3.3.3 Cycle Detection

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

**Strategy A (scrapped):** Was to inject cluster decomposition at [`initSolution()` line 1739](src/Mod/Sketcher/App/planegcs/GCS.cpp:1739), splitting `reductionmaps[cid]` across clusters and modifying the [`solve()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:1906) iteration order. **Rejected** as too invasive for a Phase 4a foundation commit.

**Strategy B (mandated):** Cluster decomposition happens **inside** [`solve_DL()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2270), between the local variable declarations (lines 2314–2325) and the first call to [`subsys->redirectParams()`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2327).

### 4.2 solve_DL() Modified Pseudocode (with Cluster-Local Dogleg Specification)

```cpp
int System::solve_DL(SubSystem* subsys, bool isRedundantsolving)
{
    // ... existing preamble (lines 2270–2325) unchanged ...
    // Variables declared at lines 2314–2325:
    //   xsize, csize, tolg, tolx, tolf, maxIterNumber
    //   A_sparse, sparse_ldlt, sparse_pattern_locked, diag_offsets, pattern_triplets, mu
    //   x(xsize), x_new(xsize), fx(csize), fx_new(csize)
    //   Jx(csize, xsize), Jx_new(csize, xsize)
    //   g(xsize), h_sd(xsize), h_gn(xsize), h_dl(xsize)

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
    // Solve clusters in DAG topological order.
    //
    // v3 pmap POLICY: subsys->pmap is NEVER modified in the clustered path.
    // Instead, we build cluster-local parameter index vectors and operate
    // directly on subsys->pvals via position indices.

    // Pre-allocate flat parameter-to-pval-index lookup (NON-BLOCKING 5 mitigation)
    // Maps original parameter pointer → position in subsys->pvals
    std::vector<std::pair<double*, int>> param_to_pval_index;
    param_to_pval_index.reserve(subsys->pSize());
    for (int i = 0; i < subsys->pSize(); i++) {
        param_to_pval_index.emplace_back(subsys->plist[i], i);
    }
    std::sort(param_to_pval_index.begin(), param_to_pval_index.end());

    // Fast lookup lambda (binary search on sorted flat array — cache-friendly)
    auto pval_lookup = [&](double* param) -> int {
        auto it = std::lower_bound(param_to_pval_index.begin(),
                                    param_to_pval_index.end(),
                                    std::make_pair(param, 0),
                                    [](const auto& a, const auto& b) {
                                        return a.first < b.first;
                                    });
        if (it != param_to_pval_index.end() && it->first == param) {
            return it->second;  // index into subsys->pvals
        }
        return -1;  // parameter not in this subsystem (boundary / external param)
    };

    int overall_result = Success;

    for (int cluster_idx : solve_order) {
        ClusterInfo& ci = clusters[cluster_idx];

        // ---- §4.2.1: Build cluster-local parameter index map ----
        // ci.param_pointers[] contains the double* to original params in this cluster
        // ci.param_count = number of parameters in this cluster
        // ci.constraint_indices[] contains the indices into subsys->clist

        std::vector<int> cluster_pval_indices;     // [local_param_idx] → pvals index
        cluster_pval_indices.reserve(ci.param_count);
        for (int lp = 0; lp < ci.param_count; lp++) {
            int pi = pval_lookup(ci.param_pointers[lp]);
            // pi != -1: cluster params are subsets of subsystem params
            cluster_pval_indices.push_back(pi);
        }

        // ---- §4.2.2: Slice Jacobian and residual for this cluster ----
        // Cluster-local dimensions
        int cl_xsize = ci.param_count;                    // number of parameters
        int cl_csize = static_cast<int>(ci.constraint_indices.size());  // number of constraints

        // ---- §4.2.2a: Jx slicing ----
        // The monolithic Jx has columns indexed by global pvals position.
        // Cluster Jx_c maps: cluster constraint rows × cluster parameter columns.
        //
        // Use Eigen::Map with stride for zero-copy where possible?
        // No — Jx columns are not contiguous in the cluster subset. Must copy.
        // Pre-allocate Jx_c workspace once for the largest cluster.
        Eigen::MatrixXd Jx_c(cl_csize, cl_xsize);
        Eigen::VectorXd fx_c(cl_csize);
        Eigen::VectorXd x_c(cl_xsize), x_new_c(cl_xsize), g_c(cl_xsize);
        Eigen::VectorXd h_sd_c(cl_xsize), h_gn_c(cl_xsize), h_dl_c(cl_xsize);

        // ---- §4.2.2b: fx slicing ----
        // The residual vector is ordered by constraint index in subsys->clist.
        // For each constraint in ci.constraint_indices[], copy its residual.

        // ---- §4.2.3: Pre-solve hook — subsys->redirectParams() equivalent ----
        // The standard path calls subsys->redirectParams() to populate the pmap
        // mapping original parameter pointers → pvals slots.
        //
        // v3 policy (§1.2 invariant 4): We do NOT modify subsys->pmap.
        // Instead, we construct a CLUSTER-LOCAL redirect:
        //   - Create a temporary MAP_pD_pD (or flat vector) mapping
        //     the cluster's original param pointers → subsys->pvals slots
        //   - Pass this to constraint evaluation via a modified call path.
        //
        // For Phase 4a, the simplest approach: before evaluating cluster
        // constraints, save the current subsys->pmap entries for cluster
        // parameters, update them, evaluate, then restore.
        //
        // v3 FINAL POLICY (cluster-transparent): Use save/restore at boundaries.
        // This is explicit, auditable, and zero-cost in the hot loop (done once
        // per cluster, not per Newton iteration).

        // Save original pmap entries for cluster params
        std::vector<std::pair<double*, double*>> saved_pmap_entries;
        saved_pmap_entries.reserve(cl_xsize);
        for (int lp = 0; lp < cl_xsize; lp++) {
            double* orig = ci.param_pointers[lp];
            double* pval_ptr = &subsys->pvals[cluster_pval_indices[lp]];
            auto it = subsys->pmap.find(orig);
            if (it != subsys->pmap.end()) {
                saved_pmap_entries.emplace_back(orig, it->second);
            } else {
                saved_pmap_entries.emplace_back(orig, nullptr);
            }
            subsys->pmap[orig] = pval_ptr;  // cluster-local redirect
        }

        // ---- §4.2.4: Cluster-local dogleg iteration ----
        // Convergence criteria per cluster (tuned per-cluster for efficiency):
        double cl_tolg = DL_tolg;
        double cl_tolx = DL_tolx;
        double cl_tolf = DL_tolf;
        int cl_maxIter = maxIterNumber;  // or scaled by cl_xsize
        double delta = 0.1;
        double nu = 2.0;
        int cl_iter = 0, cl_stop = 0, cl_reduce = 0;

        // Initial evaluation for this cluster
        // populate fx_c by evaluating only cluster constraints
        // populate Jx_c similarly

        while (!cl_stop) {
            // Convergence checks (same structure as monolithic, §4.2.4a):
            double fx_inf = fx_c.lpNorm<Eigen::Infinity>();
            double g_inf = (Jx_c.transpose() * (-fx_c)).lpNorm<Eigen::Infinity>();

            if (fx_inf <= cl_tolf) { cl_stop = 1; break; }
            if (g_inf <= cl_tolg) { cl_stop = 2; break; }
            if (delta <= cl_tolx * (cl_tolx + x_c.norm())) { cl_stop = 2; break; }
            if (cl_iter >= cl_maxIter) { cl_stop = 4; break; }

            // Gauss-Newton step (same switch as monolithic, reusing global dogLegGaussStep):
            // ... h_gn_c computed from Jx_c, fx_c ...

            // Dogleg blending:
            // ... h_dl_c computed from h_sd_c, h_gn_c, delta ...

            // Update and re-evaluate:
            x_new_c = x_c + h_dl_c;
            // ... evaluate fx_new_c at x_new_c ...
            // ... trust-region adjustment (delta, nu) ...
            cl_iter++;
        }

        // Post-solve: write x_c back to subsys->pvals via cluster_pval_indices
        for (int lp = 0; lp < cl_xsize; lp++) {
            subsys->pvals[cluster_pval_indices[lp]] = x_c(lp);
        }

        // Restore saved pmap entries
        for (auto& [orig, saved_val] : saved_pmap_entries) {
            if (saved_val != nullptr) {
                subsys->pmap[orig] = saved_val;
            } else {
                subsys->pmap.erase(orig);
            }
        }

        if (cl_stop > overall_result) {
            overall_result = cl_stop;
        }
    }

    return overall_result;
}
```

#### §4.2.4a: Detailed Cluster-Local Convergence Criteria

Per-cluster convergence mirrors the monolithic criteria at [`GCS.cpp:2347–2370`](src/Mod/Sketcher/App/planegcs/GCS.cpp:2347) but operates on the reduced `Jx_c`/`fx_c`/`x_c`:

| Condition | Monolithic Equivalent | Cluster-Local |
|-----------|----------------------|---------------|
| Residual small | `fx_inf <= tolf` (line 2349) | `fx_c.lpNorm<Infinity>() <= cl_tolf` |
| Gradient small | `g_inf <= tolg` (line 2354) | `(Jx_c^T · (-fx_c)).lpNorm<Infinity>() <= cl_tolg` |
| Step too small | `delta <= tolx * (tolx + x.norm())` (line 2358) | Same formula with `x_c` |
| Max iterations | `iter >= maxIterNumber` (line 2362) | `cl_iter >= cl_maxIter` |
| Diverging/NaN | `err > divergingLim \|\| err != err` (line 2366) | Same guard with cluster-local `err` |

#### §4.2.4b: Boundary Parameter Handling

Cluster constraints may reference parameters that are NOT in the cluster's parameter set (e.g., an inter-cluster edge connects a vertex in this cluster to one in a dependent cluster). These **boundary parameters** are:

1. **Pinned at current values:** Their values are taken from `subsys->pvals` at the start of the cluster solve and held constant during the cluster-local Newton iteration.
2. **Excluded from the cluster Jacobian columns:** The cluster-local `Jx_c` has columns only for the cluster's own parameters. Boundary parameters contribute to `fx_c` (the residual) but have NO corresponding columns in `Jx_c` (their derivatives are dropped).
3. **Updated after upstream cluster solves:** Because we solve clusters in DAG topological order, boundary parameters belonging to UPSTREAM (already-solved) clusters have their final converged values. Boundary parameters belonging to DOWNSTREAM clusters use their pre-solve values, which will be refined when those clusters are solved.

**Implementation in constraint evaluation:**

```cpp
// When evaluating constraint `c` for cluster `ci`:
//   For each parameter `p` referenced by `c`:
//     if (p is in ci.param_pointers[]):
//       → p is an ACTIVE cluster parameter (has a column in Jx_c)
//       → its value is read from x_c[local_idx] (the current Newton iterate)
//     else:
//       → p is a BOUNDARY parameter
//       → its value is read from subsys->pvals[pval_lookup(p)] (fixed at current value)
//       → derivatives w.r.t. p are NOT included in Jx_c
```

### 4.3 Existing Code — What Changes

| File | Change | Lines Affected |
|------|--------|---------------|
| [`GCS.cpp`](src/Mod/Sketcher/App/planegcs/GCS.cpp) | Add pebble game initialization + clustered solve path inside `solve_DL()` | After 2325, before 2327 |
| [`GCS.cpp`](src/Mod/Sketcher/App/planegcs/GCS.cpp) | Add `pebbleOwnerTransfer()` helper (used in DFS flip) | New static function |
| [`GCS.h`](src/Mod/Sketcher/App/planegcs/GCS.h) | Add `PebbleGameState` struct as private nested type | After line 111 |
| [`GCS.h`](src/Mod/Sketcher/App/planegcs/GCS.h) | Add `ClusterInfo` struct as private nested type | After PebbleGameState |

**No changes to:** `SubSystem.h`, `SubSystem.cpp`, `Constraints.h`, `Constraints.cpp`, `Geo.h`, `Geo.cpp`, `initSolution()`.

### 4.4 MAP_pD_pD Heap Allocation Mitigation

The [`MAP_pD_pD`](src/Mod/Sketcher/App/planegcs/Util.h:37) type (`std::map<double*, double*>`) performs a heap allocation on every `operator[]` or `insert()` call. This is problematic inside the Newton-Raphson hot loop.

**Mitigation strategy (consistent with §1.2 invariant 4 — cluster-transparent):**

1. **Cluster-boundary save/restore:** `subsys->pmap` is mutated ONLY once per cluster (before the dogleg loop), not per Newton iteration. The save/restore pattern (§4.2) bounds the pmap mutation to O(cluster_count) operations, not O(iteration_count).
2. **Flat lookup buffer:** Pre-allocate `std::vector<std::pair<double*, int>>` once per `solve_DL()` call, mapping parameter pointers → pvals indices. Use `std::lower_bound` for O(log n) lookups (binary search on sorted flat array — superior cache locality to `std::map` red-black tree traversal).
3. **Cluster-local iteration uses direct index access:** Once the cluster parameter indices are resolved (via the flat lookup buffer), the dogleg loop accesses `subsys->pvals[cidx]` and `x_c(local_idx)` directly with zero pointer-chasing.

---

## 5. Non-Generic Constraint Classification Catalog (COMPLETE — All 37 Types)

### 5.1 Constraint → Parameter Count → Vertex Count Mapping

This catalog provides the exact mapping for the pebble game's edge construction. "Value params" are scalar parameters excluded from the vertex set.

| Constraint Class | ID | Geom Params | Value Params | Vertices | Edges |
|-----------------|-----|-------------|-------------|----------|-------|
| `None` | 0 | — | — | — | 0 |
| `Equal` | 1 | 2 `double*` | 0 | 0 (absorbed by reductionmap) | 0 |
| `Difference` | 2 | 2 `double*` | 1 `double*` | 0 (absorbed by reductionmap) | 0 |
| `P2PDistance` | 3 | p1{x,y}, p2{x,y} | d | p1, p2 | 1 |
| `P2PAngle` | 4 | p1{x,y}, p2{x,y} | angle, da | p1, p2 | 1 |
| `P2LDistance` | 5 | p{x,y}, l.p1{x,y}, l.p2{x,y} | d | p, l.p1, l.p2 | 1 |
| `PointOnLine` | 6 | p{x,y}, l.p1{x,y}, l.p2{x,y} | — | p, l.p1, l.p2 | 1 |
| `PointOnPerpBisector` | 7 | p0{x,y}, p1{x,y}, p2{x,y} | — | p0, p1, p2 | 1 |
| `Parallel` | 8 | l1.p1{x,y}, l1.p2{x,y}, l2.p1{x,y}, l2.p2{x,y} | — | l1.p1, l1.p2, l2.p1, l2.p2 | 1 |
| `Perpendicular` | 9 | l1.p1{x,y}, l1.p2{x,y}, l2.p1{x,y}, l2.p2{x,y} | — | l1.p1, l1.p2, l2.p1, l2.p2 | 1 |
| `L2LAngle` | 10 | l1.p1{x,y}, l1.p2{x,y}, l2.p1{x,y}, l2.p2{x,y} | angle | l1.p1, l1.p2, l2.p1, l2.p2 | 1 |
| `MidpointOnLine` | 11 | l1.p1{x,y}, l1.p2{x,y}, l2.p1{x,y}, l2.p2{x,y} | — | l1.p1, l1.p2, l2.p1, l2.p2 | 1 |
| `TangentCircumf` | 12 | p1{x,y}, p2{x,y} | rad1, rad2 | p1, p2 | 1 |
| `PointOnEllipse` | 13 | p{x,y}, e.center{x,y}, e.focus1{x,y} | e.radmin | p, e.center, e.focus1 | 1 |
| `TangentEllipseLine` | 14 | e.center{x,y}, e.focus1{x,y}, l.p1{x,y}, l.p2{x,y} | e.radmin | e.center, e.focus1, l.p1, l.p2 | 1 |
| `InternalAlignmentPoint2Ellipse` | 15 | p{x,y}, e.center{x,y}, e.focus1{x,y} | alignment type (enum) | p, e.center, e.focus1 | 1 |
| `EqualMajorAxesConic` | 16 | e1.{center,focus1}{x,y}, e2.{center,focus1}{x,y} | e1.radmin, e2.radmin | e1.center, e1.focus1, e2.center, e2.focus1 | 1 |
| `EllipticalArcRangeToEndPoints` | 17 | arc.{center,start,end}{x,y}, p1{x,y}, p2{x,y} | arc.rad, arc.startAngle, arc.endAngle | arc.center, arc.start, arc.end, p1, p2 | 1 |
| `AngleViaPoint` | 18 | p{x,y}, crv1 geom points, crv2 geom points | angle | p + crv1 points + crv2 points | 1 |
| `Snell` | 19 | p{x,y}, ray1 endpoints, ray2 endpoints, boundary endpoints | n1, n2 | p + all ray/boundary points | 1 |
| `CurveValue` | 20 | p{x,y}, pcoord, crv geom points | u | p + crv points | 1 |
| `PointOnHyperbola` | 21 | p{x,y}, h.center{x,y}, h.focus1{x,y} | h.radmin | p, h.center, h.focus1 | 1 |
| `InternalAlignmentPoint2Hyperbola` | 22 | p{x,y}, h.center{x,y}, h.focus1{x,y} | alignment type (enum) | p, h.center, h.focus1 | 1 |
| `PointOnParabola` | 23 | p{x,y}, pb.focus1{x,y} | focal length | p, pb.focus1 | 1 |
| `EqualFocalDistance` | 24 | c1.{focus1,focus2}{x,y}, c2.{focus1,focus2}{x,y} | — | c1.focus1, c1.focus2, c2.focus1, c2.focus2 | 1 |
| `EqualLineLength` | 25 | l1.p1{x,y}, l1.p2{x,y}, l2.p1{x,y}, l2.p2{x,y} | — | l1.p1, l1.p2, l2.p1, l2.p2 | 1 |
| `CenterOfGravity` | 26 | center{x,y}, p1..pn{x,y} | n (weights) | center, p1..pn | 1 |
| `WeightedLinearCombination` | 27 | q{x,y}, p1..pn{x,y} | n (weights) | q, p1..pn | 1 |
| `SlopeAtBSplineKnot` | 28 | bsp control points{x,y} | u (knot parameter) | bsp control points | 1 |
| `PointOnBSpline` | 29 | p{x,y}, bsp control points{x,y} | u (curve parameter) | p + bsp control points | 1 |
| `C2CDistance` | 30 | crv1 geom points, crv2 geom points | d | crv1 points + crv2 points | 1 |
| `C2LDistance` | 31 | crv geom points, l.p1{x,y}, l.p2{x,y} | d | crv points + l.p1, l.p2 | 1 |
| `P2CDistance` | 32 | p{x,y}, crv geom points | d | p + crv points | 1 |
| `AngleViaPointAndParam` | 33 | p{x,y}, crv1 geom points, crv2 geom points | angle, param | p + crv1 points + crv2 points | 1 |
| `AngleViaPointAndTwoParams` | 34 | p{x,y}, crv1 geom points, crv2 geom points | angle, param1, param2 | p + crv1 points + crv2 points | 1 |
| `AngleViaTwoPoints` | 35 | p1{x,y}, p2{x,y}, crv1 geom points, crv2 geom points | — | p1, p2 + crv1 points + crv2 points | 1 |
| `ArcLength` | 36 | arc.{center,start,end}{x,y} | arc.rad, arc.startAngle, arc.endAngle, d | arc.center, arc.start, arc.end | 1 |

**Note on curve constraints (IDs 13–36):** When a curve's geometric points (center, focus1, focus2, start, end) appear as `double*` entries in the subsystem's [`plist`](src/Mod/Sketcher/App/planegcs/SubSystem.h:43), they are included as pebble-game vertices. All scalar curve parameters (radii, angles, focal lengths, parameter u) are excluded from the vertex set.

### 5.2 Driving vs Driven Constraint Handling

Driven constraints ([`driving == false`](src/Mod/Sketcher/App/planegcs/Constraints.h:129)) do NOT consume pebbles. They are excluded from the pebble game edge set. The `SubSystem` constructor already separates driving and non-driving constraints via the tag mechanism ([`getTag() >= 0` check at line 1865](src/Mod/Sketcher/App/planegcs/GCS.cpp:1865)).

---

## 6. Implementation Sequence (Build Order)

### Phase 4a.1 — Data Structures (no behavioral change)
- Add `PebbleGameState` and `ClusterInfo` structs to [`GCS.h`](src/Mod/Sketcher/App/planegcs/GCS.h)
- Add `edge_donor_vertex[]` tracking to initialization
- Add pre-allocated workspace vectors as `System` private members
- Compile and verify no regression

### Phase 4a.2 — Pebble Game Engine (testable in isolation)
- Implement `PebbleGameState::initialize()` with vertex/edge construction
- Implement `collectPebble()` with DFS + v3 leaf→root pebble cascade (BLOCKER 2 fix)
- Add unit test: known rigid graph → all edges covered
- Add unit test: known underconstrained graph → correct free pebble count
- Add unit test: hyperedge constraints (8, 9, 10, 11, 26) with 4–10 vertices against ground-truth rigidity

### Phase 4a.3 — Cluster Decomposition
- Implement connected components on covered subgraph
- Implement DAG construction (with `edge_donor_vertex[]` for dependency direction) + Kahn topological sort
- Add cycle detection fallback
- Add overconstrained/underconstrained detection

### Phase 4a.4 — Strategy B Integration in solve_DL()
- Insert cluster decomposition call before line 2327
- Implement pmap save/restore at cluster boundaries (§4.2.3)
- Implement cluster-local dogleg iteration with Jx/fx slicing (§4.2.2)
- Implement boundary parameter handling (§4.2.4b)
- Implement monolithic fallback on decomposition failure

### Phase 4a.5 — Validation & Benchmarking
- Regression test against existing sketcher test suite ([`tests/src/Mod/Sketcher/App/planegcs/GCS.cpp`](tests/src/Mod/Sketcher/App/planegcs/GCS.cpp))
- Hyperedge regression test harness: 18 constraint types × 4 vertex-count configurations
- Benchmark cluster decomposition overhead vs solve time savings
- Profile pebble game on 100+ constraint sketches

---

## 7. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Pebble game O(|V|·|E|) DFS is too slow for large sketches | Medium | Medium | Iterative DFS with pre-allocated stack; early termination on pebble found |
| Cluster Jacobian sparsity pattern differs from monolithic | Low | High | Re-analyze sparse pattern per cluster (acceptable: pattern analysis is O(nnz), not O(n³)) |
| Inter-cluster parameter coupling breaks Newton convergence | Medium | Medium | DAG ordering ensures upstream clusters converge before downstream; boundary params pinned at converged values |
| DAG construction misidentifies dependency direction | Low | High | `edge_donor_vertex[]` directly records donor; no heuristic; if donor ambiguous, fall through to monolithic |
| Existing `redirectParams()` logic interacts badly with cluster-local pmap | Low | Medium | v3 pmap save/restore pattern bounds mutation to cluster boundaries; original pmap fully restored |
| Hyperedge pebble game misclassifies N>6 hyperedge rigidity | Medium | Medium | Fallback to monolithic if cluster solve fails AND contains hyperedge with >6 vertices |

---

## 8. Verification Checklist (for Adversarial Critic)

- [ ] BLOCKER 1: §5.1 catalog lists ALL 37 constraint types (IDs 0–36). No duplicates. §2.1.2 summary table is in 1:1 correspondence.
- [ ] BLOCKER 2: Cascade walks from `current` (leaf, known to have pebbles) toward root. At each step, `current` donates to edge. Final orientation uses `dfs_root`, not `src_vertex`.
- [ ] BLOCKER 3: `subsys->pmap` is NEVER modified in the clustered path. §1.2 invariant 4, §4.2.3, and §4.4 all describe save/restore at cluster boundaries — consistent.
- [ ] NON-BLOCKING 4: §4.2.4a specifies convergence criteria. §4.2.4b specifies boundary parameter handling. §4.2.2 specifies Jx/fx slicing.
- [ ] NON-BLOCKING 5: Streinu & Theran (2009) cited for hypergraph pebble game. Reduction strategies tabled. Regression test plan documented in §2.1.3 and Phase 4a.5.
- [ ] NON-BLOCKING 6: `initSolution()` cited at [`GCS.cpp:1739`](src/Mod/Sketcher/App/planegcs/GCS.cpp:1739). All other citations verified.
- [ ] All file/line citations verified against current codebase.
- [ ] Build order respects dependency: data structures → engine → clusters → integration.
- [ ] Zero dynamic allocation in Newton-Raphson inner loop.
- [ ] Eigen `Map` and `.noalias()` used for all matrix-vector products.
- [ ] Cluster-transparent pmap policy: one explicit save/restore per cluster boundary, zero pmap mutations in hot loop.
