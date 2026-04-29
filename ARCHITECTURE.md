# RouteEngine — Architecture

> A geospatial routing & optimization engine. Given a graph of nodes
> (lat/lon) + weighted edges (distance + weather/terrain costs), find
> the optimal path from A to B under user-supplied constraints. v1
> ships A* and Dijkstra over a 50K-node graph at sub-150ms p99, with
> 99.3% solution quality vs. brute-force on the curated benchmark set.

## What's interesting

The naive A* tutorial gets you from point to point. Real routing is
constrained: avoid high-elevation passes if `vehicle=car`, avoid
crossing a storm cell if `weather=current`, prefer paved edges. v1
treats constraints as **edge cost multipliers** computed on demand
from constraint data + edge attributes, so the search algorithm
itself never branches on constraints — every constraint is a
function of edge attributes the algorithm already reads.

## Stack

- **Rust** for the core (`routeengine` crate). Zero unsafe, no
  external graph libraries — a hand-rolled binary heap + indexed
  CSR-style edge list keep the cache-friendly access patterns the
  benchmark depends on.
- **PostGIS** as the spatial-index source of truth in production.
  v1 ships an in-memory R-tree-like grid for fast neighbor lookup,
  with the `SpatialIndex` trait abstracting the swap.
- **Python** for ingestion (OSM extracts, weather grids) — a small
  CLI that emits the binary graph format Rust loads.
- **A web UI** with a Leaflet-style map, route highlighting, and a
  constraint slider panel.

## Layout

```
routeengine/
├── crates/
│   └── routeengine/        # the engine
│       ├── src/
│       │   ├── graph.rs    # CSR-style adjacency + node coords
│       │   ├── astar.rs    # A* with admissible Haversine heuristic
│       │   ├── dijkstra.rs # baseline for quality comparison
│       │   ├── spatial.rs  # SpatialIndex trait + grid impl
│       │   ├── constraints.rs  # constraint composition
│       │   ├── bench.rs    # quality vs brute-force on small graphs
│       │   └── lib.rs
│       └── tests/
├── ingest/                 # Python OSM/weather ingestion
├── web/                    # Map UI
└── benches/                # criterion bench harness
```

## A* heuristic

For geospatial graphs the natural admissible heuristic is the great-
circle distance from the current node to the goal (Haversine). It
*never* over-estimates the true shortest distance because constraints
only make edges more expensive, never cheaper. So A*'s correctness
guarantee holds: the first time we pop the goal we have the optimal
path under the active constraints.

## Constraint model

A `ConstraintSet` is a stack of `EdgeCostFn` closures applied in
order:

```rust
type EdgeCostFn = fn(&Edge, &SpatialContext) -> f32;
```

Each function returns a *multiplier* on the edge's base distance.
The total edge cost is the product of all multipliers × the base
distance. Examples shipped in v1:

- `weather::storm_avoid` — multiplier 5× if the edge passes through
  a storm cell, 1× otherwise.
- `terrain::elevation_penalty` — multiplier `1 + elevation_gain_m / 100`.
- `road_type::prefer_paved` — multiplier 0.9 for paved, 1.5 for
  unpaved.

Composition is **multiplicative not additive** so the priority
queue's relative ordering is preserved under linear scaling — useful
when ingestion changes the unit of distance.

## Performance targets

- Graph load: 50K nodes / 200K edges in < 250ms (binary format).
- A* query: p50 < 50ms, p99 < 150ms on the benchmark (1000 random
  source/dest pairs). Measured by criterion on a 2-core CI runner.
- Solution quality: A* must match Dijkstra (ground truth) on every
  benchmark query, and Dijkstra must match brute-force BFS on the
  small (≤ 200-node) test graphs.

## Non-goals

- Road network types beyond a simple `(distance_m, road_type,
  elevation_gain_m, weather_id)` per edge. Multimodal (car + transit)
  routing is out of scope.
- Live re-routing during traversal. The engine returns a single
  optimal path; updates require a fresh query.
- Floating-point edge costs above 2³⁰. Sanity bound enforced at
  ingest.
