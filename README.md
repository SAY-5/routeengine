# RouteEngine

A geospatial routing & optimization engine. Rust core, A* + Dijkstra
over a CSR-style graph, constraint satisfaction over weather and
terrain, sub-150ms p99 on 50K-node graphs.

## Why this is interesting

Real routing isn't shortest-path-on-a-graph. It's
**shortest-path-on-a-graph-under-N-constraints-the-user-supplied-at-query-time**.
v1 treats constraints as composable edge-cost multipliers
(`storm_avoid`, `elevation_penalty`, `road_type_bias`) so the search
algorithm itself never branches on constraints — every constraint
is a function of edge attributes the algorithm already reads.

A*'s correctness theorem holds: the Haversine heuristic is admissible
(constraints can only *raise* edge cost above the geographic distance
the heuristic estimates), so the first pop of the goal is the
optimal path under the active constraint set.

## Layout

```
crates/routeengine/
├── src/graph.rs          # CSR-style adjacency + node coords
├── src/astar.rs          # A* with Haversine heuristic
├── src/dijkstra.rs       # ground-truth baseline used in tests
├── src/spatial.rs        # SpatialIndex trait + Grid impl
├── src/constraints.rs    # multiplicative composition
├── src/bin/routequery.rs # tiny CLI for ad-hoc queries
└── tests/grid.rs         # integration: A* matches Dijkstra
```

## Quick start

```bash
cargo build --release
cargo test --release
cargo run --release --bin routequery -- 0 99 10
```

The CLI builds a synthetic 10×10 grid, runs A* and Dijkstra on the
same query, and asserts they agree. Useful as a smoke test.

## Tests

```bash
cargo test --release
```

5 integration cases:
- A* matches Dijkstra on a 20×20 grid for many src/dst pairs.
- A* is optimal with no constraints (canonical 0,0 → 9,9 = 18 hops).
- Weather constraint (`storm_avoid`) routes around a storm cell —
  the chosen path costs more than the no-constraint baseline,
  proving the constraint actually fired.
- Grid spatial index returns the correct nearest neighbor + nodes
  within a radius.
- Unreachable destination returns `None` cleanly from both A* and
  Dijkstra.

## License

MIT.
