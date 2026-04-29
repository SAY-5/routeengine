//! Tiny CLI for ad-hoc queries against a synthetic grid graph.
//! `cargo run --release --bin routequery -- 0 99` runs A* from
//! node 0 to node 99 of a generated 10×10 grid.

use routeengine::{astar, constraints, dijkstra, graph::*, spatial::LatLon};

fn build_grid(side: usize) -> Graph {
    let mut b = Graph::builder();
    let mut ids = Vec::with_capacity(side * side);
    for i in 0..side {
        for j in 0..side {
            ids.push(b.add_node(LatLon::new(i as f32 * 0.01, j as f32 * 0.01)));
        }
    }
    for i in 0..side {
        for j in 0..side {
            let here = ids[i * side + j];
            if j + 1 < side {
                b.add_bidirectional(Edge {
                    from: here,
                    to: ids[i * side + (j + 1)],
                    distance_m: 1110.0,
                    road_type: RoadType::Paved,
                    elevation_gain_m: 0.0,
                    weather_id: 0,
                });
            }
            if i + 1 < side {
                b.add_bidirectional(Edge {
                    from: here,
                    to: ids[(i + 1) * side + j],
                    distance_m: 1110.0,
                    road_type: RoadType::Paved,
                    elevation_gain_m: 0.0,
                    weather_id: 0,
                });
            }
        }
    }
    b.build()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: routequery <src> <dst> [side=10]");
        std::process::exit(2);
    }
    let src: NodeId = args[1].parse().expect("src");
    let dst: NodeId = args[2].parse().expect("dst");
    let side: usize = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(10);
    let g = build_grid(side);
    let cs = constraints::default_set();
    let ctx = constraints::SpatialContext::default();

    let (path, stats) = astar(&g, src, dst, &cs, &ctx).expect("path");
    println!("A* path: {:?}", path.nodes);
    println!(
        "A* cost: {:.1}m  popped={} relaxed={}",
        path.cost, stats.nodes_popped, stats.edges_relaxed
    );
    let dpath = dijkstra(&g, src, dst, &cs, &ctx).expect("path");
    println!("Dijkstra cost: {:.1}m (must equal A*)", dpath.cost);
    assert!((path.cost - dpath.cost).abs() < 1e-3);
}
