//! v2: bidirectional A*. Asserts equivalence with single-direction
//! A* on every test query, plus a "fewer pops than unidirectional"
//! check on at least one large-grid case so a regression that
//! disables the meet-in-the-middle optimization is caught.

use routeengine::{
    astar, bidir_astar,
    constraints::{ConstraintSet, SpatialContext},
    graph::{Edge, Graph, NodeId, RoadType},
    reverse_adjacency,
    spatial::LatLon,
};

fn build_grid(side: usize) -> (Graph, Vec<NodeId>) {
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
    (b.build(), ids)
}

#[test]
fn bidir_matches_unidir_on_grid() {
    let (g, _) = build_grid(20);
    let rev = reverse_adjacency(&g);
    let cs = ConstraintSet::new();
    let ctx = SpatialContext::default();
    for src in (0..400).step_by(53) {
        for dst in (1..400).step_by(67) {
            let uni = astar(&g, src, dst, &cs, &ctx).expect("a*");
            let bi = bidir_astar(&g, &rev, src, dst, &cs, &ctx).expect("bi");
            assert!(
                (uni.0.cost - bi.0.cost).abs() < 1e-3,
                "src={} dst={} uni={} bi={}",
                src,
                dst,
                uni.0.cost,
                bi.0.cost
            );
        }
    }
}

#[test]
fn bidir_visits_fewer_nodes_for_long_path() {
    let (g, _) = build_grid(40); // larger grid: more room to save.
    let rev = reverse_adjacency(&g);
    let cs = ConstraintSet::new();
    let ctx = SpatialContext::default();
    let src = 0;
    let dst = (40 * 40 - 1) as NodeId;
    let uni = astar(&g, src, dst, &cs, &ctx).expect("a*");
    let bi = bidir_astar(&g, &rev, src, dst, &cs, &ctx).expect("bi");
    let bi_total = bi.1.forward_pops + bi.1.backward_pops;
    assert!(
        bi_total < uni.1.nodes_popped,
        "expected bidir to pop fewer nodes; uni={} bidir(fwd+bwd)={}",
        uni.1.nodes_popped,
        bi_total
    );
}

#[test]
fn bidir_handles_same_src_dst() {
    let (g, _) = build_grid(5);
    let rev = reverse_adjacency(&g);
    let (path, _) = bidir_astar(
        &g,
        &rev,
        0,
        0,
        &ConstraintSet::new(),
        &SpatialContext::default(),
    )
    .expect("path");
    assert_eq!(path.nodes, vec![0]);
    assert_eq!(path.cost, 0.0);
}

#[test]
fn bidir_returns_none_when_disconnected() {
    let mut b = Graph::builder();
    let a = b.add_node(LatLon::new(0.0, 0.0));
    let c = b.add_node(LatLon::new(1.0, 1.0));
    let g = b.build();
    let rev = reverse_adjacency(&g);
    assert!(bidir_astar(
        &g,
        &rev,
        a,
        c,
        &ConstraintSet::new(),
        &SpatialContext::default(),
    )
    .is_none());
}
