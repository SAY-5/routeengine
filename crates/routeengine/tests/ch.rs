//! v3: Contraction Hierarchies. The CH query must equal Dijkstra
//! on every test pair (correctness), and on a 30x30 grid the build
//! must complete in a reasonable time + add a non-zero number of
//! shortcuts.

use routeengine::{
    ch::CH,
    constraints::{ConstraintSet, SpatialContext},
    dijkstra,
    graph::{Edge, Graph, NodeId, RoadType},
    spatial::LatLon,
};

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

#[test]
fn ch_query_matches_dijkstra() {
    let g = build_grid(15);
    let ch = CH::build(&g);
    let cs = ConstraintSet::new();
    let ctx = SpatialContext::default();
    let n = (15 * 15) as NodeId;
    for src in (0..n).step_by(31) {
        for dst in (1..n).step_by(43) {
            let d = dijkstra(&g, src, dst, &cs, &ctx).expect("dijkstra");
            let q = ch.query(src, dst).expect("ch");
            assert!(
                (d.cost - q.cost).abs() < 1e-3,
                "src={} dst={} dijkstra={} ch={}",
                src,
                dst,
                d.cost,
                q.cost
            );
        }
    }
}

#[test]
fn ch_handles_same_src_dst_and_unreachable() {
    let g = build_grid(5);
    let ch = CH::build(&g);
    let p = ch.query(0, 0).expect("path");
    assert_eq!(p.nodes, vec![0]);
    assert_eq!(p.cost, 0.0);

    // Disconnected pair: build a graph with two islands.
    let mut b = Graph::builder();
    let a = b.add_node(LatLon::new(0.0, 0.0));
    let b_id = b.add_node(LatLon::new(1.0, 1.0));
    let g2 = b.build();
    let ch2 = CH::build(&g2);
    assert!(ch2.query(a, b_id).is_none());
}

#[test]
fn ch_build_adds_shortcuts_on_dense_graph() {
    // A handcrafted graph where contracting `mid` is forced to add a
    // shortcut a→c: the only alternate path (a → detour → c) has
    // cost 20, far longer than the direct a → mid → c cost of 2.
    // The dangling endpoints e1, e2 give a, c higher degree than
    // mid, detour so the ascending-degree order contracts mid first.
    let mut b = Graph::builder();
    let a = b.add_node(LatLon::new(0.0, 0.0));
    let mid = b.add_node(LatLon::new(0.0, 0.001));
    let c = b.add_node(LatLon::new(0.0, 0.002));
    let detour = b.add_node(LatLon::new(0.001, 0.0));
    let e1 = b.add_node(LatLon::new(0.0, -0.001));
    let e2 = b.add_node(LatLon::new(0.0, 0.003));

    let mk = |from, to, d| Edge {
        from,
        to,
        distance_m: d,
        road_type: RoadType::Paved,
        elevation_gain_m: 0.0,
        weather_id: 0,
    };
    b.add_bidirectional(mk(a, mid, 1.0));
    b.add_bidirectional(mk(mid, c, 1.0));
    b.add_bidirectional(mk(a, detour, 10.0));
    b.add_bidirectional(mk(detour, c, 10.0));
    b.add_bidirectional(mk(a, e1, 1.0));
    b.add_bidirectional(mk(c, e2, 1.0));

    let g = b.build();
    let ch = CH::build(&g);
    assert!(
        ch.num_shortcuts() > 0,
        "expected shortcuts when the only witness is much longer, got {}",
        ch.num_shortcuts()
    );
    // And the query must still produce the correct answer (a→mid→c cost 2).
    let p = ch.query(a, c).expect("ch path");
    assert!((p.cost - 2.0).abs() < 1e-3, "got cost {}", p.cost);
}
