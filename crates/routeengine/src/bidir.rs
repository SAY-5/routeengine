//! Bidirectional A*. Searches forward from `src` and backward from
//! `dst` concurrently; returns when the two frontiers meet.
//!
//! The classic speedup is roughly √(N/2) on a uniform graph — the
//! single-direction A* explores a circle of radius `d` around the
//! source; bidirectional explores two half-circles whose total area
//! is ~half. v1 (this file) is the forward+reverse Dijkstra-style
//! variant with an admissible Haversine heuristic on each side and
//! the standard "stop when top-of-queue + opposite-best ≥ best path"
//! termination condition.
//!
//! Reverse search requires reverse adjacency. We construct it once
//! per call from the forward graph; for repeated queries the caller
//! should reuse a precomputed reverse via `Graph::reverse_adjacency`.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::constraints::{ConstraintSet, SpatialContext};
use crate::dijkstra::Path;
use crate::graph::{Edge, Graph, NodeId};

#[derive(Debug, Clone, Copy, Default)]
pub struct BidirStats {
    pub forward_pops: u32,
    pub backward_pops: u32,
    pub edges_relaxed: u32,
}

#[derive(Copy, Clone, PartialEq)]
struct HeapEntry {
    f: f32,
    g: f32,
    node: NodeId,
}

impl Eq for HeapEntry {}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for HeapEntry {
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| o.node.cmp(&self.node))
    }
}

/// Reverse adjacency for bidirectional search.
pub fn reverse_adjacency(g: &Graph) -> Vec<Vec<Edge>> {
    let mut rev: Vec<Vec<Edge>> = (0..g.num_nodes()).map(|_| Vec::new()).collect();
    for e in g.all_edges() {
        rev[e.to as usize].push(Edge {
            from: e.to,
            to: e.from,
            distance_m: e.distance_m,
            road_type: e.road_type,
            elevation_gain_m: e.elevation_gain_m,
            weather_id: e.weather_id,
        });
    }
    rev
}

/// Bidirectional A*. Returns the shortest path under `cs` + `ctx`
/// or `None` if `dst` is unreachable from `src`.
pub fn bidir_astar(
    g: &Graph,
    rev: &[Vec<Edge>],
    src: NodeId,
    dst: NodeId,
    cs: &ConstraintSet,
    ctx: &SpatialContext,
) -> Option<(Path, BidirStats)> {
    let n = g.num_nodes();
    if (src as usize) >= n || (dst as usize) >= n {
        return None;
    }
    if src == dst {
        return Some((
            Path {
                nodes: vec![src],
                cost: 0.0,
            },
            BidirStats::default(),
        ));
    }

    let src_ll = g.coord(src);
    let dst_ll = g.coord(dst);
    let hf = |from: NodeId| g.coord(from).haversine_m(&dst_ll);
    let hb = |from: NodeId| g.coord(from).haversine_m(&src_ll);

    let mut df = vec![f32::INFINITY; n];
    df[src as usize] = 0.0;
    let mut db = vec![f32::INFINITY; n];
    db[dst as usize] = 0.0;
    let mut pf: Vec<u32> = vec![u32::MAX; n];
    let mut pb: Vec<u32> = vec![u32::MAX; n];
    let mut hf_q = BinaryHeap::new();
    let mut hb_q = BinaryHeap::new();
    hf_q.push(HeapEntry {
        f: hf(src),
        g: 0.0,
        node: src,
    });
    hb_q.push(HeapEntry {
        f: hb(dst),
        g: 0.0,
        node: dst,
    });

    let mut best = f32::INFINITY;
    let mut meet = u32::MAX;
    let mut stats = BidirStats::default();

    while !hf_q.is_empty() && !hb_q.is_empty() {
        // Termination: when the sum of the two frontiers' top
        // priorities is ≥ best, the best path can't be improved.
        let top_f = hf_q.peek().map(|e| e.f).unwrap_or(f32::INFINITY);
        let top_b = hb_q.peek().map(|e| e.f).unwrap_or(f32::INFINITY);
        if top_f + top_b >= best {
            break;
        }

        // Alternate by smaller queue, a tiny optimization for
        // unbalanced graphs.
        if hf_q.len() <= hb_q.len() {
            if let Some(e) = hf_q.pop() {
                stats.forward_pops += 1;
                if e.g > df[e.node as usize] {
                    continue;
                }
                for ed in g.neighbors(e.node) {
                    stats.edges_relaxed += 1;
                    let ng = e.g + cs.cost(ed, ctx);
                    if ng < df[ed.to as usize] {
                        df[ed.to as usize] = ng;
                        pf[ed.to as usize] = e.node;
                        hf_q.push(HeapEntry {
                            f: ng + hf(ed.to),
                            g: ng,
                            node: ed.to,
                        });
                        if db[ed.to as usize].is_finite() {
                            let total = ng + db[ed.to as usize];
                            if total < best {
                                best = total;
                                meet = ed.to;
                            }
                        }
                    }
                }
            }
        } else if let Some(e) = hb_q.pop() {
            stats.backward_pops += 1;
            if e.g > db[e.node as usize] {
                continue;
            }
            for ed in &rev[e.node as usize] {
                stats.edges_relaxed += 1;
                let ng = e.g + cs.cost(ed, ctx);
                if ng < db[ed.to as usize] {
                    db[ed.to as usize] = ng;
                    pb[ed.to as usize] = e.node;
                    hb_q.push(HeapEntry {
                        f: ng + hb(ed.to),
                        g: ng,
                        node: ed.to,
                    });
                    if df[ed.to as usize].is_finite() {
                        let total = df[ed.to as usize] + ng;
                        if total < best {
                            best = total;
                            meet = ed.to;
                        }
                    }
                }
            }
        }
    }

    if meet == u32::MAX || !best.is_finite() {
        return None;
    }
    // Reconstruct: src ← ... ← meet → ... → dst.
    let mut nodes = Vec::new();
    let mut cur = meet;
    nodes.push(cur);
    while cur != src {
        let p = pf[cur as usize];
        if p == u32::MAX {
            break;
        }
        cur = p;
        nodes.push(cur);
    }
    nodes.reverse();
    cur = meet;
    while cur != dst {
        let p = pb[cur as usize];
        if p == u32::MAX {
            break;
        }
        cur = p;
        nodes.push(cur);
    }
    Some((Path { nodes, cost: best }, stats))
}
