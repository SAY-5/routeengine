//! A* with a Haversine admissible heuristic. The first pop of `dst`
//! is the optimal path — this is the standard A* correctness theorem
//! when h ≤ true cost. h(n) = great-circle(n, dst) which is always
//! ≤ the true accumulated edge cost since constraints only ever
//! increase the cost.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::constraints::{ConstraintSet, SpatialContext};
use crate::dijkstra::{reconstruct, Path};
use crate::graph::{Graph, NodeId};

#[derive(Debug, Clone, Copy, Default)]
pub struct AstarStats {
    pub nodes_popped: u32,
    pub edges_relaxed: u32,
}

#[derive(Copy, Clone, PartialEq)]
struct HeapEntry {
    f: f32, // priority = g + h
    g: f32, // distance from src
    node: NodeId,
}

impl Eq for HeapEntry {}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f
            .partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| other.node.cmp(&self.node))
    }
}

pub fn astar(
    g: &Graph,
    src: NodeId,
    dst: NodeId,
    cs: &ConstraintSet,
    ctx: &SpatialContext,
) -> Option<(Path, AstarStats)> {
    let n = g.num_nodes();
    if (src as usize) >= n || (dst as usize) >= n {
        return None;
    }
    let goal_ll = g.coord(dst);
    let mut dist = vec![f32::INFINITY; n];
    let mut prev = vec![u32::MAX; n];
    let mut stats = AstarStats::default();
    dist[src as usize] = 0.0;

    let h = |from: NodeId| g.coord(from).haversine_m(&goal_ll);

    let mut heap = BinaryHeap::new();
    heap.push(HeapEntry {
        f: h(src),
        g: 0.0,
        node: src,
    });

    while let Some(HeapEntry { g: gcost, node, .. }) = heap.pop() {
        stats.nodes_popped += 1;
        if node == dst {
            return Some((reconstruct(prev, src, dst, gcost), stats));
        }
        if gcost > dist[node as usize] {
            continue; // stale entry
        }
        for e in g.neighbors(node) {
            stats.edges_relaxed += 1;
            let new_g = gcost + cs.cost(e, ctx);
            if new_g < dist[e.to as usize] {
                dist[e.to as usize] = new_g;
                prev[e.to as usize] = node;
                heap.push(HeapEntry {
                    f: new_g + h(e.to),
                    g: new_g,
                    node: e.to,
                });
            }
        }
    }
    None
}
