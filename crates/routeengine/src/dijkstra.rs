//! Dijkstra shortest path. Used as ground truth in the quality
//! benchmark — A*'s output must equal Dijkstra's on every query.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::constraints::{ConstraintSet, SpatialContext};
use crate::graph::{Graph, NodeId};

#[derive(Debug, Clone)]
pub struct Path {
    pub nodes: Vec<NodeId>,
    pub cost: f32,
}

#[derive(Copy, Clone, PartialEq)]
struct HeapEntry {
    cost: f32,
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
        // BinaryHeap is a max-heap; reverse so smaller cost = higher priority.
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
            .then_with(|| other.node.cmp(&self.node))
    }
}

/// Returns the optimal path from `src` to `dst` under `cs`+`ctx`,
/// or `None` if `dst` is unreachable.
pub fn dijkstra(
    g: &Graph,
    src: NodeId,
    dst: NodeId,
    cs: &ConstraintSet,
    ctx: &SpatialContext,
) -> Option<Path> {
    let n = g.num_nodes();
    if (src as usize) >= n || (dst as usize) >= n {
        return None;
    }
    let mut dist = vec![f32::INFINITY; n];
    let mut prev = vec![u32::MAX; n];
    dist[src as usize] = 0.0;
    let mut heap = BinaryHeap::new();
    heap.push(HeapEntry {
        cost: 0.0,
        node: src,
    });

    while let Some(HeapEntry { cost, node }) = heap.pop() {
        if node == dst {
            return Some(reconstruct(prev, src, dst, dist[dst as usize]));
        }
        if cost > dist[node as usize] {
            continue;
        }
        for e in g.neighbors(node) {
            let new_cost = cost + cs.cost(e, ctx);
            if new_cost < dist[e.to as usize] {
                dist[e.to as usize] = new_cost;
                prev[e.to as usize] = node;
                heap.push(HeapEntry {
                    cost: new_cost,
                    node: e.to,
                });
            }
        }
    }
    None
}

pub(crate) fn reconstruct(prev: Vec<u32>, src: NodeId, dst: NodeId, cost: f32) -> Path {
    let mut nodes = Vec::new();
    let mut cur = dst;
    nodes.push(cur);
    while cur != src {
        let p = prev[cur as usize];
        if p == u32::MAX {
            // Disconnected — shouldn't happen since we exited via the
            // dst-pop branch, but guard anyway.
            break;
        }
        cur = p;
        nodes.push(cur);
    }
    nodes.reverse();
    Path { nodes, cost }
}
