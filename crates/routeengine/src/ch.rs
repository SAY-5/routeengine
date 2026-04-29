//! Contraction Hierarchies (basic). Precompute a node-ordered
//! contraction: for each node n, in priority order, add a shortcut
//! edge u→v whenever the shortest u→n→v path is also the shortest
//! u→v path through any node already contracted. Queries become
//! bidirectional searches over the augmented graph that only relax
//! edges going to higher-priority nodes — order-of-magnitude
//! speedups on large graphs.
//!
//! v3 ships a small, correct CH that's optimized for clarity over
//! peak speed: the witness-search uses a bounded local Dijkstra
//! with a hop limit; the priority order is the simplest standard
//! (edge-difference). Production CH builders use more aggressive
//! ordering (importance metrics, lazy updates) but the QUERY path
//! is identical.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::dijkstra::Path;
use crate::graph::{Graph, NodeId};

/// Augmented graph: original edges plus precomputed shortcuts.
/// Each shortcut carries the via-node so the path can be unpacked.
pub struct CH {
    /// node_id → priority rank in the contraction. Higher rank =
    /// contracted later. The query only relaxes edges going to
    /// strictly higher rank nodes.
    rank: Vec<u32>,
    /// Forward + reverse adjacency in (to, cost, via) form. via=None
    /// is an original edge; via=Some(n) is a shortcut representing
    /// the optimal path through n that we pre-discovered.
    fwd: Vec<Vec<ShortcutEdge>>,
    rev: Vec<Vec<ShortcutEdge>>,
}

#[derive(Debug, Clone, Copy)]
struct ShortcutEdge {
    to: NodeId,
    cost: f32,
    via: Option<NodeId>,
}

#[derive(Copy, Clone, PartialEq)]
struct HE {
    cost: f32,
    node: NodeId,
}
impl Eq for HE {}
impl PartialOrd for HE {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for HE {
    fn cmp(&self, o: &Self) -> Ordering {
        o.cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
            .then_with(|| o.node.cmp(&self.node))
    }
}

impl CH {
    /// Build a CH from `g` using static base-distance edge costs.
    /// Constraint-aware queries are out of scope for v3 — CH
    /// preprocessing fixes edge weights at build time. Pair this
    /// with a no-constraint `ConstraintSet` for now; tour-style
    /// constraint sets reuse v1's A*.
    pub fn build(g: &Graph) -> CH {
        let n = g.num_nodes();
        // Initial fwd/rev are the original edges.
        let mut fwd: Vec<Vec<ShortcutEdge>> = (0..n).map(|_| Vec::new()).collect();
        let mut rev: Vec<Vec<ShortcutEdge>> = (0..n).map(|_| Vec::new()).collect();
        for e in g.all_edges() {
            fwd[e.from as usize].push(ShortcutEdge {
                to: e.to,
                cost: e.distance_m,
                via: None,
            });
            rev[e.to as usize].push(ShortcutEdge {
                to: e.from,
                cost: e.distance_m,
                via: None,
            });
        }

        // Compute a contraction order via edge-difference.
        // edge_difference = (added_shortcuts) − (removed_edges_at_n).
        // We compute it greedily: visit nodes in ascending degree,
        // then on each contraction recompute affected nodes' priority.
        let mut contracted = vec![false; n];
        let mut rank = vec![0u32; n];
        let mut next_rank = 0u32;

        // Precompute initial degree-based priority. Smaller is
        // better. For demo correctness we don't lazily update; we
        // just contract in a stable degree order.
        let mut order: Vec<NodeId> = (0..n as NodeId).collect();
        order.sort_by_key(|&v| (fwd[v as usize].len() + rev[v as usize].len()) as u32);

        for v in order {
            // For every (u → v → w) where u, w ∉ contracted, add a
            // shortcut u → w with cost = c(u,v) + c(v,w) IF no
            // shorter path u → w exists in the current graph
            // ignoring v (witness search).
            let in_edges: Vec<ShortcutEdge> = rev[v as usize].clone();
            let out_edges: Vec<ShortcutEdge> = fwd[v as usize].clone();
            for ie in &in_edges {
                if contracted[ie.to as usize] {
                    continue;
                }
                let u = ie.to;
                for oe in &out_edges {
                    if contracted[oe.to as usize] || u == oe.to {
                        continue;
                    }
                    let w = oe.to;
                    let direct = ie.cost + oe.cost;
                    // Witness search: does a path u → w exist that
                    // avoids v with cost ≤ direct? Bounded Dijkstra
                    // with a small hop limit.
                    if witness_no_shorter(u, w, v, direct, &fwd, &contracted, 6) {
                        // No witness — must add a shortcut.
                        fwd[u as usize].push(ShortcutEdge {
                            to: w,
                            cost: direct,
                            via: Some(v),
                        });
                        rev[w as usize].push(ShortcutEdge {
                            to: u,
                            cost: direct,
                            via: Some(v),
                        });
                    }
                }
            }
            contracted[v as usize] = true;
            rank[v as usize] = next_rank;
            next_rank += 1;
        }

        CH { rank, fwd, rev }
    }

    /// CH query — bidirectional Dijkstra restricted to upward edges.
    pub fn query(&self, src: NodeId, dst: NodeId) -> Option<Path> {
        let n = self.fwd.len();
        if (src as usize) >= n || (dst as usize) >= n {
            return None;
        }
        if src == dst {
            return Some(Path {
                nodes: vec![src],
                cost: 0.0,
            });
        }
        let mut df = vec![f32::INFINITY; n];
        df[src as usize] = 0.0;
        let mut db = vec![f32::INFINITY; n];
        db[dst as usize] = 0.0;
        let mut pf = vec![u32::MAX; n];
        let mut pb = vec![u32::MAX; n];
        let mut hf_q = BinaryHeap::new();
        let mut hb_q = BinaryHeap::new();
        hf_q.push(HE {
            cost: 0.0,
            node: src,
        });
        hb_q.push(HE {
            cost: 0.0,
            node: dst,
        });

        let mut best = f32::INFINITY;
        let mut meet = u32::MAX;

        while !hf_q.is_empty() || !hb_q.is_empty() {
            let top_f = hf_q.peek().map(|e| e.cost).unwrap_or(f32::INFINITY);
            let top_b = hb_q.peek().map(|e| e.cost).unwrap_or(f32::INFINITY);
            if top_f.min(top_b) >= best {
                break;
            }
            if top_f <= top_b {
                let e = hf_q.pop().unwrap();
                if e.cost > df[e.node as usize] {
                    continue;
                }
                if db[e.node as usize].is_finite() {
                    let total = e.cost + db[e.node as usize];
                    if total < best {
                        best = total;
                        meet = e.node;
                    }
                }
                for ed in &self.fwd[e.node as usize] {
                    // Only relax UPWARD: rank[next] > rank[current].
                    if self.rank[ed.to as usize] <= self.rank[e.node as usize] {
                        continue;
                    }
                    let ng = e.cost + ed.cost;
                    if ng < df[ed.to as usize] {
                        df[ed.to as usize] = ng;
                        pf[ed.to as usize] = e.node;
                        hf_q.push(HE {
                            cost: ng,
                            node: ed.to,
                        });
                    }
                }
            } else {
                let e = hb_q.pop().unwrap();
                if e.cost > db[e.node as usize] {
                    continue;
                }
                if df[e.node as usize].is_finite() {
                    let total = df[e.node as usize] + e.cost;
                    if total < best {
                        best = total;
                        meet = e.node;
                    }
                }
                for ed in &self.rev[e.node as usize] {
                    if self.rank[ed.to as usize] <= self.rank[e.node as usize] {
                        continue;
                    }
                    let ng = e.cost + ed.cost;
                    if ng < db[ed.to as usize] {
                        db[ed.to as usize] = ng;
                        pb[ed.to as usize] = e.node;
                        hb_q.push(HE {
                            cost: ng,
                            node: ed.to,
                        });
                    }
                }
            }
        }
        if meet == u32::MAX || !best.is_finite() {
            return None;
        }
        // Path reconstruction. Compressed (via shortcut nodes
        // collapsed) for now — the unpack-shortcuts step is a
        // documented v3.x extension.
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
        Some(Path { nodes, cost: best })
    }

    pub fn num_shortcuts(&self) -> usize {
        self.fwd
            .iter()
            .map(|v| v.iter().filter(|e| e.via.is_some()).count())
            .sum()
    }

    /// v4: unpack shortcut edges along the compressed CH path so the
    /// caller sees the original sequence of nodes the route actually
    /// traverses. CH stores each shortcut as `(u → w via v)`; we
    /// recursively replace `u → w` with `u → v → w` and unpack each
    /// half until every edge is an original (via == None).
    ///
    /// The unpacked path is what a user-facing turn-by-turn renderer
    /// or traffic visualizer needs. The compressed path is what the
    /// query path emits because it's faster to construct; v4 makes
    /// the trade-off explicit at the API boundary.
    pub fn unpack_path(&self, compressed: &Path) -> Path {
        if compressed.nodes.len() < 2 {
            return compressed.clone();
        }
        let mut out = Vec::with_capacity(compressed.nodes.len());
        out.push(compressed.nodes[0]);
        for w in compressed.nodes.windows(2) {
            self.unpack_edge(w[0], w[1], &mut out);
        }
        Path {
            nodes: out,
            cost: compressed.cost,
        }
    }

    /// Recursively walk shortcut bookkeeping. We look up the edge
    /// `u → v` in fwd; if its `via` is set, it's a shortcut and we
    /// recurse on each half. Otherwise it's an original edge and we
    /// just append `v`.
    fn unpack_edge(&self, u: NodeId, v: NodeId, out: &mut Vec<NodeId>) {
        let edges = &self.fwd[u as usize];
        // Pick the cheapest matching edge to `v` — there can be
        // duplicates (original + shortcut) and we want the canonical
        // representation (a shortcut edge points at the via node).
        let mut best: Option<&ShortcutEdge> = None;
        for e in edges {
            if e.to == v {
                match best {
                    None => best = Some(e),
                    Some(b) => {
                        if e.cost < b.cost {
                            best = Some(e);
                        }
                    }
                }
            }
        }
        match best {
            None => {
                // No direct edge — happens only for paths that go
                // backwards through the augmented graph. The CH
                // query never produces these; if we hit one, fall
                // through to `v` so the path stays a valid sequence.
                out.push(v);
            }
            Some(e) => match e.via {
                None => out.push(v),
                Some(mid) => {
                    self.unpack_edge(u, mid, out);
                    self.unpack_edge(mid, v, out);
                }
            },
        }
    }
}

fn witness_no_shorter(
    u: NodeId,
    w: NodeId,
    avoid: NodeId,
    target_cost: f32,
    fwd: &[Vec<ShortcutEdge>],
    contracted: &[bool],
    hop_limit: u32,
) -> bool {
    // Bounded Dijkstra from u; succeeds (== "no witness exists") if
    // we cannot reach w with cost ≤ target_cost while avoiding `avoid`
    // and contracted nodes. Returns `true` when the shortcut IS needed.
    let mut dist = std::collections::HashMap::new();
    dist.insert(u, (0.0f32, 0u32));
    let mut heap = BinaryHeap::new();
    heap.push(HE { cost: 0.0, node: u });

    while let Some(e) = heap.pop() {
        if e.node == w {
            return e.cost > target_cost; // witness ≤ target → not needed
        }
        if e.cost > target_cost {
            continue;
        }
        let (cur_cost, cur_hops) = dist.get(&e.node).copied().unwrap_or((f32::INFINITY, 0));
        if e.cost > cur_cost {
            continue;
        }
        if cur_hops >= hop_limit {
            continue;
        }
        for ed in &fwd[e.node as usize] {
            if ed.to == avoid || contracted[ed.to as usize] {
                continue;
            }
            let ng = e.cost + ed.cost;
            if ng > target_cost {
                continue;
            }
            let entry = dist.entry(ed.to).or_insert((f32::INFINITY, 0));
            if ng < entry.0 {
                *entry = (ng, cur_hops + 1);
                heap.push(HE {
                    cost: ng,
                    node: ed.to,
                });
            }
        }
    }
    // Never reached w within budget → no witness → shortcut needed.
    true
}

// Small re-export to keep the test file's import surface clean.
pub use crate::graph::Graph as CHGraph;
