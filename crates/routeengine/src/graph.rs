//! Graph in CSR-style storage. The hot loops in A*/Dijkstra read
//! neighbor edges by index range, which is cache-friendly and lets
//! us avoid pointer chasing.

use crate::spatial::LatLon;

pub type NodeId = u32;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoadType {
    Highway,
    Paved,
    Unpaved,
    Service,
}

#[derive(Debug, Clone, Copy)]
pub struct Edge {
    pub from: NodeId,
    pub to: NodeId,
    /// Base cost in metres. Constraints multiply this.
    pub distance_m: f32,
    pub road_type: RoadType,
    pub elevation_gain_m: f32,
    /// Opaque key into the weather grid (0 = no weather data).
    pub weather_id: u32,
}

#[derive(Debug, Clone)]
pub struct Graph {
    /// `nodes[i]` is the lat/lon of node id `i`.
    nodes: Vec<LatLon>,
    /// CSR row pointers: edges of node `i` are
    /// `edges[adj_start[i]..adj_start[i+1]]`.
    adj_start: Vec<u32>,
    edges: Vec<Edge>,
}

impl Graph {
    pub fn builder() -> GraphBuilder {
        GraphBuilder {
            nodes: Vec::new(),
            edges: Vec::new(),
        }
    }

    #[inline]
    pub fn num_nodes(&self) -> usize {
        self.nodes.len()
    }

    #[inline]
    pub fn num_edges(&self) -> usize {
        self.edges.len()
    }

    #[inline]
    pub fn coord(&self, n: NodeId) -> LatLon {
        self.nodes[n as usize]
    }

    /// Outgoing edges of `n` as a slice. O(1).
    #[inline]
    pub fn neighbors(&self, n: NodeId) -> &[Edge] {
        let lo = self.adj_start[n as usize] as usize;
        let hi = self.adj_start[n as usize + 1] as usize;
        &self.edges[lo..hi]
    }

    /// All node coordinates, in id order.
    pub fn coords(&self) -> &[LatLon] {
        &self.nodes
    }

    /// Iterator over every edge in the graph (any order).
    pub fn all_edges(&self) -> impl Iterator<Item = &Edge> {
        self.edges.iter()
    }
}

/// Builder for `Graph`. Edges may be added in any order; the build
/// step sorts and constructs the CSR layout.
pub struct GraphBuilder {
    nodes: Vec<LatLon>,
    edges: Vec<Edge>,
}

impl GraphBuilder {
    pub fn add_node(&mut self, ll: LatLon) -> NodeId {
        let id = self.nodes.len() as NodeId;
        self.nodes.push(ll);
        id
    }

    pub fn add_edge(&mut self, e: Edge) {
        // Sanity: 2³⁰ caps absurd ingest values that would overflow
        // the priority queue's float math after constraint scaling.
        debug_assert!(e.distance_m.is_finite() && e.distance_m < (1u32 << 30) as f32);
        self.edges.push(e);
    }

    /// Convenience: add `from→to` with a symmetric reverse edge.
    pub fn add_bidirectional(&mut self, e: Edge) {
        let rev = Edge {
            from: e.to,
            to: e.from,
            ..e
        };
        self.add_edge(e);
        self.add_edge(rev);
    }

    pub fn build(mut self) -> Graph {
        // Sort edges by `from` to enable the CSR row layout.
        self.edges.sort_by_key(|e| e.from);
        let n = self.nodes.len();
        let mut adj_start = vec![0u32; n + 1];
        for e in &self.edges {
            adj_start[e.from as usize + 1] += 1;
        }
        for i in 1..=n {
            adj_start[i] += adj_start[i - 1];
        }
        Graph {
            nodes: self.nodes,
            adj_start,
            edges: self.edges,
        }
    }
}
