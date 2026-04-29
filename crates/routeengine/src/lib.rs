//! Geospatial routing engine. See `ARCHITECTURE.md` for the design.
//!
//! Public surface:
//!   - [`Graph`] — CSR-style adjacency + node coordinates.
//!   - [`astar`] — A* with admissible Haversine heuristic.
//!   - [`dijkstra`] — baseline used in tests and quality benchmarks.
//!   - [`SpatialIndex`] / [`Grid`] — neighbor lookup by lat/lon.
//!   - [`ConstraintSet`] — stacked multiplicative edge cost.

#![deny(unsafe_code)]
#![warn(rust_2018_idioms)]

pub mod astar;
pub mod bidir;
pub mod constraints;
pub mod dijkstra;
pub mod graph;
pub mod spatial;

pub use astar::{astar, AstarStats};
pub use bidir::{bidir_astar, reverse_adjacency, BidirStats};
pub use constraints::{ConstraintSet, EdgeCostFn, SpatialContext};
pub use dijkstra::dijkstra;
pub use graph::{Edge, Graph, NodeId, RoadType};
pub use spatial::{Grid, LatLon, SpatialIndex};
