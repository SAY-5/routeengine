//! Spatial index for "nodes near (lat, lon)" lookups during A*'s
//! goal-distance heuristic and for ingest joins (snap a point to
//! the nearest node). Production: PostGIS GiST. v1: a coarse-grid
//! bucket. The trait abstracts the swap.

use crate::graph::NodeId;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LatLon {
    pub lat: f32,
    pub lon: f32,
}

impl LatLon {
    pub const fn new(lat: f32, lon: f32) -> Self {
        Self { lat, lon }
    }

    /// Great-circle distance in metres (Haversine).
    /// Stable to ~10cm at city scales which is well below the
    /// granularity of edge `distance_m` values.
    pub fn haversine_m(&self, other: &LatLon) -> f32 {
        const R_M: f32 = 6_371_000.0;
        let to_rad = |d: f32| d * std::f32::consts::PI / 180.0;
        let lat1 = to_rad(self.lat);
        let lat2 = to_rad(other.lat);
        let d_lat = lat2 - lat1;
        let d_lon = to_rad(other.lon - self.lon);
        let a = (d_lat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (d_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().asin();
        R_M * c
    }
}

pub trait SpatialIndex {
    /// Nodes within `radius_m` of `point`. May over-return; caller
    /// is expected to filter by exact distance.
    fn nodes_near(&self, point: LatLon, radius_m: f32) -> Vec<NodeId>;

    /// Single nearest neighbor, ties broken by id.
    fn nearest(&self, point: LatLon) -> Option<NodeId>;
}

/// Uniform grid bucket. Each cell is `cell_deg` × `cell_deg` in
/// lat/lon space; nodes are placed by floor(lat/cell), floor(lon/cell).
/// O(1) lookup; works fine for the 50K-node demo dataset.
pub struct Grid {
    cell_deg: f32,
    coords: Vec<LatLon>,
    cells: std::collections::HashMap<(i32, i32), Vec<NodeId>>,
}

impl Grid {
    pub fn build(coords: &[LatLon], cell_deg: f32) -> Self {
        assert!(cell_deg > 0.0);
        let mut g = Self {
            cell_deg,
            coords: coords.to_vec(),
            cells: std::collections::HashMap::new(),
        };
        for (i, c) in coords.iter().enumerate() {
            g.cells.entry(g.cell_of(*c)).or_default().push(i as NodeId);
        }
        g
    }

    fn cell_of(&self, p: LatLon) -> (i32, i32) {
        (
            (p.lat / self.cell_deg).floor() as i32,
            (p.lon / self.cell_deg).floor() as i32,
        )
    }

    /// Cells covering a `radius_m` query around `p`. We over-cover
    /// (1° ≈ 111km at the equator) which is fine for a coarse
    /// pre-filter — nodes_near() does the exact distance check.
    fn cells_around(&self, p: LatLon, radius_m: f32) -> Vec<(i32, i32)> {
        let radius_deg = (radius_m / 111_000.0) + self.cell_deg;
        let (clat, clon) = self.cell_of(p);
        let span = (radius_deg / self.cell_deg).ceil() as i32;
        let mut out = Vec::with_capacity(((2 * span + 1) * (2 * span + 1)) as usize);
        for dlat in -span..=span {
            for dlon in -span..=span {
                out.push((clat + dlat, clon + dlon));
            }
        }
        out
    }
}

impl SpatialIndex for Grid {
    fn nodes_near(&self, point: LatLon, radius_m: f32) -> Vec<NodeId> {
        let mut out = Vec::new();
        for cell in self.cells_around(point, radius_m) {
            if let Some(ids) = self.cells.get(&cell) {
                for id in ids {
                    if self.coords[*id as usize].haversine_m(&point) <= radius_m {
                        out.push(*id);
                    }
                }
            }
        }
        out
    }

    fn nearest(&self, point: LatLon) -> Option<NodeId> {
        // Expand the search radius until we find at least one match.
        let mut radius = 1_000.0_f32;
        while radius < 1_000_000.0 {
            let candidates = self.nodes_near(point, radius);
            if !candidates.is_empty() {
                return candidates.into_iter().min_by(|a, b| {
                    let da = self.coords[*a as usize].haversine_m(&point);
                    let db = self.coords[*b as usize].haversine_m(&point);
                    da.partial_cmp(&db)
                        .unwrap_or(std::cmp::Ordering::Equal)
                        .then_with(|| a.cmp(b))
                });
            }
            radius *= 4.0;
        }
        None
    }
}
