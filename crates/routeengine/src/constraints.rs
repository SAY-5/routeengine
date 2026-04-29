//! Constraint composition. Each constraint is a function returning
//! a multiplier on an edge's base distance. The total edge cost is
//! the *product* of all multipliers — multiplicative composition
//! preserves relative ordering under linear scaling and avoids the
//! "constraint dominates everything" failure of additive composition.

use std::collections::HashMap;

use crate::graph::{Edge, RoadType};

/// What a constraint sees beyond the edge itself: the weather grid,
/// elevation lookups, etc. Production deployments add to this; v1
/// gives the constraint a small key/value bag.
#[derive(Default, Clone)]
pub struct SpatialContext {
    /// `weather_id` → severity in [0, 1]. 0 = clear, 1 = severe.
    pub weather: HashMap<u32, f32>,
    /// Tunable knob, range [0, 1]. 0 disables weather; 1 gives the
    /// max storm-avoidance multiplier.
    pub weather_strength: f32,
    /// Tunable knob for elevation penalty, range [0, ∞). 0 disables.
    pub elevation_strength: f32,
}

pub type EdgeCostFn = fn(&Edge, &SpatialContext) -> f32;

#[derive(Default, Clone)]
pub struct ConstraintSet {
    fns: Vec<EdgeCostFn>,
}

impl ConstraintSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(mut self, f: EdgeCostFn) -> Self {
        self.fns.push(f);
        self
    }

    /// Cost of traversing `e` under this set's constraints.
    /// Always ≥ `e.distance_m` because constraints can only ever
    /// raise the cost (multipliers are clamped ≥ 1 by convention).
    pub fn cost(&self, e: &Edge, ctx: &SpatialContext) -> f32 {
        let mut mult = 1.0f32;
        for f in &self.fns {
            mult *= f(e, ctx);
        }
        e.distance_m * mult.max(1e-6)
    }
}

// --- shipped constraints -----------------------------------------

pub fn storm_avoid(e: &Edge, ctx: &SpatialContext) -> f32 {
    if e.weather_id == 0 || ctx.weather_strength <= 0.0 {
        return 1.0;
    }
    let severity = ctx.weather.get(&e.weather_id).copied().unwrap_or(0.0);
    // Severity 1.0 with strength 1.0 → 5x penalty. Tunable.
    1.0 + 4.0 * severity * ctx.weather_strength
}

pub fn elevation_penalty(e: &Edge, ctx: &SpatialContext) -> f32 {
    if ctx.elevation_strength <= 0.0 {
        return 1.0;
    }
    1.0 + (e.elevation_gain_m.max(0.0) / 100.0) * ctx.elevation_strength
}

/// A small road-type preference: paved is faster, unpaved is slower.
pub fn road_type_bias(e: &Edge, _ctx: &SpatialContext) -> f32 {
    match e.road_type {
        RoadType::Highway => 0.85,
        RoadType::Paved => 1.0,
        RoadType::Unpaved => 1.4,
        RoadType::Service => 1.2,
    }
}

/// All shipped constraints, in the order most callers want.
pub fn default_set() -> ConstraintSet {
    ConstraintSet::new()
        .push(storm_avoid)
        .push(elevation_penalty)
        .push(road_type_bias)
}
