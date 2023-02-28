use cimvr_common::{
    nalgebra::{Isometry3, Point3},
    Transform,
};

pub struct Path {
    ctrlps: Vec<Transform>,
}

impl Path {
    pub fn new(ctrlps: Vec<Transform>) -> Self {
        Self { ctrlps }
    }

    pub fn index(&self, t: f32) -> (usize, usize) {
        // Index part of path position
        let i = t.floor() as usize;
        let len = self.ctrlps.len();

        let behind = i % len;
        let in_front = (i + 1) % len;

        (behind, in_front)
    }

    pub fn lerp(&self, t: f32) -> Transform {
        let (behind, in_front) = self.index(t);
        trans_lerp_slerp(self.ctrlps[behind], self.ctrlps[in_front], t.fract())
    }

    /// Estimates the nearest curve index `t` to the given 3D position.
    /// Increasing iterations increases accuracy at the cost of performance
    pub fn nearest_t(&self, pt: Point3<f32>, _iters: usize) -> f32 {
        let mut smallest_idx = 0;
        let mut smallest_dist = f32::MAX;

        for (idx, ctrlpt) in self.ctrlps.iter().enumerate() {
            let dist = (ctrlpt.pos - pt).magnitude();
            if dist < smallest_dist {
                smallest_dist = dist;
                smallest_idx = idx;
            }
        }

        // TODO: Binary search refinement
        smallest_idx as f32
    }
}

/// Interpolate between transforms
pub fn trans_lerp_slerp(a: Transform, b: Transform, t: f32) -> Transform {
    let a: Isometry3<f32> = a.into();
    let b: Isometry3<f32> = b.into();
    a.lerp_slerp(&b, t).into()
}
