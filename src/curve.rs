use cimvr_common::{glam::Vec3, Transform};

pub struct Path {
    pub ctrlps: Vec<Transform>,
}

impl Path {
    pub fn new(ctrlps: Vec<Transform>) -> Self {
        Self { ctrlps }
    }

    /// Get the indices of the transforms behind and in front of the given t value respectively
    pub fn index(&self, t: f32) -> (usize, usize) {
        // Index part of path position
        let i = t.floor() as usize;
        let len = self.ctrlps.len();

        let behind = i % len;
        let in_front = (i + 1) % len;

        (behind, in_front)
    }

    /// Linearly interpolate along the path
    pub fn lerp(&self, t: f32) -> Transform {
        let (behind, in_front) = self.index(t);
        trans_lerp_slerp(self.ctrlps[behind], self.ctrlps[in_front], t.fract())
    }

    /// Estimates the nearest curve index `t` to the given 3D position.
    /// Increasing iterations increases accuracy at the cost of performance
    pub fn nearest_ctrlp(&self, pt: Vec3) -> usize {
        let mut smallest_idx = 0;
        let mut smallest_dist = f32::MAX;

        for (idx, ctrlpt) in self.ctrlps.iter().enumerate() {
            let dist = (ctrlpt.pos - pt).length();
            if dist < smallest_dist {
                smallest_dist = dist;
                smallest_idx = idx;
            }
        }

        smallest_idx
    }
}

/// Interpolate between transforms
pub fn trans_lerp_slerp(a: Transform, b: Transform, t: f32) -> Transform {
    Transform {
        pos: a.pos.lerp(b.pos, t),
        orient: a.orient.slerp(b.orient, t),
    }
}
