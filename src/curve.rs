use cimvr_common::{
    glam::{Mat3, Quat, Vec3},
    render::Mesh,
    Transform,
};

/// Extract position and orientation data from the specially designed path mesh
pub fn path_mesh_to_transforms(mesh: &Mesh) -> Vec<Transform> {
    let mut transforms = vec![];

    for axes in mesh.vertices.chunks_exact(4) {
        let origin = Vec3::from(axes[1].pos);

        let to_vect = |i: usize| (Vec3::from(axes[i].pos) - origin);

        let x = -to_vect(0);
        let y = to_vect(2);
        let z = -to_vect(3);

        let mat = Mat3::from_cols(-x, y, z);
        let orient = Quat::from_mat3(&mat);

        transforms.push(Transform {
            pos: origin,
            orient,
        })
    }

    transforms
}

pub struct Curve {
    pub ctrlps: Vec<Transform>,
}

impl Curve {
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
        self.ctrlps[behind].lerp_slerp(&self.ctrlps[in_front], t.fract())
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
