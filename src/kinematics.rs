use cimvr_common::{nalgebra, Transform};
use cimvr_engine_interface::{pkg_namespace, prelude::*};
use nalgebra::{Isometry3, Vector3, Matrix4, Point3, Translation3, UnitQuaternion};
use serde::{Deserialize, Serialize};

/// Component for objects simulated with the kinematics system
#[derive(Serialize, Deserialize, Copy, Clone, Debug, PartialEq)]
pub struct KinematicPhysics {
    /// Velocity
    pub vel: Vector3<f32>,
    /// The mass of this object
    pub mass: f32,
}

impl KinematicPhysics {
    /// Create a new kinematic
    pub fn new(mass: f32) -> Self {
        Self {
            mass,
            vel: Vector3::zeros(),
        }
    }

    /// Apply a force to this object
    pub fn force(&mut self, f: Vector3<f32>) {
        self.vel += f / self.mass;
    }
}

pub fn simulate(query: &mut QueryResult, dt: f32) {
    for key in query.iter() {
        let kine = query.read::<KinematicPhysics>(key);
        query.modify::<Transform>(key, |t| t.pos += kine.vel * dt)
    }
}

pub fn gravity(query: &mut QueryResult, dt: f32, g: Vector3<f32>) {
    for key in query.iter() {
        query.modify::<KinematicPhysics>(key, |k| k.vel += dt * g);
    }
}

impl Component for KinematicPhysics {
    const ID: ComponentIdStatic = ComponentIdStatic {
        id: pkg_namespace!("KinematicPhysics"),
        size: 24,
    };
}
