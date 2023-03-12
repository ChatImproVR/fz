use cimvr_common::{nalgebra, Transform};
use cimvr_engine_interface::{pkg_namespace, prelude::*};
use nalgebra::{Vector3, UnitQuaternion};
use serde::{Deserialize, Serialize};

/// Component for objects simulated with the kinematics system
#[derive(Serialize, Deserialize, Default, Copy, Clone, Debug, PartialEq)]
pub struct KinematicPhysics {
    /// Velocity
    pub vel: Vector3<f32>,
    /// The mass of this object
    pub mass: f32,
    /// Angular velocity
    pub ang_vel: Vector3<f32>,
    /// Moment of inertia
    pub moment: f32,
}

impl KinematicPhysics {
    /// Create a new kinematic
    pub fn new(mass: f32) -> Self {
        Self {
            mass,
            vel: Vector3::zeros(),
            ang_vel: Vector3::zeros(),
            // Assume I = MR^2 where R = 1m
            // TODO: More nuanced representation
            moment: mass,
        }
    }

    /// Apply a force to this object
    pub fn force(&mut self, f: Vector3<f32>) {
        self.vel += f / self.mass;
    }

    /// Apply a torque to this object
    pub fn torque(&mut self, t: Vector3<f32>) {
        self.ang_vel += t / self.moment;
    }
}

pub fn simulate(query: &mut QueryResult, dt: f32) {
    for key in query.iter() {
        let kine = query.read::<KinematicPhysics>(key);
        query.modify::<Transform>(key, |t| {
            t.pos += kine.vel * dt;
            t.orient = UnitQuaternion::from_scaled_axis(kine.ang_vel * dt) * t.orient;
        })
    }
}

pub fn gravity(query: &mut QueryResult, dt: f32, g: Vector3<f32>) {
    for key in query.iter() {
        query.modify::<KinematicPhysics>(key, |k| k.vel += dt * g);
    }
}

impl Component for KinematicPhysics {
    const ID: &'static str = pkg_namespace!("KinematicPhysics");
}
