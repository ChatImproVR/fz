use cimvr_common::nalgebra;
use cimvr_engine_interface::{pkg_namespace, prelude::*};
use nalgebra::{Isometry3, Vector3, Matrix4, Point3, Translation3, UnitQuaternion};
use serde::{Deserialize, Serialize};

/// Component for objects simulated with the kinematics system
#[derive(Serialize, Deserialize, Copy, Clone, Debug, PartialEq)]
pub struct KinematicPhysics {
    /// The mass of this object
    pub mass: f32,
    /// Velocity
    pub vel: Vector3<f32>,
    /// # Applied impluse next frame (force per unit time)
    ///
    /// **Q: Why don't we just use `vel`, and then add something like a
    /// `force` function instead of using impulse?**
    /// A: We want a global time step
    ///
    /// A: So every plugin gets the same current velocity, while keeping the ability to accelerate
    /// the object based on that (e.g. Lorentz force, drag)
    pub impulse: Vector3<f32>,
    // TODO: Angular impulse, momentum!
}

impl KinematicPhysics {
    /// Create a new kinematic
    pub fn new(mass: f32) -> Self {
        Self {
            mass,
            impulse: Vector3::zeros(),
            vel: Vector3::zeros(),
        }
    }

    /// Perform a time step of this kinematics component, re-setting its impulse to zero
    pub fn step(&mut self, dt: f32) {
        self.vel += self.impulse * dt / self.mass;
        self.impulse = Vector3::zeros();
    }
}

pub fn simulate(query: &mut QueryResult, dt: f32) {
    for key in query.iter() {
        query.modify::<KinematicPhysics>(key, |k| k.step(dt));
    }
}

impl Component for KinematicPhysics {
    const ID: ComponentIdStatic = ComponentIdStatic {
        id: pkg_namespace!("KinematicPhysics"),
        size: 16,
    };
}
