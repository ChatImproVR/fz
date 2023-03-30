use std::f32::consts::PI;

use cimvr_common::{
    glam::{EulerRot, Quat, Vec3},
    Transform,
};

use crate::{curve::Curve, kinematics::KinematicPhysics, InputAbstraction, ShipCharacteristics};

pub fn ship_controller(
    dt: f32,
    ship: ShipCharacteristics,
    input: InputAbstraction,
    path: &Curve,
    tf: &mut Transform,
    kt: &mut KinematicPhysics,
) {
    // Calculate position within the course
    let nearest_ctrlp_idx = path.nearest_ctrlp(tf.pos);
    let nearest_ctrlp = path.ctrlps[nearest_ctrlp_idx];
    let nearest_iso: Transform = nearest_ctrlp.into();
    let tf_iso: Transform = tf.clone().into();
    let path_local_space = nearest_iso.inverse() * tf_iso;

    // Collision detection
    const TRACK_WIDTH: f32 = 32.;
    const TRACK_HEIGHT: f32 = 10.;
    const TRACK_LENGTH: f32 = 10.;
    let z_bound = path_local_space.pos.z.abs() > TRACK_WIDTH / 2.;
    let y_bound = path_local_space.pos.y.abs() > TRACK_HEIGHT / 2.;
    if z_bound || y_bound {
        *tf = nearest_ctrlp;
        kt.ang_vel = Vec3::ZERO;
        kt.vel = Vec3::ZERO;
    }

    // Force controls
    let throttle_deadzone = 0.1;
    let force_live = input.throttle.abs() > throttle_deadzone;
    let wanted_impulse = if force_live {
        tf.orient * Vec3::X * input.throttle * ship.max_impulse
    } else {
        Vec3::ZERO
    };

    // Apply directional impulse
    if wanted_impulse != Vec3::ZERO {
        let total_impulse = wanted_impulse.length().min(ship.max_impulse);
        let norm = wanted_impulse.normalize_or_zero();
        let impulse = total_impulse * norm;
        kt.force(impulse * dt);
    }

    // Roll input
    let roll_deadzone = 0.05;
    let desired_roll = if input.roll.abs() > roll_deadzone {
        input.roll
    } else {
        0.
    };

    // Follow pathdirection smoothly
    let future_pt = path.lerp(nearest_ctrlp_idx as f32 + 3.5);
    let wanted_orient =
        future_pt.orient * Quat::from_euler(EulerRot::XYZ, desired_roll * PI / 16., 0., 0.);

    let track_rel_vel = nearest_ctrlp.orient.inverse() * kt.vel;
    let lerp_speed = dt * track_rel_vel.x / TRACK_LENGTH;
    tf.orient = tf.orient.slerp(wanted_orient, lerp_speed * 2.);

    // Horizontal thrusters
    let horiz_force = nearest_ctrlp.orient * Vec3::Z;

    let available_power = track_rel_vel.x.abs().powf(1.1) + track_rel_vel.z.abs() + 1.;
    kt.vel += horiz_force * dt * available_power * (desired_roll * PI / 2.).sin();

    // Zero velocity component in the y direction relative to the track
    kt.vel -= nearest_ctrlp.orient * Vec3::Y * track_rel_vel.y;

    // Lock Y pos to track
    let wanted_y = nearest_ctrlp.pos.y;
    tf.pos.y = lerp(tf.pos.y, wanted_y, lerp_speed);
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    (1. - t) * a + t * b
}
