use cimvr_common::{render::MeshHandle, Transform};
use cimvr_engine_interface::{make_app_state, pkg_namespace, prelude::*};
use kinematics::KinematicPhysics;
use serde::{Deserialize, Serialize};

//mod client_tag;
mod client;
mod controls;
mod countdown;
mod curve;
mod kinematics;
mod obj;
mod server;
mod shapes;
use client::ClientState;
use server::ServerState;

pub const SHIP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Ship"));

/// Clients own the ship positions; this message sends the positions of clients' ships
/// to the server
#[derive(Message, Copy, Clone, Default, Serialize, Deserialize)]
#[locality("Remote")]
struct ShipUpload(Transform, KinematicPhysics);

/// Sent to inform a given client is ready or not
#[derive(Message, Copy, Clone, Default, Serialize, Deserialize)]
#[locality("Remote")]
struct ClientReady(bool);

/// A client finished the race! In the given time...
#[derive(Message, Copy, Clone, Default, Serialize, Deserialize)]
#[locality("Remote")]
struct Finished(f32);

/// Denotes the single ship client-side
#[derive(Component, serde::Serialize, serde::Deserialize, Default, Copy, Clone, PartialEq, Eq)]
struct ClientShipComponent;

/// Denotes a ship corresponding to a client
#[derive(Component, serde::Serialize, serde::Deserialize, Default, Copy, Clone, PartialEq, Eq)]
struct ServerShipComponent {
    pub client_id: ClientId,
    pub is_racing: bool,
    pub is_ready: bool,
}

#[derive(Clone, Default, Copy)]
pub struct ShipCharacteristics {
    /// Mass of the ship (Kg)
    pub mass: f32,
    /// Ship's moment of inertia (Kg * m^2)
    pub moment: f32,
    /// Maximum angular impulse power (Newton-meters)
    pub max_twirl: f32,
    /// Maximum thrust (Newtons)
    pub max_impulse: f32,
}

// Defines entry points for the engine to hook into.
// Calls new() for the appropriate state.
make_app_state!(ClientState, ServerState);

/// Message telling a client which ID it has
#[derive(Message, Serialize, Deserialize, Debug, Clone, Copy)]
#[locality("Remote")]
struct StartRace {
    client_id: ClientId,
    position: Transform,
}

#[derive(Serialize, Deserialize, Debug, Default, Clone, Copy)]
pub struct InputAbstraction {
    /// Desired pitching power
    pitch: f32,
    /// Desired yaw power
    yaw: f32,
    /// Desired roll power
    roll: f32,
    /// Desired thrust
    throttle: f32,
}
