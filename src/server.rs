use std::collections::{HashMap, HashSet};

use cimvr_common::{
    render::{Primitive, Render},
    Transform,
};
use cimvr_engine_interface::{prelude::*, println, FrameTime};
use kinematics::KinematicPhysics;

use crate::{kinematics, ClientIdMessage, ServerShipComponent, ShipUpload, SHIP_RDR};

// All state associated with server-side behaviour
pub struct ServerState {
    last_clients: HashSet<ClientId>,
}

impl UserState for ServerState {
    // Implement a constructor
    fn new(_io: &mut EngineIo, sched: &mut EngineSchedule<Self>) -> Self {
        // Add connection monitoring
        sched
            .add_system(Self::conn_update)
            .stage(Stage::PreUpdate)
            .subscribe::<Connections>()
            .query("ServerShip")
                .intersect::<ServerShipComponent>(Access::Write)
                .finish()
            .build();

        // Add physics system
        sched
            .add_system(Self::kinematics_update)
            .query("Kinematics")
                .intersect::<Transform>(Access::Write)
                .intersect::<KinematicPhysics>(Access::Write)
                .finish()
            .subscribe::<FrameTime>()
            .build();

        sched
            .add_system(Self::ship_update)
            .subscribe::<ShipUpload>()
            .query("ServerShips")
                .intersect::<ServerShipComponent>(Access::Read)
                .intersect::<Transform>(Access::Write)
                .intersect::<KinematicPhysics>(Access::Write)
                .finish()
            .build();

        Self {
            last_clients: Default::default(),
        }
    }
}

impl ServerState {
    fn ship_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        // Interpret the last shipupload message we received from each client,
        // and use it to set the position of each ship entity
        let ship_updates: HashMap<ClientId, ShipUpload> =
            io.inbox_clients::<ShipUpload>().collect();

        for entity in query.iter("ServerShips") {
            let ServerShipComponent(client_id) = query.read(entity);
            if let Some(ShipUpload(transform, kt)) = ship_updates.get(&client_id) {
                query.write(entity, transform);
                query.write(entity, kt);
            }
        }
    }

    fn conn_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        if let Some(Connections { clients }) = io.inbox_first() {
            let current_connections: HashSet<ClientId> =
                clients.into_iter().map(|c| c.id).collect();

            // Remove entities corresponding to disconnected clients
            for entity in query.iter("ServerShip") {
                let ServerShipComponent(client_id) = query.read(entity);
                if !current_connections.contains(&client_id) {
                    io.remove_entity(entity);
                }
            }

            // Filter for new connections
            let mut new_connections = current_connections;
            for entity in query.iter("ServerShip") {
                let ServerShipComponent(client_id) = query.read(entity);
                if !new_connections.remove(&client_id) {
                    println!("{:?} disconnected", client_id);
                }

                io.send_to_client(&ClientIdMessage(client_id), client_id);
            }

            // Add a new ship entity for each new connection
            for client_id in new_connections {
                println!("{:?} connected", client_id);
                io.create_entity()
                    .add_component(Transform::identity())
                    .add_component(Render::new(SHIP_RDR).primitive(Primitive::Lines))
                    .add_component(ServerShipComponent(client_id))
                    .add_component(Synchronized)
                    .add_component(KinematicPhysics::default())
                    .build();
            }
        }
    }

    /// Simulate kinematics
    fn kinematics_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        let Some(FrameTime { delta, .. }) = io.inbox_first() else { return };
        kinematics::simulate(query, delta);
    }
}
