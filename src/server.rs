use std::collections::{HashMap, HashSet};

use cimvr_common::{
    glam::Vec3,
    render::{Primitive, Render},
    Transform,
};
use cimvr_engine_interface::{dbg, prelude::*, println, FrameTime};
use kinematics::KinematicPhysics;

use crate::{kinematics, ClientReady, ServerShipComponent, ShipUpload, StartRace, SHIP_RDR};

// All state associated with server-side behaviour
pub struct ServerState;

impl UserState for ServerState {
    // Implement a constructor
    fn new(_io: &mut EngineIo, sched: &mut EngineSchedule<Self>) -> Self {
        // Add connection monitoring
        sched
            .add_system(Self::conn_update)
            .stage(Stage::PreUpdate)
            .subscribe::<Connections>()
            .query(Query::new("ServerShip").intersect::<ServerShipComponent>(Access::Write))
            .build();

        // Add physics system
        sched
            .add_system(Self::kinematics_update)
            .query(
                Query::new("Kinematics")
                    .intersect::<Transform>(Access::Write)
                    .intersect::<KinematicPhysics>(Access::Write),
            )
            .subscribe::<FrameTime>()
            .build();

        sched
            .add_system(Self::client_state_update)
            .subscribe::<ClientReady>()
            .query(Query::new("ServerShips").intersect::<ServerShipComponent>(Access::Write))
            .build();

        sched
            .add_system(Self::ship_update)
            .subscribe::<ShipUpload>()
            .query(
                Query::new("ServerShips")
                    .intersect::<ServerShipComponent>(Access::Read)
                    .intersect::<Transform>(Access::Write)
                    .intersect::<KinematicPhysics>(Access::Write),
            )
            .build();

        Self {}
    }
}

impl ServerState {
    fn ship_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        // Interpret the last shipupload message we received from each client,
        // and use it to set the position of each ship entity
        let ship_updates: HashMap<ClientId, ShipUpload> =
            io.inbox_clients::<ShipUpload>().collect();

        for entity in query.iter("ServerShips") {
            let ServerShipComponent { client_id, .. } = query.read(entity);
            if let Some(ShipUpload(transform, kt)) = ship_updates.get(&client_id) {
                query.write(entity, transform);
                query.write(entity, kt);
            }
        }
    }

    fn client_state_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        // Update ready-states
        for (client_id, ClientReady(is_ready)) in io.inbox_clients() {
            for entity in query.iter("ServerShips") {
                if query.read::<ServerShipComponent>(entity).client_id == client_id {
                    query.modify::<ServerShipComponent>(entity, |s| s.is_ready = is_ready);
                    println!(
                        "{:?} is {}",
                        client_id,
                        if is_ready { "ready" } else { "not ready" }
                    );
                }
            }
        }

        // Check if all ships are ready
        let mut all_ready = true;
        let mut any_ready = false;
        for entity in query.iter("ServerShips") {
            let shipc = query.read::<ServerShipComponent>(entity);
            let is_ready = shipc.is_ready;
            all_ready &= is_ready;
            any_ready |= is_ready;
        }

        // Start the race!
        if any_ready && all_ready {
            println!("Starting race!");
            let mut position = Transform::new().with_position(Vec3::new(0., 0., -5.));

            for entity in query.iter("ServerShips") {
                let client_id = query.read::<ServerShipComponent>(entity).client_id;

                io.send_to_client(
                    &StartRace {
                        position,
                        client_id,
                    },
                    client_id,
                );

                position.pos.x -= 5.;
                position.pos.z = -position.pos.z;

                query.modify::<ServerShipComponent>(entity, |s| s.is_ready = false);
            }
        }
    }

    fn conn_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        if let Some(Connections { clients }) = io.inbox_first() {
            let current_connections: HashSet<ClientId> =
                clients.into_iter().map(|c| c.id).collect();

            // Remove entities corresponding to disconnected clients
            for entity in query.iter("ServerShip") {
                let ServerShipComponent { client_id, .. } = query.read(entity);
                if !current_connections.contains(&client_id) {
                    io.remove_entity(entity);
                }
            }

            // Filter for new connections
            let mut new_connections = current_connections;
            for entity in query.iter("ServerShip") {
                let ServerShipComponent { client_id, .. } = query.read(entity);
                if !new_connections.remove(&client_id) {
                    println!("{:?} disconnected", client_id);
                }
            }

            // Add a new ship entity for each new connection
            for client_id in new_connections {
                println!("{:?} connected", client_id);
                io.create_entity()
                    .add_component(Transform::identity())
                    .add_component(Render::new(SHIP_RDR).primitive(Primitive::Lines))
                    .add_component(ServerShipComponent {
                        client_id,
                        racing: false,
                        is_ready: false,
                    })
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
