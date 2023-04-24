use std::collections::{HashMap, HashSet};

use cimvr_common::{
    glam::Vec3,
    render::{Primitive, Render},
    Transform,
};
use cimvr_engine_interface::{dbg, prelude::*, println, FrameTime};
use kinematics::KinematicPhysics;

use crate::{
    kinematics, ClientReady, Finished, ServerShipComponent, ShipUpload, StartRace, SHIP_RDR,
};

// All state associated with server-side behaviour
pub struct ServerState {
    winner: Option<(ClientId, f32)>,
    reset_countdown: f32,
}

// All players have 50 seconds after the winner
const RESET_TIME: f32 = 50.;

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
            .add_system(Self::win_reset)
            .query(Query::new("Clients").intersect::<ServerShipComponent>(Access::Write))
            .subscribe::<Finished>()
            .subscribe::<FrameTime>()
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

        Self {
            winner: None,
            reset_countdown: 0.,
        }
    }
}

impl ServerState {
    fn win_reset(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        let Some(FrameTime { time: server_time, .. }) = io.inbox_first() else { return };

        for (client_id, Finished(finish_time)) in io.inbox_clients() {
            // Mark this client as having finished
            for entity in query.iter("Clients") {
                if query.read::<ServerShipComponent>(entity).client_id == client_id {
                    query.modify::<ServerShipComponent>(entity, |s| s.is_racing = false);
                }
            }

            // Decide winner
            if let Some((_, winning_time)) = self.winner {
                if finish_time > winning_time {
                    // Try the next client
                    continue;
                }
            }
            self.winner = Some((client_id, finish_time));
            //io.send(&AnnounceWinner(String));
            self.reset_countdown = server_time + RESET_TIME;
        }

        // Check if anybody is reacing
        let mut anybody_racing = false;
        for entity in query.iter("Clients") {
            anybody_racing |= query.read::<ServerShipComponent>(entity).is_racing;
        }

        // Reset
        let awaiting_losers = server_time > self.reset_countdown;
        if self.winner.is_some() && (awaiting_losers || !anybody_racing) {
            dbg!("Reset");
            self.winner = None;
        }
    }

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

                query.modify::<ServerShipComponent>(entity, |s| {
                    s.is_ready = false;
                    s.is_racing = true;
                });
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
                        is_racing: false,
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
