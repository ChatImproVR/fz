use cimvr_common::{
    nalgebra::{Point3, UnitQuaternion},
    render::{Mesh, MeshHandle, Primitive, Render, UploadMesh},
    Transform, FrameTime,
};
use cimvr_engine_interface::{dbg, make_app_state, pkg_namespace, prelude::*, println};

use crate::obj::obj_lines_to_mesh;

mod obj;

// All state associated with client-side behaviour
struct ClientState;

pub const SHIP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Ship"));
pub const PATH_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Path"));
pub const ENVIRONMENT_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Environment"));

fn orientations(mesh: &Mesh) -> Vec<Transform> {
    let mut transforms = vec![];

    for axes in mesh.vertices.chunks_exact(4) {
        let origin = Point3::from(axes[0].pos);
        transforms.push(Transform {
            pos: origin,
            orient: UnitQuaternion::identity(),
        })
    }

    transforms
}

impl UserState for ClientState {
    // Implement a constructor
    fn new(io: &mut EngineIo, _sched: &mut EngineSchedule<Self>) -> Self {
        //let mesh = obj_lines_to_mesh(include_str!("assets/ship.obj"));
        let environment_mesh = obj_lines_to_mesh(include_str!("assets/environment.obj"));
        io.send(&UploadMesh {
            mesh: environment_mesh,
            id: ENVIRONMENT_RDR,
        });

        let ship_mesh = obj_lines_to_mesh(include_str!("assets/ship.obj"));
        io.send(&UploadMesh {
            mesh: ship_mesh,
            id: SHIP_RDR,
        });

        Self
    }
}

// All state associated with server-side behaviour
struct ServerState {
    n: usize,
    transforms: Vec<Transform>,
    ship_ent: EntityId,
}

impl UserState for ServerState {
    // Implement a constructor
    fn new(io: &mut EngineIo, sched: &mut EngineSchedule<Self>) -> Self {
        let ship_ent = io.create_entity();
        io.add_component(ship_ent, &Transform::identity());
        io.add_component(ship_ent, &Render::new(SHIP_RDR).primitive(Primitive::Lines));
        io.add_component(ship_ent, &Synchronized);

        let env_ent = io.create_entity();
        io.add_component(env_ent, &Transform::identity());
        io.add_component(
            env_ent,
            &Render::new(ENVIRONMENT_RDR).primitive(Primitive::Lines),
        );
        io.add_component(env_ent, &Synchronized);

        let path_mesh = obj_lines_to_mesh(include_str!("assets/path.obj"));
        let transforms = orientations(&path_mesh);

        sched.add_system(Self::update, SystemDescriptor::new(Stage::Update).subscribe::<FrameTime>());

        println!("Hello, server!");
        Self {
            n: 0,
            transforms,
            ship_ent,
        }
    }
}

impl ServerState {
    fn update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        //self.n = (self.n + 1) % self.transforms.len();

        if let Some(FrameTime { time, .. }) = io.inbox_first() {
            let i = (time * 1.) as usize;
            self.n = i % self.transforms.len();
            io.add_component(self.ship_ent, &self.transforms[self.n]);
        }

    }
}

// Defines entry points for the engine to hook into.
// Calls new() for the appropriate state.
make_app_state!(ClientState, ServerState);
