use cimvr_common::{
    nalgebra::{Matrix3, Point3, Rotation3, UnitQuaternion, Vector3, Matrix4},
    render::{Mesh, MeshHandle, Primitive, Render, UploadMesh, CameraComponent, Vertex},
    FrameTime, Transform,
};
use cimvr_engine_interface::{dbg, make_app_state, pkg_namespace, prelude::*, println};

use crate::obj::obj_lines_to_mesh;

mod obj;

// All state associated with client-side behaviour
struct ClientState;

pub const SHIP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Ship"));
pub const PATH_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Path"));
pub const ENVIRONMENT_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Environment"));
pub const FLOOR_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Floor"));

fn orientations(mesh: &Mesh) -> Vec<Transform> {
    let mut transforms = vec![];

    for axes in mesh.vertices.chunks_exact(4) {
        let origin = Point3::from(axes[1].pos);

        let to_vect = |i: usize| (Point3::from(axes[i].pos) - origin);

        let x = to_vect(0);
        let y = to_vect(2);
        let z = -to_vect(3);

        let mat = Matrix3::from_columns(&[z, y, -x]);
        let orient = UnitQuaternion::from_matrix(&mat);

        transforms.push(Transform {
            pos: origin,
            orient,
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

        io.send(&UploadMesh {
            mesh: grid_mesh(20, 20.),
            id: FLOOR_RDR,
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
        //io.add_component(ship_ent, &Render::new(SHIP_RDR).primitive(Primitive::Lines));
        io.add_component(ship_ent, &Synchronized);
        io.add_component(ship_ent, &CameraComponent {
            clear_color: [0.; 3],
            projection: [Matrix4::new_perspective(2., 1.3, 0.001, 1000.); 2]
        });

        let env_ent = io.create_entity();
        io.add_component(env_ent, &Transform::identity());
        io.add_component(
            env_ent,
            &Render::new(ENVIRONMENT_RDR).primitive(Primitive::Lines),
        );
        io.add_component(env_ent, &Synchronized);

        let floor_ent = io.create_entity();
        io.add_component(floor_ent, &Transform::identity());
        io.add_component(
            floor_ent,
            &Render::new(FLOOR_RDR).primitive(Primitive::Lines),
        );
        io.add_component(floor_ent, &Synchronized);


        let path_mesh = obj_lines_to_mesh(include_str!("assets/path.obj"));
        let transforms = orientations(&path_mesh);

        sched.add_system(
            Self::update,
            SystemDescriptor::new(Stage::Update).subscribe::<FrameTime>(),
        );

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
            let time = time * 8.;

            let i = time.floor() as usize;
            let len = self.transforms.len();
            self.n = i % len;

            let behind = &self.transforms[self.n];
            let in_front = &self.transforms[(self.n + 1) % len];

            let interp = time.fract();
            let transf = transf_lerp(behind, in_front, interp);

            io.add_component(self.ship_ent, &transf);
        }
    }
}

fn transf_lerp(a: &Transform, b: &Transform, t: f32) -> Transform {
    Transform {
        pos: a.pos.coords.lerp(&b.pos.coords, t).into(),
        orient: a.orient.slerp(&b.orient, t),
    }
}

// Defines entry points for the engine to hook into.
// Calls new() for the appropriate state.
make_app_state!(ClientState, ServerState);

fn grid_mesh(n: i32, scale: f32) -> Mesh {
    let mut m = Mesh::new();

    let color = [0., 1., 0.];
    let z = -50.;

    for i in -n..=n {
        let a = [i as f32 * scale, z, n as f32 * scale];
        let b = [i as f32 * scale, z, -n as f32 * scale];

        m.indices.push(m.vertices.len() as u32);
        m.vertices.push(Vertex::new(a, color));

        m.indices.push(m.vertices.len() as u32);
        m.vertices.push(Vertex::new(b, color));

        let a = [n as f32 * scale, z, i as f32 * scale];
        let b = [-n as f32 * scale, z, i as f32 * scale];

        m.indices.push(m.vertices.len() as u32);
        m.vertices.push(Vertex::new(a, color));

        m.indices.push(m.vertices.len() as u32);
        m.vertices.push(Vertex::new(b, color));
    }

    m
}
