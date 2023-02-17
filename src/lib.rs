use cimvr_common::{
    nalgebra::{Matrix3, Point3, UnitQuaternion, Matrix4, Isometry3},
    render::{Mesh, MeshHandle, Primitive, Render, UploadMesh, CameraComponent, Vertex},
    FrameTime, Transform,
};
use cimvr_engine_interface::{make_app_state, pkg_namespace, prelude::*, println};

use crate::obj::obj_lines_to_mesh;

mod obj;

// All state associated with client-side behaviour
struct ClientState;

pub const SHIP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Ship"));
pub const PATH_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Path"));
pub const ENVIRONMENT_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Environment"));
pub const FLOOR_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Floor"));

// Need a function which can turn a position in 3D and a previous value, and return a next value
// This value correspondss to the curve interpolation over the whole shibam

fn orientations(mesh: &Mesh) -> Vec<Transform> {
    let mut transforms = vec![];

    for axes in mesh.vertices.chunks_exact(4) {
        let origin = Point3::from(axes[1].pos);

        let to_vect = |i: usize| (Point3::from(axes[i].pos) - origin);

        let x = -to_vect(0);
        let y = to_vect(2);
        let z = -to_vect(3);

        let mat = Matrix3::from_columns(&[x, y, -z]);
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
        io.add_component(floor_ent, &Transform::new().with_position(Point3::new(0., -50., 0.)));
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
    fn update(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {
        //self.n = (self.n + 1) % self.transforms.len();

        if let Some(FrameTime { time, .. }) = io.inbox_first() {
            let time = time * 2.;
            let transf = curve(&self.transforms, time);
            io.add_component(self.ship_ent, &transf);
        }
    }
}

fn curve(transf: &[Transform], t: f32) -> Transform {
    // Index part of path position
    let i = t.floor() as usize;
    let len = transf.len();

    let behind = transf[i % len];
    let in_front = transf[(i + 1) % len];

    let interp = t.fract();
    transf_lerp(behind, in_front, interp)
}

fn transf_lerp(a: Transform, b: Transform, t: f32) -> Transform {
    let a: Isometry3<f32> = a.into();
    let b: Isometry3<f32> = b.into();
    a.lerp_slerp(&b, t).into()
}

// Defines entry points for the engine to hook into.
// Calls new() for the appropriate state.
make_app_state!(ClientState, ServerState);

fn grid_mesh(n: i32, scale: f32) -> Mesh {
    let mut m = Mesh::new();

    let color = [0., 1., 0.];

    let width = n as f32 * scale;

    for i in -n..=n {
        let j = i as f32 * scale;

        let positions = [
            [j, 0.0, width],
            [j, 0.0, -width],
            [width, 0.0, j],
            [-width, 0.0, j],
        ];

        for pos in positions {
            let idx = m.push_vertex(Vertex::new(pos, color));
            m.indices.push(idx);
        }
    }

    m
}
