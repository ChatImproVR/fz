use std::f32::consts::FRAC_PI_2;

use cimvr_common::{
    desktop::InputEvents,
    nalgebra::{Isometry3, Matrix3, Matrix4, Point3, UnitQuaternion, Vector2, Point2},
    render::{CameraComponent, Mesh, MeshHandle, Primitive, Render, UploadMesh, Vertex},
    utils::camera::Perspective,
    vr::VrUpdate,
    Transform,
};
use cimvr_engine_interface::{FrameTime, make_app_state, pkg_namespace, dbg, prelude::*, println};

use crate::obj::obj_lines_to_mesh;

mod kinematics;
mod obj;

// All state associated with client-side behaviour
struct ClientState {
    proj: Perspective,
}

pub const SHIP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Ship"));
pub const PATH_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Path"));
pub const MAP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Map"));
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

        let mat = Matrix3::from_columns(&[-x, y, z]);
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
    fn new(io: &mut EngineIo, sched: &mut EngineSchedule<Self>) -> Self {
        //let mesh = obj_lines_to_mesh(include_str!("assets/ship.obj"));
        let environment_mesh = obj_lines_to_mesh(include_str!("assets/environment.obj"));
        io.send(&UploadMesh {
            mesh: environment_mesh,
            id: MAP_RDR,
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

        sched.add_system(
            Self::camera,
            SystemDescriptor::new(Stage::PreUpdate)
                .subscribe::<InputEvents>()
                .subscribe::<VrUpdate>()
                .subscribe::<FrameTime>()
                .query::<CameraComponent>(Access::Write),
        );

        Self {
            proj: Perspective::new(),
        }
    }
}

impl ClientState {
    fn camera(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        if let Some(FrameTime { delta, time }) = io.inbox_first::<FrameTime>() {
            //dbg!(delta);
        }

        if let Some(input) = io.inbox_first::<InputEvents>() {
            self.proj.handle_input_events(&input);
        }

        if let Some(update) = io.inbox_first::<VrUpdate>() {
            self.proj.handle_vr_update(&update);
        }

        let projection = self.proj.matrices();
        self.proj.fov = 79_f32.to_radians();
        let clear_color = [0.; 3];

        for key in query.iter() {
            query.write::<CameraComponent>(
                key,
                &CameraComponent {
                    clear_color,
                    projection,
                },
            );
        }
    }
}

// All state associated with server-side behaviour
struct ServerState {
    n: usize,
    path: Path,
    ship_ent: EntityId,
    camera_ent: EntityId,
}

impl UserState for ServerState {
    // Implement a constructor
    fn new(io: &mut EngineIo, sched: &mut EngineSchedule<Self>) -> Self {
        // Add ship
        let ship_ent = io.create_entity();
        io.add_component(ship_ent, &Transform::identity());
        io.add_component(ship_ent, &Render::new(SHIP_RDR).primitive(Primitive::Lines));
        io.add_component(ship_ent, &Synchronized);

        // Add camera
        let camera_ent = io.create_entity();
        io.add_component(camera_ent, &Transform::identity());
        io.add_component(camera_ent, &CameraComponent::default());
        io.add_component(camera_ent, &Synchronized);

        // Add environment
        let env_ent = io.create_entity();
        io.add_component(env_ent, &Transform::identity());
        io.add_component(env_ent, &Render::new(MAP_RDR).primitive(Primitive::Lines));
        io.add_component(env_ent, &Synchronized);

        // Add floor
        let floor_ent = io.create_entity();
        io.add_component(
            floor_ent,
            &Transform::new().with_position(Point3::new(0., -50., 0.)),
        );
        io.add_component(
            floor_ent,
            &Render::new(FLOOR_RDR).primitive(Primitive::Lines),
        );
        io.add_component(floor_ent, &Synchronized);

        // Parse path mesh
        let path_mesh = obj_lines_to_mesh(include_str!("assets/path.obj"));
        let transforms = orientations(&path_mesh);
        let path = Path::new(transforms);

        // Add update system
        sched.add_system(
            Self::update,
            SystemDescriptor::new(Stage::Update).subscribe::<FrameTime>(),
        );

        Self {
            camera_ent,
            n: 0,
            path,
            ship_ent,
        }
    }
}

impl ServerState {
    fn update(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {
        if let Some(FrameTime { time, delta }) = io.inbox_first() {
            //dbg!(delta);
            let time = time * 7.;
            let ship_transf = self.path.lerp(time);
            io.add_component(self.ship_ent, &ship_transf);

            let cam_pos = Transform::new()
                .with_rotation(UnitQuaternion::from_euler_angles(0., -FRAC_PI_2, 0.))
                .with_position(Point3::new(-13., 2., 0.));

            let cam_transf = ship_transf * cam_pos;

            io.add_component(self.camera_ent, &cam_transf);
        }
    }
}

struct Path {
    ctrlps: Vec<Transform>,
}

impl Path {
    pub fn new(ctrlps: Vec<Transform>) -> Self {
        Self { ctrlps }
    }

    fn index(&self, t: f32) -> (usize, usize) {
        // Index part of path position
        let i = t.floor() as usize;
        let len = self.ctrlps.len();

        let behind = i % len;
        let in_front = (i + 1) % len;

        (behind, in_front)
    }

    fn lerp(&self, t: f32) -> Transform {
        let (behind, in_front) = self.index(t);
        transf_lerp(self.ctrlps[behind], self.ctrlps[in_front], t.fract())
    }

    /// Returns an index which is approximately `dist` units along the curve from index `t`
    /// Uses pos2d to determine the path length (e.g. the insides of curves are shortest)
    fn step_distance(&self, remaining_dist: f32, t: f32, pos2d: Point2<f32>) -> f32 {
        let mut t = t;
        /* wip 
        let mut remaining_dist = remaining_dist;

        fn pos_3d(tf: Transform, pos2d: Point2<f32>) -> Point3<f32> {
            let iso: Isometry3<f32> = tf.into();
            let pos3d = Point3::new(0., pos2d.y, pos2d.x);
            iso.transform_point(&pos3d)
        }

        while remaining_dist > 0. {
            let current_orient = self.lerp(t);
            let (in_front, _) = self.index(t);
            let current_pos3d = pos_3d(current_orient, pos2d);
            let front_pos3d = pos_3d(self.ctrlps[in_front], pos2d);
            let dist = (current_pos3d - front_pos3d).magnitude();
            if dist <= 0.0 {
                return t;
            }


        }

        */
        t
    }
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
