use std::f32::consts::FRAC_PI_2;

use cimvr_common::{
    desktop::InputEvents,
    gamepad::{Axis, GamepadState},
    nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3},
    render::{CameraComponent, Mesh, MeshHandle, Primitive, Render, UploadMesh, Vertex},
    utils::camera::Perspective,
    vr::VrUpdate,
    Transform,
};
use cimvr_engine_interface::{dbg, make_app_state, pkg_namespace, prelude::*, println, FrameTime};
use kinematics::KinematicPhysics;
use serde::{Deserialize, Serialize};

use crate::obj::obj_lines_to_mesh;

mod kinematics;
mod obj;
mod curve;
use curve::Path;

// All state associated with client-side behaviour
struct ClientState {
    proj: Perspective,
}

pub const SHIP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Ship"));
pub const MAP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Map"));
pub const FLOOR_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Floor"));
pub const CUBE_HANDLE: MeshHandle = MeshHandle::new(pkg_namespace!("Cube"));

// Need a function which can turn a position in 3D and a previous value, and return a next value
// This value correspondss to the curve interpolation over the whole shibam

#[derive(Serialize, Deserialize, Debug, Default, Clone, Copy)]
struct InputAbstraction {
    /// Desired pitching power
    pitch: f32,
    /// Desired yaw power
    yaw: f32,
    /// Desired roll power
    roll: f32,
    /// Desired thrust
    throttle: f32,
}

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
        // Upload ship
        io.send(&UploadMesh {
            mesh: ship_mesh,
            id: SHIP_RDR,
        });

        // Upload cube
        io.send(&UploadMesh {
            mesh: cube(),
            id: CUBE_HANDLE,
        });

        sched.add_system(
            Self::camera,
            SystemDescriptor::new(Stage::PreUpdate)
                .subscribe::<InputEvents>()
                .subscribe::<VrUpdate>()
                .subscribe::<FrameTime>()
                .query::<CameraComponent>(Access::Write),
        );

        sched.add_system(
            Self::controller_input,
            SystemDescriptor::new(Stage::PreUpdate).subscribe::<GamepadState>(),
        );

        Self {
            proj: Perspective::new(),
        }
    }
}

impl ClientState {
    fn camera(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
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

    fn controller_input(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {
        let mut input = InputAbstraction::default();

        if let Some(GamepadState(gamepads)) = io.inbox_first() {
            if let Some(gamepad) = gamepads.into_iter().next() {
                input.yaw = gamepad.axes[&Axis::RightStickX];
                input.pitch = gamepad.axes[&Axis::LeftStickY];
                input.roll = gamepad.axes[&Axis::LeftStickX];
                input.throttle = gamepad.axes[&Axis::RightStickY];
            }
        }

        io.send(&input);
    }
}

// All state associated with server-side behaviour
struct ServerState {
    n: usize,
    path: Path,
    ship: ShipCharacteristics,
    ship_ent: EntityId,
    camera_ent: EntityId,
    cube_ent: EntityId,
    last_input_state: InputAbstraction,
}

#[derive(serde::Serialize, serde::Deserialize, Copy, Clone)]
struct ShipComponent;
impl Component for ShipComponent {
    const ID: ComponentIdStatic = ComponentIdStatic {
        id: pkg_namespace!("Ship"),
        size: 0,
    };
}

impl UserState for ServerState {
    // Implement a constructor
    fn new(io: &mut EngineIo, sched: &mut EngineSchedule<Self>) -> Self {
        // Add ship
        let ship_ent = io.create_entity();
        io.add_component(ship_ent, &Transform::identity());
        io.add_component(ship_ent, &Render::new(SHIP_RDR).primitive(Primitive::Lines));
        io.add_component(ship_ent, &Synchronized);
        io.add_component(ship_ent, &ShipComponent);
        io.add_component(
            ship_ent,
            &KinematicPhysics {
                vel: Vector3::zeros(),
                mass: 1.,
                ang_vel: Vector3::zeros(),
                moment: 1.,
            },
        );

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

        // Create a cube
        let cube_ent = io.create_entity();
        io.add_component(cube_ent, &Transform::default());
        io.add_component(cube_ent, &Render::new(CUBE_HANDLE).primitive(Primitive::Triangles));
        io.add_component(cube_ent, &Synchronized);

        // Parse path mesh
        let path_mesh = obj_lines_to_mesh(include_str!("assets/path.obj"));
        let transforms = orientations(&path_mesh);
        let path = Path::new(transforms);

        // Add gamepad/keyboard input system
        sched.add_system(
            Self::input_update,
            SystemDescriptor::new(Stage::PreUpdate).subscribe::<InputAbstraction>(),
        );

        // Add physics system
        sched.add_system(
            Self::kinematics_update,
            SystemDescriptor::new(Stage::Update)
                .query::<Transform>(Access::Write)
                .query::<KinematicPhysics>(Access::Read)
                .subscribe::<FrameTime>(),
        );

        // NOTE: Camera update happens after kinematics update so it doesn't lag behind in quick
        // action!
        sched.add_system(
            Self::camera_update,
            SystemDescriptor::new(Stage::Update)
                .query::<Transform>(Access::Read)
                // Here we're using the ShipComponent as a filter, so the engine doesn't have to
                // send us a tone of data! We DON'T have to use a component to set the camera's
                // position, because we can abuse add_component()...
                .query::<ShipComponent>(Access::Read)
                .subscribe::<FrameTime>(),
        );

        // Define ship capabilities
        let ship = ShipCharacteristics {
            mass: 1000.,
            moment: 1000. * 3_f32.powi(2),
            max_twirl: 5.,
            max_impulse: 30.,
        };

        Self {
            camera_ent,
            cube_ent,
            last_input_state: InputAbstraction::default(),
            ship,
            n: 0,
            path,
            ship_ent,
        }
    }
}

#[derive(Clone, Default, Copy)]
struct ShipCharacteristics {
    /// Mass of the ship (Kg)
    mass: f32,
    /// Ship's moment of inertia (Kg * m^2)
    moment: f32,
    /// Maximum angular impulse power (Newton-meters)
    max_twirl: f32,
    /// Maximum thrust (Newtons)
    max_impulse: f32,
}

fn ship_controller(
    dt: f32,
    ship: ShipCharacteristics,
    input: InputAbstraction,
    tf: Transform,
    path: &Path,
    kt: &mut KinematicPhysics,
) {
    // Control vector
    let ang_control = Vector3::new(input.roll, -input.yaw, -input.pitch);

    let yaw_pitch_deadzone = 0.1;
    let roll_deadzone = 0.8;
    let ang_damping = 8.;
    let forward_damping = 1.;
    let throttle_deadzone = 0.1;
    let ang_multiplier = 0.3;
    let centering_mul = 80.;

    // Calculate wanted rotation to match the course
    let nearest_ctrlp = path.nearest_ctrlp(tf.pos);
    let lookahead_ctrlp = (nearest_ctrlp + 2) % path.ctrlps.len();
    let local_path_tf = path.ctrlps[lookahead_ctrlp];


    // Angular controls
    // Decide if the user wants control
    let ang_live =
        ang_control.yz().magnitude() > yaw_pitch_deadzone || ang_control.x.abs() > roll_deadzone;

    let wanted_ang_impulse = if ang_live {
        // Angular impulse
        tf.orient * ang_control * ship.max_twirl * ang_multiplier
    } else {
        // Align with path direction
        let wanted_rotation = tf.orient.rotation_to(&local_path_tf.orient);
        let (r, p, y) = wanted_rotation.euler_angles();
        let centering = Vector3::new(r, p, y) * centering_mul;

        // Rotation damping
        let damping = -kt.ang_vel * ang_damping;

        centering + damping
    };

    // Apply angular impulse
    if wanted_ang_impulse != Vector3::zeros() {
        let total_ang_impulse = wanted_ang_impulse.magnitude().min(ship.max_twirl);
        if let Some(ang_norm) = wanted_ang_impulse.try_normalize(f32::EPSILON) {
            let ang_impulse = total_ang_impulse * ang_norm;
            kt.torque(ang_impulse * dt);
        }
    }

    // Force controls
    let force_live = input.throttle > throttle_deadzone;
    let wanted_impulse = if force_live {
        tf.orient * Vector3::x() * input.throttle * ship.max_impulse
    } else {
        -kt.vel * forward_damping
    };

    let path_forward = local_path_tf.orient * Vector3::x();
    let proj = path_forward.dot(&kt.vel);
    let intertial_damp = -(kt.vel - proj * path_forward);
    let wanted_impulse = wanted_impulse + proj * intertial_damp;


    if wanted_impulse != Vector3::zeros() {
        let total_impulse = wanted_impulse.magnitude().min(ship.max_impulse);
        if let Some(norm) = wanted_impulse.try_normalize(f32::EPSILON) {
            let impulse = total_impulse * norm;
            kt.force(impulse * dt);
        }
    }
}

impl ServerState {
    fn input_update(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {
        for input in io.inbox::<InputAbstraction>() {
            self.last_input_state = input
        }
    }

    fn kinematics_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        if let Some(FrameTime { delta, .. }) = io.inbox_first() {
            let dt = delta;

            let gravity = Vector3::y() * -0.5;

            let ship_tf = query.read::<Transform>(self.ship_ent);

            let nearest_ctrlp = self.path.nearest_ctrlp(ship_tf.pos);
            let path_transf = self.path.ctrlps[nearest_ctrlp];
            io.add_component(self.cube_ent, &path_transf);

            query.modify::<KinematicPhysics>(self.ship_ent, |k| {
                //let diff = Vector3::zeros() - tf.translation.vector;
                //k.force(diff.magnitude() * diff / 1000.);
                //k.torque(Vector3::new(0., 0.1, 0.) * dt);

                // Antigravity drive :)
                k.force(-gravity * dt);

                ship_controller(dt, self.ship, self.last_input_state, ship_tf, &self.path, k)

                //k.force(tf.rotation * Vector3::new(10., 0., 0.) * dt * w.magnitude_squared());
            });

            //query.modify::<Transform>(ship_key, |t| t.pos = (t.pos / 100.).map(|x| x.fract()));

            kinematics::gravity(query, dt, gravity);
            kinematics::simulate(query, dt);
        } else {
            println!("Expected FrameTime message!");
        }
    }

    fn camera_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        let ship_transf: Transform = query.read(self.ship_ent);

        let cam_pos = Transform::new()
            .with_rotation(UnitQuaternion::from_euler_angles(0., -FRAC_PI_2, 0.))
            .with_position(Point3::new(-13., 2., 0.));

        let cam_transf = ship_transf * cam_pos;

        io.add_component(self.camera_ent, &cam_transf);
    }
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

impl Message for InputAbstraction {
    const CHANNEL: ChannelIdStatic = ChannelIdStatic {
        id: pkg_namespace!("InputAbstraction"),
        locality: Locality::Remote,
    };
}


/// Defines the mesh data fro a cube
fn cube() -> Mesh {
    let size = 0.25;
    let vertices = vec![
        Vertex::new([-size, -size, -size], [0.0, 1.0, 1.0]),
        Vertex::new([size, -size, -size], [1.0, 0.0, 1.0]),
        Vertex::new([size, size, -size], [1.0, 1.0, 0.0]),
        Vertex::new([-size, size, -size], [0.0, 1.0, 1.0]),
        Vertex::new([-size, -size, size], [1.0, 0.0, 1.0]),
        Vertex::new([size, -size, size], [1.0, 1.0, 0.0]),
        Vertex::new([size, size, size], [0.0, 1.0, 1.0]),
        Vertex::new([-size, size, size], [1.0, 0.0, 1.0]),
    ];

    let indices = vec![
        3, 1, 0, 2, 1, 3, 2, 5, 1, 6, 5, 2, 6, 4, 5, 7, 4, 6, 7, 0, 4, 3, 0, 7, 7, 2, 3, 6, 2, 7,
        0, 5, 4, 1, 5, 0,
    ];

    Mesh { vertices, indices }
}
