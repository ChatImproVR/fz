use std::{
    collections::{HashMap, HashSet},
    f32::consts::{FRAC_PI_2, PI},
};

use cimvr_common::{
    desktop::{ElementState, InputEvent, KeyCode, KeyboardEvent},
    gamepad::{Axis, Button, GamepadState},
    glam::{EulerRot, Mat3, Quat, Vec3},
    render::{
        CameraComponent, Mesh, MeshHandle, Primitive, Render, RenderExtra, UploadMesh, Vertex,
    },
    utils::camera::Perspective,
    vr::VrUpdate,
    Transform,
};
use cimvr_engine_interface::{dbg, make_app_state, pkg_namespace, prelude::*, println, FrameTime};
use kinematics::KinematicPhysics;
use serde::{Deserialize, Serialize};

use crate::obj::obj_lines_to_mesh;

mod curve;
mod kinematics;
mod obj;
use curve::Path;

// All state associated with client-side behaviour
struct ClientState {
    proj: Perspective,
    camera_ent: EntityId,
    ship_ent: Option<EntityId>,
    w_is_pressed: bool,
    a_is_pressed: bool,
    s_is_pressed: bool,
    d_is_pressed: bool,
    anim: CountdownAnimation,
}

pub const SHIP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Ship"));
pub const MAP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Map"));
pub const FLOOR_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Floor"));
pub const CUBE_HANDLE: MeshHandle = MeshHandle::new(pkg_namespace!("Cube"));

// Need a function which can turn a position in 3D and a previous value, and return a next value
// This value correspondss to the curve interpolation over the whole shibam

#[derive(Message, Serialize, Deserialize, Debug, Default, Clone, Copy)]
#[locality("Remote")]
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
        let origin = Vec3::from(axes[1].pos);

        let to_vect = |i: usize| (Vec3::from(axes[i].pos) - origin);

        let x = -to_vect(0);
        let y = to_vect(2);
        let z = -to_vect(3);

        let mat = Mat3::from_cols(-x, y, z);
        let orient = Quat::from_mat3(&mat);

        transforms.push(Transform {
            pos: origin,
            orient,
        })
    }

    transforms
}

const ENV_OBJ: &str = include_str!("assets/loop1_env.obj");
const PATH_OBJ: &str = include_str!("assets/loop1_path.obj");

impl UserState for ClientState {
    // Implement a constructor
    fn new(io: &mut EngineIo, sched: &mut EngineSchedule<Self>) -> Self {
        //let mesh = obj_lines_to_mesh(include_str!("assets/ship.obj"));
        let mut environment_mesh = obj_lines_to_mesh(ENV_OBJ);
        environment_mesh.recolor([0.2, 1., 0.2]);
        io.send(&UploadMesh {
            mesh: environment_mesh,
            id: MAP_RDR,
        });

        io.send(&UploadMesh {
            mesh: grid_mesh(20, 20., [0., 0.2, 0.]),
            id: FLOOR_RDR,
        });

        let ship_mesh = obj_lines_to_mesh(include_str!("assets/ship.obj"));
        // Upload ship
        io.send(&UploadMesh {
            mesh: ship_mesh,
            id: SHIP_RDR,
        });

        // Add camera
        let camera_ent = io
            .create_entity()
            .add_component(Transform::identity())
            .add_component(CameraComponent::default())
            .build();

        // Upload cube
        io.send(&UploadMesh {
            mesh: cube(),
            id: CUBE_HANDLE,
        });

        sched
            .add_system(Self::camera)
            .subscribe::<InputEvent>()
            .subscribe::<VrUpdate>()
            .subscribe::<FrameTime>()
            .subscribe::<ShipIdMessage>()
            .query::<Transform>(Access::Write)
            .query::<ShipComponent>(Access::Write)
            .build();

        sched
            .add_system(Self::controller_input)
            .subscribe::<GamepadState>()
            .subscribe::<InputEvent>()
            .build();

        sched
            .add_system(Self::animation)
            .subscribe::<FrameTime>()
            .build();

        let mut anim = CountdownAnimation::new(io);
        CountdownAnimation::assets(io);
        anim.restart();

        Self {
            proj: Perspective::new(),
            anim,
            camera_ent,
            ship_ent: None,
            w_is_pressed: false,
            a_is_pressed: false,
            s_is_pressed: false,
            d_is_pressed: false,
        }
    }
}

impl ClientState {
    fn animation(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {
        let Some(time) = io.inbox_first::<FrameTime>() else { return };
        self.anim.update(io, time);
    }

    fn camera(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        // Perspective matrix stuff
        for event in io.inbox::<InputEvent>() {
            self.proj.handle_event(&event);
        }

        if let Some(update) = io.inbox_first::<VrUpdate>() {
            self.proj.handle_vr_update(&update);
        }

        let projection = self.proj.matrices();
        self.proj.fov = 79_f32.to_radians();
        let clear_color = [0.; 3];

        io.add_component(
            self.camera_ent,
            CameraComponent {
                clear_color,
                projection,
            },
        );

        // Ship ID
        if self.ship_ent.is_none() {
            if let Some(ShipIdMessage(ent)) = io.inbox_first() {
                self.ship_ent = Some(ent);
            }
        }

        // Set camera pos
        if let Some(ship_ent) = query.iter().next() {
            let ship_transf: Transform = query.read(ship_ent);

            let cam_pos = Transform::new()
                .with_rotation(Quat::from_euler(EulerRot::XYZ, 0., -FRAC_PI_2, 0.))
                .with_position(Vec3::new(-13., 2., 0.));

            let cam_transf = ship_transf * cam_pos;

            io.add_component(self.camera_ent, cam_transf);
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
                if gamepad.buttons[&Button::RightTrigger2] {
                    input.throttle = 1.;
                }
                if gamepad.buttons[&Button::LeftTrigger2] {
                    input.throttle = -1.;
                }
            }
        }

        for event in io.inbox::<InputEvent>() {
            if let InputEvent::Keyboard(KeyboardEvent::Key { key, state }) = event {
                let is_pressed = state == ElementState::Pressed;
                match key {
                    KeyCode::W => self.w_is_pressed = is_pressed,
                    KeyCode::A => self.a_is_pressed = is_pressed,
                    KeyCode::S => self.s_is_pressed = is_pressed,
                    KeyCode::D => self.d_is_pressed = is_pressed,
                    _ => (),
                }
            }
        }

        if self.w_is_pressed {
            input.throttle = 1.0;
        }

        if self.s_is_pressed {
            input.throttle = -1.0;
        }

        if self.a_is_pressed {
            input.roll = -1.0;
        }

        if self.d_is_pressed {
            input.roll = 1.0;
        }

        io.send(&input);
    }
}

// All state associated with server-side behaviour
struct ServerState {
    path: Path,
    motion_cfg: ShipCharacteristics,
    clients: HashMap<ClientId, InputAbstraction>,
}

/// Denotes a ship corresponding to a client
#[derive(Component, serde::Serialize, serde::Deserialize, Default, Copy, Clone, PartialEq, Eq)]
struct ShipComponent(ClientId);

impl UserState for ServerState {
    // Implement a constructor
    fn new(io: &mut EngineIo, sched: &mut EngineSchedule<Self>) -> Self {
        // Add environment
        io.create_entity()
            .add_component(Transform::identity())
            .add_component(Render::new(MAP_RDR).primitive(Primitive::Lines))
            .add_component(Synchronized)
            .build();

        // Add floor
        io.create_entity()
            .add_component(Transform::new().with_position(Vec3::new(0., -50., 0.)))
            .add_component(Render::new(FLOOR_RDR).primitive(Primitive::Lines))
            .add_component(Synchronized)
            .build();

        // Parse path mesh
        let path_mesh = obj_lines_to_mesh(PATH_OBJ);
        let transforms = orientations(&path_mesh);
        let path = Path::new(transforms);

        // Add connection monitoring
        sched
            .add_system(Self::conn_update)
            .stage(Stage::PreUpdate)
            .subscribe::<Connections>()
            .query::<ShipComponent>(Access::Write)
            .build();

        // Add gamepad/keyboard input system
        sched
            .add_system(Self::input_update)
            .stage(Stage::PreUpdate)
            .subscribe::<InputAbstraction>()
            .build();

        // Add physics system
        sched
            .add_system(Self::motion_update)
            .query::<Transform>(Access::Write)
            .query::<KinematicPhysics>(Access::Write)
            .query::<ShipComponent>(Access::Read)
            .subscribe::<FrameTime>()
            .build();

        // Add physics system
        sched
            .add_system(Self::kinematics_update)
            .query::<Transform>(Access::Write)
            .query::<KinematicPhysics>(Access::Write)
            .subscribe::<FrameTime>()
            .build();

        // Define ship capabilities
        let ship = ShipCharacteristics {
            mass: 1000.,
            moment: 1000. * 3_f32.powi(2),
            max_twirl: 5.,
            max_impulse: 30.,
        };

        Self {
            motion_cfg: ship,
            path,
            clients: Default::default(),
        }
    }
}

impl ServerState {
    fn conn_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        if let Some(Connections { clients }) = io.inbox_first() {
            for client in &clients {
                let found_ship = query
                    .iter()
                    .find(|&ent| query.read::<ShipComponent>(ent) == ShipComponent(client.id));

                let ship_ent;
                if let Some(ent) = found_ship {
                    ship_ent = ent;
                } else {
                    // Add ship for newly connected client
                    ship_ent = io
                        .create_entity()
                        .add_component(Transform::identity())
                        .add_component(Render::new(SHIP_RDR).primitive(Primitive::Lines))
                        .add_component(Synchronized)
                        .add_component(ShipComponent(client.id))
                        .add_component(KinematicPhysics {
                            vel: Vec3::ZERO,
                            mass: 1.,
                            ang_vel: Vec3::ZERO,
                            moment: 1.,
                        })
                        .build();
                }

                // Tell the client which ship entity to track...
                io.send_to_client(&ShipIdMessage(ship_ent), client.id);
            }

            // Remove ships of disconnected clients
            for ship_ent in query.iter() {
                let ShipComponent(client_id) = query.read(ship_ent);
                let found_client = clients.iter().find(|c| c.id == client_id);
                if found_client.is_none() {
                    io.remove_entity(ship_ent);
                }
            }
        }
    }

    fn input_update(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {
        for (client_id, input) in io.inbox_clients::<InputAbstraction>() {
            self.clients.insert(client_id, input);
        }
    }

    fn motion_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        let Some(FrameTime { delta, .. }) = io.inbox_first() else { return };

        for ship_ent in query.iter() {
            // Get current physical properties
            let mut kt: KinematicPhysics = query.read(ship_ent);
            let mut tf: Transform = query.read(ship_ent);
            let ShipComponent(client_id) = query.read(ship_ent);

            let input = *self
                .clients
                .entry(client_id)
                .or_insert(InputAbstraction::default());

            // Step ship forward in time
            ship_controller(delta, self.motion_cfg, input, &self.path, &mut tf, &mut kt);

            query.write(ship_ent, &kt);
            query.write(ship_ent, &tf);
        }
    }

    /// Simulate kinematics
    fn kinematics_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        let Some(FrameTime { delta, .. }) = io.inbox_first() else { return };
        kinematics::simulate(query, delta);

        //let gravity = Vec3::Y * -0.5;
        //kinematics::gravity(query, delta, gravity);
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
    path: &Path,
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

// Defines entry points for the engine to hook into.
// Calls new() for the appropriate state.
make_app_state!(ClientState, ServerState);

fn grid_mesh(n: i32, scale: f32, color: [f32; 3]) -> Mesh {
    let mut m = Mesh::new();

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

/// Message telling a client which ID it has
#[derive(Message, Serialize, Deserialize, Debug, Clone, Copy)]
#[locality("Remote")]
struct ShipIdMessage(EntityId);

struct CountdownAnimation {
    entities: Vec<EntityId>,
    start_time: f32,
    needs_restart: bool,
}

impl CountdownAnimation {
    const RDR_ID_1: MeshHandle = MeshHandle::new(pkg_namespace!("Countdown1"));
    const RDR_ID_2: MeshHandle = MeshHandle::new(pkg_namespace!("Countdown2"));
    const RDR_ID_3: MeshHandle = MeshHandle::new(pkg_namespace!("Countdown3"));
    const RDR_ID_GO: MeshHandle = MeshHandle::new(pkg_namespace!("CountdownGo"));

    fn colors() -> Vec<[f32; 3]> {
        vec![[1., 0., 1.], [0., 1., 1.], [1., 1., 0.]]
    }

    pub fn assets(io: &mut EngineIo) {
        io.send(&UploadMesh {
            mesh: obj_lines_to_mesh(include_str!("assets/1.obj")),
            id: Self::RDR_ID_1,
        });
        io.send(&UploadMesh {
            mesh: obj_lines_to_mesh(include_str!("assets/2.obj")),
            id: Self::RDR_ID_2,
        });
        io.send(&UploadMesh {
            mesh: obj_lines_to_mesh(include_str!("assets/3.obj")),
            id: Self::RDR_ID_3,
        });
        io.send(&UploadMesh {
            mesh: obj_lines_to_mesh(include_str!("assets/go.obj")),
            id: Self::RDR_ID_GO,
        });
    }

    pub fn new(io: &mut EngineIo) -> Self {
        let entities = (0..Self::colors().len())
            .map(|_| {
                io.create_entity()
                    .add_component(Transform::default().with_rotation(Quat::from_euler(
                        EulerRot::XYZ,
                        0.,
                        -FRAC_PI_2,
                        0.,
                    )))
                    .add_component(Render::new(Self::RDR_ID_1))
                    .build()
            })
            .collect();

        Self {
            entities,
            start_time: 0.,
            needs_restart: false,
        }
    }

    pub fn restart(&mut self) {
        self.needs_restart = true;
    }

    pub fn update(&mut self, io: &mut EngineIo, time: FrameTime) {
        if self.needs_restart {
            self.start_time = time.time;
            self.needs_restart = false;
        }

        let elapsed = time.time - self.start_time;

        let rdr_component = match elapsed as i32 + 1 {
            1 => Render::new(Self::RDR_ID_3),
            2 => Render::new(Self::RDR_ID_2),
            3 => Render::new(Self::RDR_ID_1),
            _ => Render::new(Self::RDR_ID_GO),
        };

        let limit = match elapsed < 8. {
            true => None,
            false => Some(0),
        };

        let rdr_component = rdr_component.limit(limit).primitive(Primitive::Lines);

        for (&entity, color) in self.entities.iter().zip(Self::colors()) {
            io.add_component(entity, rdr_component);
            io.add_component(entity, color_extra(color));
        }
    }
}

fn color_extra([r, g, b]: [f32; 3]) -> RenderExtra {
    RenderExtra([r, g, b, 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
}
