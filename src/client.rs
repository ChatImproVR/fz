use std::f32::consts::FRAC_PI_2;

use chat::{ChatDownload, ChatUpload};
use cimvr_common::{
    desktop::{InputEvent, KeyCode},
    gamepad::{Axis, Button, GamepadState},
    glam::{EulerRot, Quat, Vec3},
    render::{CameraComponent, MeshHandle, Primitive, Render, UploadMesh},
    ui::{Schema, State, UiHandle, UiStateHelper, UiUpdate},
    utils::{camera::Perspective, input_helper::InputHelper},
    vr::VrUpdate,
    Transform,
};
use cimvr_engine_interface::{dbg, pkg_namespace, prelude::*, FrameTime};
use kinematics::KinematicPhysics;

use crate::{
    controls::ship_controller,
    countdown::CountdownAnimation,
    curve::{path_mesh_to_transforms, Curve},
    kinematics,
    obj::obj_lines_to_mesh,
    shapes::grid_mesh,
    ClientReady, ClientShipComponent, Finished, InputAbstraction, ServerShipComponent,
    ShipCharacteristics, ShipUpload, StartRace, SHIP_RDR,
};

// TODO: This is a dumb thing to hardcode lol
const N_LAPS: usize = 3;
const ENV_OBJ: &str = include_str!("assets/loop1_env.obj");
const PATH_OBJ: &str = include_str!("assets/loop1_path.obj");

enum GameMode {
    Spectator {
        /// Which player to spectate (if any)
        watching: Option<ClientId>,
        /// Whether the player is ready to enter the next race when it starts
        ready: bool,
    },
    Racing {
        /// ID of this client, used to ascertain
        client_id: ClientId,
        /// Lap count
        lap: usize,
    },
}

// All state associated with client-side behaviour
pub struct ClientState {
    mode: GameMode,
    proj: Perspective,
    camera_ent: EntityId,
    ship_ent: EntityId,
    countdown: CountdownAnimation,
    input_helper: InputHelper,
    input: InputAbstraction,
    motion_cfg: ShipCharacteristics,
    path: Curve,
    last_ship_pos: Transform,

    // TODO: This should all go in another struct
    gui: UiStateHelper,
    ready_state_element: UiHandle,
}

pub const MAP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Map"));
pub const FLOOR_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Floor"));
pub const FINISH_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("FinishLine"));

const FINISH_LINE_INDEX: f32 = 10.;

fn finish_line_pos(curve: &Curve) -> Transform {
    curve.lerp(FINISH_LINE_INDEX)
}

impl UserState for ClientState {
    // Implement a constructor
    fn new(io: &mut EngineIo, sched: &mut EngineSchedule<Self>) -> Self {
        // Parse path mesh
        let path = Curve::new(path_mesh_to_transforms(&obj_lines_to_mesh(PATH_OBJ)));

        // Add environment
        io.create_entity()
            .add_component(Transform::identity())
            .add_component(Render::new(MAP_RDR).primitive(Primitive::Lines))
            .build();

        // Add finish line
        io.create_entity()
            .add_component(finish_line_pos(&path))
            .add_component(Render::new(FINISH_RDR).primitive(Primitive::Lines))
            .build();

        // Add floor
        io.create_entity()
            .add_component(Transform::new().with_position(Vec3::new(0., -50., 0.)))
            .add_component(Render::new(FLOOR_RDR).primitive(Primitive::Lines))
            .build();

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

        let mut finish_line_mesh = obj_lines_to_mesh(include_str!("assets/finish_line.obj"));
        finish_line_mesh
            .vertices
            .iter_mut()
            .for_each(|v| v.uvw = [1., 0., 0.]);

        // Upload finishline
        io.send(&UploadMesh {
            mesh: finish_line_mesh,
            id: FINISH_RDR,
        });

        // Add camera
        let camera_ent = io
            .create_entity()
            .add_component(Transform::identity())
            .add_component(CameraComponent::default())
            .build();

        sched
            .add_system(Self::controller_input)
            .subscribe::<GamepadState>()
            .subscribe::<InputEvent>()
            .build();

        sched
            .add_system(Self::game_mode)
            .subscribe::<StartRace>()
            .build();

        sched
            .add_system(Self::deleter)
            .subscribe::<StartRace>()
            .query(
                "AllServerShips",
                Query::new().intersect::<ServerShipComponent>(Access::Read),
            )
            .build();

        sched
            .add_system(Self::animation)
            .subscribe::<FrameTime>()
            .build();

        sched.add_system(Self::gui).subscribe::<UiUpdate>().build();

        let animation_pos = path.lerp(6.);
        let mut countdown = CountdownAnimation::new(io, animation_pos);
        CountdownAnimation::assets(io);

        let input_helper = InputHelper::new();

        let ship_ent = io
            .create_entity()
            .add_component(Transform::identity())
            .add_component(Render::new(SHIP_RDR).primitive(Primitive::Lines))
            .add_component(ClientShipComponent)
            .add_component(KinematicPhysics {
                vel: Vec3::ZERO,
                mass: 1.,
                ang_vel: Vec3::ZERO,
                moment: 1.,
            })
            .build();

        // Add physics system
        sched
            .add_system(Self::kinematics_update)
            .query(
                "Kinematics",
                Query::new()
                    .intersect::<Transform>(Access::Write)
                    .intersect::<KinematicPhysics>(Access::Write),
            )
            .subscribe::<FrameTime>()
            .build();

        // Add motion control system
        sched
            .add_system(Self::motion_update)
            .query(
                "ClientShip",
                Query::new()
                    .intersect::<Transform>(Access::Write)
                    .intersect::<KinematicPhysics>(Access::Write)
                    .intersect::<ClientShipComponent>(Access::Read),
            )
            .query(
                "ServerShips",
                Query::new().intersect::<ServerShipComponent>(Access::Read),
            )
            .subscribe::<FrameTime>()
            .build();

        sched
            .add_system(Self::camera)
            .subscribe::<InputEvent>()
            .subscribe::<VrUpdate>()
            .subscribe::<FrameTime>()
            .query(
                "ClientShip",
                Query::new()
                    .intersect::<Transform>(Access::Write)
                    .intersect::<ClientShipComponent>(Access::Write),
            )
            .query(
                "ServerShips",
                Query::new()
                    .intersect::<Transform>(Access::Read)
                    .intersect::<ServerShipComponent>(Access::Read),
            )
            .build();

        // For editing ui: sends schema implicitly
        io.create_entity()
            .add_component(ServerShipComponent::default())
            .build();

        // Define ship capabilities
        let motion_cfg = ShipCharacteristics {
            mass: 1000.,
            moment: 1000. * 3_f32.powi(2),
            max_twirl: 5.,
            max_impulse: 30.,
        };

        let mut gui = UiStateHelper::new();
        let schema = vec![
            Schema::Button {
                text: "Toggle Ready".into(),
            },
            Schema::Label,
        ];
        let init_state = vec![
            State::Button { clicked: false },
            State::Label {
                text: "(Not ready)".into(),
            },
        ];
        let ready_state_element = gui.add(io, "FZ", schema, init_state);

        let mode = GameMode::Spectator {
            watching: None,
            ready: false,
        };

        Self {
            mode,
            motion_cfg,
            input: InputAbstraction::default(),
            path,
            proj: Perspective::new(),
            input_helper,
            countdown,
            camera_ent,
            ship_ent,
            gui,
            last_ship_pos: Transform::default(),
            ready_state_element,
        }
    }
}

impl ClientState {
    fn gui(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {
        // Toggle ready state based on UI interaction
        if let GameMode::Spectator { ready, .. } = &mut self.mode {
            self.gui.download(io);
            let clicked =
                self.gui.read(self.ready_state_element)[0] != (State::Button { clicked: false });
            if clicked {
                *ready = !*ready;
            }

            let ready_text = match ready {
                true => "Ready!".to_string(),
                false => "(Not ready)".to_string(),
            };

            self.gui.modify(io, self.ready_state_element, |ui_state| {
                let text = ready_text.clone();
                ui_state[1] = State::Label { text };
            });

            if clicked {
                io.send(&ClientReady(*ready));
                io.send(&ChatUpload(ready_text));
            }
        }
    }

    fn deleter(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        if let GameMode::Racing { client_id, .. } = self.mode {
            for ship_entity in query.iter("AllServerShips") {
                let ServerShipComponent {
                    client_id: ships_id,
                    is_racing,
                    ..
                } = query.read(ship_entity);

                if ships_id == client_id || !is_racing {
                    io.remove_entity(ship_entity);
                }
            }
        }
    }

    fn animation(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {
        let Some(time) = io.inbox_first::<FrameTime>() else { return };
        self.countdown.update(io, time);
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

        let camera_tf = match &mut self.mode {
            GameMode::Racing { .. } => Self::camera_trail_behind(query),
            GameMode::Spectator { watching, .. } => Self::camera_spectate(query, watching),
        };

        io.add_component(self.camera_ent, camera_tf);
    }

    fn camera_spectate(query: &mut QueryResult, watching: &mut Option<ClientId>) -> Transform {
        // Find someone to watch
        if watching.is_none() {
            for entity in query.iter("ServerShips") {
                let shipc = query.read::<ServerShipComponent>(entity);
                //if shipc.is_racing {
                *watching = Some(shipc.client_id);
                //}
            }
        }

        // Watch from their ship's perspective
        let mut pos = Transform::default();
        for entity in query.iter("ServerShips") {
            let shipc = query.read::<ServerShipComponent>(entity);
            let tf = query.read::<Transform>(entity);
            if Some(shipc.client_id) == *watching {
                pos = tf;
            }
        }

        pos * Self::cam_offset()
    }

    fn cam_offset() -> Transform {
        Transform::new()
            .with_rotation(Quat::from_euler(EulerRot::XYZ, 0., -FRAC_PI_2, 0.))
            .with_position(Vec3::new(-13., 2., 0.))
    }

    fn camera_trail_behind(query: &mut QueryResult) -> Transform {
        // Set camera pos
        if let Some(ship_ent) = query.iter("ClientShip").next() {
            let ship_transf: Transform = query.read(ship_ent);

            ship_transf * Self::cam_offset()
        } else {
            Transform::new()
        }
    }

    fn controller_input(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {
        self.input = InputAbstraction::default();

        if let Some(GamepadState(gamepads)) = io.inbox_first() {
            if let Some(gamepad) = gamepads.into_iter().next() {
                self.input.yaw = gamepad.axes[&Axis::RightStickX];
                self.input.pitch = gamepad.axes[&Axis::LeftStickY];
                self.input.roll = gamepad.axes[&Axis::LeftStickX];
                self.input.throttle = gamepad.axes[&Axis::RightStickY];
                if gamepad.buttons[&Button::RightTrigger2] {
                    self.input.throttle = 1.;
                }
                if gamepad.buttons[&Button::LeftTrigger2] {
                    self.input.throttle = -1.;
                }
            }
        }

        self.input_helper.handle_input_events(io);

        if self.input_helper.key_held(KeyCode::W) {
            self.input.throttle = 1.0;
        }

        if self.input_helper.key_held(KeyCode::S) {
            self.input.throttle = -1.0;
        }

        if self.input_helper.key_held(KeyCode::A) {
            self.input.roll = -1.0;
        }

        if self.input_helper.key_held(KeyCode::D) {
            self.input.roll = 1.0;
        }
    }

    fn game_mode(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {
        if let Some(StartRace {
            client_id,
            position,
        }) = io.inbox_first()
        {
            self.mode = GameMode::Racing { client_id, lap: 0 };

            self.countdown.restart();

            // Reset ship position
            io.add_component(self.ship_ent, position);
            self.last_ship_pos = position;
        }
    }

    fn motion_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        let Some(time) = io.inbox_first() else { return };
        let FrameTime { delta, .. } = time;

        let is_racing = matches!(self.mode, GameMode::Racing { .. });
        let match_started = self.countdown.match_started(time);
        let should_be_moving = is_racing && match_started;

        let Some(ship_ent) = query.iter("ClientShip").next() else { return };

        // Get current physical properties
        let mut kt: KinematicPhysics = query.read(ship_ent);
        let mut tf: Transform = query.read(ship_ent);
        //let ShipComponent(client_id) = query.read(ship_ent);

        // Step ship forward in time
        if should_be_moving {
            ship_controller(
                delta,
                self.motion_cfg,
                self.input,
                &self.path,
                &mut tf,
                &mut kt,
            );
        } else {
            kt.vel = Vec3::ZERO;
            kt.ang_vel = Vec3::ZERO;
        }

        io.send(&ShipUpload(tf, kt));

        query.write(ship_ent, &kt);
        query.write(ship_ent, &tf);

        // Check if we've crossed the finish line
        let area_sanity_check =
            (self.path.nearest_ctrlp(tf.pos) as i32 - FINISH_LINE_INDEX as i32).abs() < 3;
        let finish_line = finish_line_pos(&self.path);
        let cross_over = (finish_line.inverse() * self.last_ship_pos).pos.x < 0.
            && (finish_line.inverse() * tf).pos.x > 0.;
        if area_sanity_check && cross_over {
            if let GameMode::Racing { lap, .. } = &mut self.mode {
                if *lap != 9 {
                    let time = self.countdown.elapsed(time);
                    let minutes = (time / 60.).floor();
                    let seconds = (time % 60.).floor();
                    let milliseconds = ((time % 60.).fract() * 1000.).floor();
                    io.send(&ChatUpload(format!(
                        "Lap {lap}, time: {minutes}:{seconds}:{milliseconds}"
                    )))
                }

                *lap += 1;

                // We've finisehd the whole race!
                if *lap > N_LAPS {
                    io.send(&Finished(self.countdown.elapsed(time)));

                    self.mode = GameMode::Spectator {
                        watching: None,
                        ready: false,
                    };
                }
            }
        }

        self.last_ship_pos = tf;
    }

    /// Simulate kinematics
    fn kinematics_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        let Some(FrameTime { delta, .. }) = io.inbox_first() else { return };
        kinematics::simulate(query, delta);
    }
}
