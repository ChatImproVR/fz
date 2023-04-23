use std::f32::consts::FRAC_PI_2;

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
use cimvr_engine_interface::{pkg_namespace, prelude::*, FrameTime};
use kinematics::KinematicPhysics;

use crate::{
    controls::ship_controller,
    countdown::CountdownAnimation,
    curve::{path_mesh_to_transforms, Curve},
    kinematics,
    obj::obj_lines_to_mesh,
    shapes::grid_mesh,
    ClientIdMessage, ClientShipComponent, InputAbstraction, ServerShipComponent,
    ShipCharacteristics, ShipUpload, SHIP_RDR,
};

const ENV_OBJ: &str = include_str!("assets/loop1_env.obj");
const PATH_OBJ: &str = include_str!("assets/loop1_path.obj");

enum GameMode {
    Spectator {
        /// Which player to spectate (if any) 
        watching: Option<EntityId>,
        /// Whether the player is ready to enter the next race when it starts
        ready: bool,
    },
    Racing {
        /// ID of this client, used to ascertain
        client_id: ClientId,
        /// Lap count
        lap: usize,
        /// Laps in this course
        needed_laps: usize,
    },
}

// All state associated with client-side behaviour
pub struct ClientState {
    mode: GameMode,
    proj: Perspective,
    camera_ent: EntityId,
    ship_ent: EntityId,
    anim: CountdownAnimation,
    input_helper: InputHelper,
    input: InputAbstraction,
    motion_cfg: ShipCharacteristics,
    path: Curve,

    // TODO: This should all go in another struct
    gui: UiStateHelper,
    gui_element: UiHandle,
}

pub const MAP_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Map"));
pub const FLOOR_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Floor"));

impl UserState for ClientState {
    // Implement a constructor
    fn new(io: &mut EngineIo, sched: &mut EngineSchedule<Self>) -> Self {
        // Add environment
        io.create_entity()
            .add_component(Transform::identity())
            .add_component(Render::new(MAP_RDR).primitive(Primitive::Lines))
            .build();

        // Add floor
        io.create_entity()
            .add_component(Transform::new().with_position(Vec3::new(0., -50., 0.)))
            .add_component(Render::new(FLOOR_RDR).primitive(Primitive::Lines))
            .build();

        // Parse path mesh
        let path = Curve::new(path_mesh_to_transforms(&obj_lines_to_mesh(PATH_OBJ)));

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

        sched
            .add_system(Self::controller_input)
            .subscribe::<GamepadState>()
            .subscribe::<InputEvent>()
            .build();

        sched
            .add_system(Self::deleter)
            .subscribe::<ClientIdMessage>()
            .query(Query::new("AllServerShips")
                .intersect::<ServerShipComponent>(Access::Read)
            )
            .build();

        sched
            .add_system(Self::animation)
            .subscribe::<FrameTime>()
            .build();

        sched.add_system(Self::gui).subscribe::<UiUpdate>().build();

        let animation_pos = path.lerp(6.);
        let mut anim = CountdownAnimation::new(io, animation_pos);
        CountdownAnimation::assets(io);
        anim.restart();

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
            .query(Query::new("Kinematics")
                .intersect::<Transform>(Access::Write)
                .intersect::<KinematicPhysics>(Access::Write)
            )
            .subscribe::<FrameTime>()
            .build();

        // Add physics system
        sched
            .add_system(Self::motion_update)
            .query(Query::new("ClientShip")
                .intersect::<Transform>(Access::Write)
                .intersect::<KinematicPhysics>(Access::Write)
                .intersect::<ClientShipComponent>(Access::Read)
            )
            .subscribe::<FrameTime>()
            .build();

        sched
            .add_system(Self::camera)
            .subscribe::<InputEvent>()
            .subscribe::<VrUpdate>()
            .subscribe::<FrameTime>()
            .query(Query::new("ClientShip")
                .intersect::<Transform>(Access::Write)
                .intersect::<ClientShipComponent>(Access::Write)
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
        let gui_element = gui.add(io, "FZ", schema, init_state);

        let mode = GameMode::Spectator { watching: None, ready: false };

        Self {
            mode,
            motion_cfg,
            input: InputAbstraction::default(),
            path,
            proj: Perspective::new(),
            input_helper,
            anim,
            camera_ent,
            ship_ent,
            gui,
            gui_element,
        }
    }
}

impl ClientState {
    fn gui(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {

        let mut true_bool = true;
        let ready_state = match &mut self.mode {
            GameMode::Spectator { ready, .. } => ready,
            GameMode::Racing { .. } => &mut true_bool
        };

        self.gui.download(io);
        if self.gui.read(self.gui_element)[0] != (State::Button { clicked: false }) {
            *ready_state = !*ready_state;
            self.gui.modify(io, self.gui_element, |ui_state| {
                let text = match ready_state {
                    true => "Ready!".into(),
                    false => "(Not ready)".into(),
                };
                ui_state[1] = State::Label { text };
            })
        }
    }

    fn deleter(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        /*
        if let Some(my_id) = self.client_id {
            for entity in query.iter("AllServerShips") {
                let ServerShipComponent(id) = query.read(entity);
                if id == my_id {
                    io.remove_entity(entity);
                }
            }
        }

        if let Some(ClientIdMessage(id)) = io.inbox_first() {
            self.client_id = Some(id);
        }
        */
    }

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

        // Set camera pos
        if let Some(ship_ent) = query.iter("ClientShip").next() {
            let ship_transf: Transform = query.read(ship_ent);

            let cam_pos = Transform::new()
                .with_rotation(Quat::from_euler(EulerRot::XYZ, 0., -FRAC_PI_2, 0.))
                .with_position(Vec3::new(-13., 2., 0.));

            let cam_transf = ship_transf * cam_pos;

            io.add_component(self.camera_ent, cam_transf);
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

    fn motion_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        let Some(FrameTime { delta, .. }) = io.inbox_first() else { return };

        if let Some(ship_ent) = query.iter("ClientShip").next() {
            // Get current physical properties
            let mut kt: KinematicPhysics = query.read(ship_ent);
            let mut tf: Transform = query.read(ship_ent);
            //let ShipComponent(client_id) = query.read(ship_ent);

            // Step ship forward in time
            ship_controller(
                delta,
                self.motion_cfg,
                self.input,
                &self.path,
                &mut tf,
                &mut kt,
            );

            query.write(ship_ent, &kt);
            query.write(ship_ent, &tf);

            io.send(&ShipUpload(tf, kt));
        }
    }

    /// Simulate kinematics
    fn kinematics_update(&mut self, io: &mut EngineIo, query: &mut QueryResult) {
        let Some(FrameTime { delta, .. }) = io.inbox_first() else { return };
        kinematics::simulate(query, delta);
    }
}
