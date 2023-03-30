use cimvr_common::{
    glam::Vec3,
    render::{MeshHandle, Primitive, Render, RenderExtra, UploadMesh},
    Transform,
};
use cimvr_engine_interface::{pkg_namespace, prelude::*, FrameTime};

use crate::obj::obj_lines_to_mesh;

pub struct CountdownAnimation {
    entities: Vec<EntityId>,
    start_time: f32,
    needs_restart: bool,
    position: Transform,
}

impl CountdownAnimation {
    const RDR_ID_1: MeshHandle = MeshHandle::new(pkg_namespace!("Countdown1"));
    const RDR_ID_2: MeshHandle = MeshHandle::new(pkg_namespace!("Countdown2"));
    const RDR_ID_3: MeshHandle = MeshHandle::new(pkg_namespace!("Countdown3"));
    const RDR_ID_GO: MeshHandle = MeshHandle::new(pkg_namespace!("CountdownGo"));

    fn colors() -> Vec<[f32; 3]> {
        vec![[1., 0., 1.], [0., 1., 1.], [1., 1., 0.]]
            .into_iter()
            //.cycle()
            //.take(3 * 4)
            .collect()
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

    pub fn new(io: &mut EngineIo, position: Transform) -> Self {
        let entities = (0..Self::colors().len())
            .map(|_| {
                io.create_entity()
                    .add_component(Transform::default())
                    .add_component(Render::new(Self::RDR_ID_1))
                    .build()
            })
            .collect();

        Self {
            position,
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

        /*
        let limit = match elapsed < 8. {
        true => None,
        false => Some(0),
        };
        */

        let rdr_component = rdr_component /*.limit(limit)*/
            .primitive(Primitive::Lines);

        for (idx, (&entity, color)) in self.entities.iter().zip(Self::colors()).enumerate() {
            let animation = Transform::identity().with_position(Vec3::new(
                idx as f32 / 3.,
                (time.time * 3. + idx as f32 / 3.).cos(),
                (time.time * 3. + idx as f32 / 3.).sin(),
            ));
            let transf = self.position * animation;
            io.add_component(entity, transf);
            io.add_component(entity, rdr_component);
            io.add_component(entity, color_extra(color));
        }
    }
}

fn color_extra([r, g, b]: [f32; 3]) -> RenderExtra {
    RenderExtra([r, g, b, 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
}
