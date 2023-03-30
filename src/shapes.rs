use cimvr_common::render::{Mesh, Vertex};

pub fn grid_mesh(n: i32, scale: f32, color: [f32; 3]) -> Mesh {
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
