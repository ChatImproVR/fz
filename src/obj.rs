use cimvr_common::render::{Mesh, Vertex};
use cimvr_engine_interface::{dbg, prelude::*};
use std::{io::Read, str::FromStr, collections::{HashSet, HashMap}};

/// Read OBJ lines into the mesh
pub fn obj_lines_to_mesh(obj: &str) -> Mesh {
    let mut m = Mesh::new();

    for line in obj.lines() {
        // Split the line by whitespace
        let mut line = line.split_whitespace();

        // Break the first bit off
        let (first, mut rest) = (line.next(), line);

        // Which kind of line is it?
        match first {
            Some("v") => {
                // Treat the line as two arrays of 3 elements
                let mut parts = [[0.; 3], [1.; 3]];

                for part in &mut parts {
                    // Get strings from the rest of the line
                    for dim in part {
                        let Some(text) = rest.next() else { break };
                        *dim = text.parse().expect("Invalid float");
                    }
                }

                // Split the parts back up
                let [pos, uvw] = parts;

                // Assemble the vertex
                m.vertices.push(Vertex { pos, uvw });
            }
            Some("l") => {
                // Do the same for indices
                let mut indices = [0; 2];
                for dim in &mut indices {
                    let Some(text) = rest.next() else { break };
                    *dim = text.parse().expect("Invalid index");

                    // OBJ files are one-indexed
                    *dim -= 1;
                }
                m.indices.extend(indices);
            }
            // Ignore the rest
            _ => (),
        }
    }

    m
}
