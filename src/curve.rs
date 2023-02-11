use cimvr_common::nalgebra::{Point3, Vector3};

pub struct Curve(pub Vec<ControlPoint>);

pub struct ControlPoint {
    pub dir: Vector3<f32>,
    pub pos: Point3<f32>,
}

struct QuadBezier {
    pub a: Point3<f32>,
    pub b: Point3<f32>,
    pub c: Point3<f32>,
    pub d: Point3<f32>,
}

impl Curve {
    fn get_quad(&self, t: f32) -> QuadBezier {
        assert!(t < self.0.len() as f32);
        assert!(t >= 0.);

        let i = t.floor() as usize;
        let Self(ctrlp) = self;
        QuadBezier {
            a: ctrlp[i].pos,
            b: ctrlp[i].pos + ctrlp[i].dir,
            c: ctrlp[i + 1].pos - ctrlp[i].dir,
            d: ctrlp[i + 1].pos,
        }
    }

    pub fn interp(&self, t: f32) -> Point3<f32> {
        self.get_quad(t).interp(t.fract())
    }

    pub fn deriv(&self, t: f32) -> Vector3<f32> {
        self.get_quad(t).deriv(t.fract())
    }
}

impl QuadBezier {
    pub fn interp(&self, t: f32) -> Point3<f32> {
        let neg = 1. - t;
        neg.powi(3) * t.powi(0) * self.a
            + neg.powi(2) * t.powi(1) * self.b.coords
            + neg.powi(1) * t.powi(2) * self.c.coords
            + neg.powi(0) * t.powi(3) * self.d.coords
    }

    pub fn deriv(&self, t: f32) -> Vector3<f32> {
        let neg = 1. - t;
        3. * neg.powi(2) * t.powi(0) * (self.b - self.a)
            + 6. * neg.powi(1) * t.powi(1) * (self.c - self.b)
            + 3. * neg.powi(0) * t.powi(2) * (self.d - self.c)
    }
}
