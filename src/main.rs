/*

TODO: notate how many top-level and how many incorporated
TODO: paper piecing templates

 */

#[macro_use]
extern crate error_chain;

use phf::phf_map;

mod errors {

    error_chain!{

        foreign_links {
            Fmt(::std::fmt::Error);
            Io(::std::io::Error) #[cfg(unix)];
            Cairo(::cairo::Error);
        }

    }

}

use errors::*;

use std::path::Path;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::collections::HashMap;
use std::collections::HashSet;
use std::collections::hash_map::Entry::{Occupied, Vacant};


const PHI: f64 = 1.618033988749895;
const INVPHI: f64 = 0.618033988749895;

//const HALF_APEX_ANGLE: f64 = 36.*std::f64::consts::PI/180.0;

const COS_HALF_APEX: f64 = 0.8090169943749475;
const SIN_HALF_APEX: f64 = 0.5877852522924731;

const INCH: f64 = 72.0;

const PAGE_SHORT_EDGE: f64 = 8.5 * INCH;
const PAGE_LONG_EDGE: f64 = 11.0 * INCH;

const MARGIN: f64 = 0.5 * INCH;

const COLORS: [[f64; 3]; 4] = [
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 1.0],
    [1.0, 1.0, 0.0],
    [0.0, 1.0, 0.0]
];

//////////////////////////////////////////////////////////////////////

type Vec2d = nalgebra::Vector2<f64>;
type Vec3d = nalgebra::Vector3<f64>;
type Point2d = nalgebra::geometry::Point2<f64>;
type Similarity2d = nalgebra::Similarity2<f64>;
type Translation2d = nalgebra::Translation2<f64>;
type Rotation2d = nalgebra::Rotation2<f64>;
type Transform2d = nalgebra::Transform2<f64>;
type Matrix3d = nalgebra::Matrix3<f64>;

struct Rect2d {

    p0: Point2d,
    p1: Point2d

}

impl Rect2d {
    
    fn empty() -> Self {

        let p0 = Point2d::new(f64::MAX, f64::MAX);
        let p1 = -p0;

        Rect2d { p0: p0, p1: p1 }
        
    }

    fn new(p0: Point2d, p1: Point2d) -> Self {

        Rect2d { p0: p0, p1: p1 }

    }

    fn expand(&mut self, p: &Point2d) {

        self.p0 = self.p0.inf(p);
        self.p1 = self.p1.sup(p);
        
    }

    fn dims(&self) -> Vec2d {
        self.p1 - self.p0
    }

    fn center(&self) -> Point2d {
        self.p0 + 0.5*(self.p1 - self.p0)
    }
        

}

struct Box2d {
    half_dims: Vec2d
}

fn intersect_hline(u: f64, v: f64, 
                   xmid: f64, ymid: f64,
                   hx: f64, hy: f64) -> bool {

    // horizontal line segment from (-u, v) to (u, v)
    // arbitrary line segment from (x0, y0) to (x1, y1)

    let contains_v = (ymid - v).abs() < hy.abs();

    if !contains_v {

        false 

    } else {

        let x = xmid + (v - ymid) * hx / hy;

        (x.abs() < u) && ((xmid - x).abs() < hx.abs())

    }
    

}

impl Box2d {

    fn new(half_dims: Vec2d) -> Self {
        Box2d { half_dims: half_dims }
    }

    fn contains_point(&self, p: &Point2d) -> bool {
        p.iter().enumerate().all(
            |(i, &v)| v.abs() <= self.half_dims[i]
        )
    }

    fn intersects_segment(&self, a: &Point2d, b: &Point2d) -> bool {

        let h = 0.5*(b - a);
        let c = a + h;

        let r = &self.half_dims;

        intersect_hline(r.x,  r.y, c.x, c.y, h.x, h.y) ||
            intersect_hline(r.x, -r.y, c.x, c.y, h.x, h.y) ||
            intersect_hline(r.y,  r.x, c.y, c.x, h.y, h.x) ||
            intersect_hline(r.y, -r.x, c.y, c.x, h.y, h.x)
        
    }

    fn overlaps_tri(&self, v0: &Point2d, v1: &Point2d, v2: &Point2d) -> bool {

        self.contains_point(v0) ||
            self.contains_point(v1) ||
            self.contains_point(v2) ||
            self.intersects_segment(v0, v1) ||
            self.intersects_segment(v1, v2) || 
            self.intersects_segment(v2, v0)
        
    }


}



fn get_page_transform(verts_rect: &Rect2d, page_rect: &Rect2d, hsign: f64) -> (Transform2d, f64)

{


    let vdims = verts_rect.dims();
    let pdims = page_rect.dims();

    let scl = (pdims.component_div(&vdims)).min();

    let vmid = verts_rect.center();
    let pmid = page_rect.center();

    let translate_page = Translation2d::new(pmid[0], pmid[1]);
    
    let scale = Transform2d::from_matrix_unchecked(
        Matrix3d::new(
            scl*hsign, 0.0, 0.0,
            0.0, -scl, 0.0,
            0.0, 0.0, 1.0
        )
    );

    let translate_verts = Translation2d::new(-vmid[0], -vmid[1]);

    let transform = translate_page * scale * translate_verts;

    (transform, scl)
    
}

fn line(a: &Point2d, b: &Point2d) -> Vec3d {

    let a = a.to_homogeneous();
    let b = b.to_homogeneous();
    
    let l = a.cross(&b);

    let p = (l[0]*l[0] + l[1]*l[1]).sqrt();

    l / p

}

fn intersect(a: &Vec3d, b: &Vec3d) -> Result<Point2d> {
    match Point2d::from_homogeneous(a.cross(b)) {
        None => bail!("lines don't intersect!"),
        Some(point) => Ok(point)
    }
}

fn tri_center(v0: &Point2d, 
              v1: &Point2d,
              v2: &Point2d) -> Result<(Point2d, f64)> {

    let l01 = line(v0, v1);
    let l12 = line(v1, v2);
    let l20 = line(v2, v0);

    let b1 = l01 - l12;
    let b2 = l12 - l20;

    let pinter = intersect(&b1, &b2)?;

    Ok((pinter, pinter.to_homogeneous().dot(&l01).abs()))

}

fn offset_convex_poly(verts: &Vec<Point2d>, dist: f64, snip_corners: bool) -> Result<Vec<Point2d>> {

    let n = verts.len();

    assert!(n > 2);

    let mut lines = Vec::new();

    for (idx, v0) in verts.iter().enumerate() {
        let v1 = &verts[(idx + 1) % n];
        lines.push(line(v0, v1));
    }

    let s = if lines[0].dot(&verts[2].to_homogeneous()) < 0.0 { -1.0 } else { 1.0 };
    for line in &mut lines {
        line[2] += s * dist;
    }

    let mut rval = Vec::new();

    for (idx, l1) in lines.iter().enumerate() {

        let l0 = &lines[(idx + n - 1) % n];

        if snip_corners {

            let mut bisector = l1-l0;
            bisector /= Vec2d::new(bisector.x, bisector.y).norm();

            let tangent = s * Vec2d::new(bisector.y, -bisector.x);

            let v_orig = &verts[idx];
            
            let mut lbisect = Vec3d::new(tangent.x, tangent.y, 
                                         -tangent.dot(&Vec2d::new(v_orig.x, v_orig.y)));
            lbisect[2] += s*dist;
            
            let v1 = intersect(l0, &lbisect)?;
            let v2 = intersect(&lbisect, l1)?;

            rval.push(v1);
            rval.push(v2);

        } else {

            let v_offset = intersect(l0, l1)?;
            rval.push(v_offset);

        }

    }

    Ok(rval)
    

}

fn tri_border(v0: &Point2d, 
              v1: &Point2d,
              v2: &Point2d, 
              dist: f64,
              dilate: f64) -> Result<(Point2d, Point2d, Point2d, Point2d)> {


    let l01 = line(v0, v1);
    let l12 = line(v1, v2);
    let l20 = line(v2, v0);

    let s = if l01.dot(&v2.to_homogeneous()) < 0.0 { -1.0 } else { 1.0 };

    let odilate = Vec3d::new(0.0, 0.0, s*dilate);
    let odist = Vec3d::new(0.0, 0.0, -s*dist);

    let l01 = l01 + odilate;
    let l12 = l12 + odilate;
    let l20 = l20 + odilate;

    let k01 = l01 + odist;
    let k12 = l12 + odist;

    let v0 = intersect(&k01, &l20)?;
    let v1 = intersect(&k01, &k12)?;
    let v2 = intersect(&k12, &l20)?;
    let v3 = intersect(&k12, &l01)?;

    Ok((v0, v1, v2, v3))

}

          

#[derive(Debug, PartialEq, PartialOrd, Eq, Ord, Clone, Copy, Hash)]
enum TriangleType {
    KiteTriangle,
    DartTriangle
}

#[derive(Debug, PartialEq, PartialOrd, Eq, Ord, Clone, Copy, Hash)]
enum TriangleSide {
    SideLeft,
    SideRight
}

impl TriangleSide {

    fn flip(&self) -> Self {
        match &self {
            TriangleSide::SideLeft => TriangleSide::SideRight,
            TriangleSide::SideRight => TriangleSide::SideLeft
        }
    }

}


fn get_cidx(ttype: TriangleType,
            tside: TriangleSide) -> usize {

    2*((ttype == TriangleType::DartTriangle) as usize) +
        ((tside == TriangleSide::SideLeft) as usize)
        
}

fn get_tri_string(ttype: TriangleType,
                  tside: TriangleSide,
                  level: usize) -> String {
    
    let i = get_cidx(ttype, tside);
    let abcd = "ABCD";
    let slice: &str = &abcd[i..i+1];
    

    format!("{:}{:}", slice, level+1)

}

#[derive(Debug, PartialEq, Clone)]
struct PTriangle {

    indices:    [usize; 3],
    ttype: TriangleType,
    tside: TriangleSide,
    parent:     Option<usize>,
    children:   Vec<usize>,
    generation: usize

}

impl PTriangle {

    fn get_verts<'a, 'b>(&'a self, verts: &'b Vec<Point2d>) -> (&'b Point2d, &'b Point2d, &'b Point2d) {
        (&verts[self.indices[0]],
         &verts[self.indices[1]],
         &verts[self.indices[2]])
    }

    fn has_vertex(&self, vidx: usize) -> bool {
        self.indices[0] == vidx ||
            self.indices[1] == vidx ||
            self.indices[2] == vidx
    }


}


#[derive(Debug)]
struct PTriangulation {
    
    verts: Vec<Point2d>,
    vparents: Vec<Option<(usize, usize)>>,
    vertex_subdiv_lookup: HashMap<(usize, usize), usize>,
    tris: Vec<PTriangle>,
    generations: Vec<usize>,
    tri_subdiv_lookup: HashMap<Option<usize>, Vec<(usize, usize, usize, usize)>>

}

#[derive(Debug, PartialEq, PartialOrd, Eq, Ord, Clone, Copy)]
enum SeamPart {
    
    Empty,
    BaseTri((TriangleType, TriangleSide, usize)),
    SeamIndex(usize)

}

impl std::fmt::Display for SeamPart {


    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {

        match self {
            SeamPart::Empty => write!(f, "[Empty SeamPart]"),
            SeamPart::BaseTri((ttype, tsize, level)) => write!(f, "{}", get_tri_string(*ttype, *tsize, *level)),
            SeamPart::SeamIndex(i) => write!(f, "assembly{}", i+1)
        }
            
    }


}


impl PTriangulation {

    fn get_child_leafs(&self) -> Vec<Vec<usize>> {

        let mut child_leafs = vec![ vec![] ; self.tris.len() ];

        for (orig_tidx, _) in self.final_gen_tris_enumerated() {
            let mut tidx = orig_tidx;
            loop { 
                child_leafs[tidx].push(orig_tidx);
                match self.tris[tidx].parent {
                    None => { break; }
                    Some(parent_tidx) => { tidx = parent_tidx; }
                }
            }
        }

        child_leafs

    }

    fn add_triangle(&mut self,
                    indices: [usize; 3],
                    ttype: TriangleType,
                    tside: TriangleSide,
                    tidx_parent: usize,
                    generation: usize) -> usize {

        let tidx_new = self.tris.len();

        self.tris[tidx_parent].children.push(tidx_new);
        
        self.tris.push( PTriangle {
            indices: indices,
            ttype: ttype,
            tside: tside,
            parent: Some(tidx_parent),
            children: Vec::new(),
            generation: generation
        } );

        tidx_new

    }

    fn split_triangle(&mut self,
                      parent: Option<usize>, 
                      vidx0: usize, vidx1: usize,
                      tidx0: usize, tidx1: usize) {

        let (vidx0, vidx1) = if vidx0 <= vidx1 {
            (vidx0, vidx1)
        } else {
            (vidx1, vidx0)
        };

        debug_assert!(vidx0 < vidx1);
        debug_assert!(tidx0 < tidx1);
        
        debug_assert!(self.tris[tidx0].parent == parent);
        debug_assert!(self.tris[tidx1].parent == parent);

        debug_assert!(self.tris[tidx0].has_vertex(vidx0));
        debug_assert!(self.tris[tidx0].has_vertex(vidx1));
        debug_assert!(self.tris[tidx1].has_vertex(vidx0));
        debug_assert!(self.tris[tidx1].has_vertex(vidx1));

        /*
        debug_assert!(match parent {
            None => true,
            Some(tidx_parent) => (
                self.vertex_along_tri(vidx0, tidx_parent) &&
                self.vertex_along_tri(vidx1, tidx_parent)
            )
        });
        */

        self.tri_subdiv_lookup.entry(parent).or_insert(vec![]).push(
            (vidx0, vidx1, tidx0, tidx1) );
        
    }

    fn split_edge(&mut self, vidx0: usize, vidx1: usize) -> usize {

        //println!("splitting edge {:} -> {:}", vidx0, vidx1);
        
        let e = self.vertex_subdiv_lookup.entry( (vidx0, vidx1) );

        match e {

            Occupied(o) => {
                //println!("...already have {:}", o.get());
                *o.get()
            }

            Vacant(v) => {

                let vidx2 = self.verts.len();

                let p0 = self.verts[vidx0];
                let p1 = self.verts[vidx1];

                let pnew = p0 + INVPHI*(p1 - p0);
                self.verts.push(pnew);

                self.vparents.push(Some((vidx0, vidx1)));

                //println!("...made new vertex {:}", vidx2);

                v.insert(vidx2);

                vidx2

            }

        }

    }


    fn half_kite() -> PTriangulation {

        let p0 = Point2d::new(0.0, 0.0);
        let p1 = Point2d::new(PHI*COS_HALF_APEX, PHI*SIN_HALF_APEX);
        let p2 = Point2d::new(PHI, 0.0);

        let tri = PTriangle {
            indices: [0, 1, 2],
            ttype: TriangleType::KiteTriangle,
            tside: TriangleSide::SideRight,
            parent: None,
            children: Vec::new(),
            generation: 0
        };

        PTriangulation {
            
            verts: vec![p0, p1, p2],
            vparents: vec![None, None, None],
            vertex_subdiv_lookup: HashMap::new(),
            tris: vec![tri],
            generations: vec![0],
            tri_subdiv_lookup: HashMap::new()
                
            
        }
        
    }

    fn half_dart() -> PTriangulation {

        let p0 = Point2d::new(0.0, 0.0);
        let p1 = Point2d::new(PHI*COS_HALF_APEX, PHI*SIN_HALF_APEX);
        let p2 = Point2d::new(1.0, 0.0);

        let tri = PTriangle {
            indices: [0, 1, 2],
            ttype: TriangleType::DartTriangle,
            tside: TriangleSide::SideRight,
            parent: None,
            children: Vec::new(),
            generation: 0
        };

        PTriangulation {
            
            verts: vec![p0, p1, p2],
            vparents: vec![None, None, None],
            vertex_subdiv_lookup: HashMap::new(),
            tris: vec![tri],
            generations: vec![0],
            tri_subdiv_lookup: HashMap::new()
           
        }
        
    }

    fn kite() -> PTriangulation {
        
        let mut pt = Self::half_kite();

        let p1 = &pt.verts[1];

        let p3 = Point2d::new(p1.x, -p1.y);

        pt.verts.push(p3);
        pt.vparents.push(None);

        for v in pt.verts.iter_mut() {
            *v = Point2d::new(v[1], -v[0]);
        }

        pt.tris.push( PTriangle {
            indices: [0, 3, 2],
            ttype: TriangleType::KiteTriangle,
            tside: TriangleSide::SideLeft,
            parent: None,
            children: Vec::new(),
            generation: 0
        } );

        pt.split_triangle(None, 0, 2, 0, 1);

        pt
        
    }


    fn final_gen_tris(&self) -> (usize, &[PTriangle]) {

        let gfinal = self.generations.len()-1;
        let tidx_first_gfinal = self.generations[gfinal];

        debug_assert!(self.tris.last().unwrap().generation == gfinal);
        debug_assert!(tidx_first_gfinal == 0 || self.tris[tidx_first_gfinal-1].generation == gfinal - 1);

        (tidx_first_gfinal, &self.tris[tidx_first_gfinal..])

    }

    fn final_gen_tris_enumerated<'a>(&'a self) -> impl Iterator<Item=(usize, &'a PTriangle)> {

        let (tidx_first_gfinal, slice) = self.final_gen_tris();
        
        slice.iter().enumerate().map(move |(i, t)| (i + tidx_first_gfinal, t))

    }
    
    fn vertex_along_edge(&self, vidx: usize, vidx0: usize, vidx2: usize) -> bool {

        debug_assert!(self.verts.len() == self.vparents.len());

        if vidx == vidx0 || vidx == vidx2 {

            true

        } else {

            if let Some(&vidx1) = self.vertex_subdiv_lookup.get(&(vidx0, vidx2)) {

                debug_assert!(!self.vertex_subdiv_lookup.contains_key(&(vidx2, vidx0)));
                self.vertex_along_edge(vidx, vidx0, vidx1) ||
                    self.vertex_along_edge(vidx, vidx1, vidx2)

            } else if let Some(&vidx1) = self.vertex_subdiv_lookup.get(&(vidx2, vidx0)) {

                self.vertex_along_edge(vidx, vidx2, vidx1) ||
                    self.vertex_along_edge(vidx, vidx1, vidx0)

            } else {

                false

            }

        }

    }

    fn subdivide(&mut self) {

        if self.tris.is_empty() || self.generations.is_empty() {
            return;
        }

        let (tidx_first_gfinal, last_tris) = self.final_gen_tris();

        //println!("at start of subdivide got {:} verts, {:} tris",
        //self.verts.len(), self.tris.len());

        let gnew = self.generations.len();

        let last_tris = last_tris.to_vec();
        
        let mut new_tris = vec![];

        self.generations.push(self.tris.len());

        for (tidx_offset, tri) in last_tris.iter().enumerate() {

            debug_assert!(tri.generation == gnew - 1);

            let vidx0 = tri.indices[0];
            let vidx1 = tri.indices[1];
            let vidx2 = tri.indices[2];

            let flip = tri.tside.flip();
            let same = tri.tside;

            let tidx = tidx_offset + tidx_first_gfinal;

            match tri.ttype {

                TriangleType::KiteTriangle => {

                    let vidx3 = self.split_edge(vidx0, vidx2);
                    let vidx4 = self.split_edge(vidx1, vidx0);

                    let a = self.add_triangle([vidx0, vidx3, vidx4],
                                              TriangleType::DartTriangle, flip,
                                              tidx, gnew);

                    let b = self.add_triangle([vidx1, vidx4, vidx3], 
                                      TriangleType::KiteTriangle, flip,
                                      tidx, gnew);

                    let c = self.add_triangle([vidx1, vidx2, vidx3], 
                                              TriangleType::KiteTriangle, same,
                                              tidx, gnew);

                    self.split_triangle(Some(tidx), vidx1, vidx3, b, c);
                    self.split_triangle(Some(tidx), vidx3, vidx4, a, b);

                },

                TriangleType::DartTriangle => {

                    let vidx3 = self.split_edge(vidx0, vidx1);

                    let a = self.add_triangle([vidx0, vidx3, vidx2], 
                                              TriangleType::KiteTriangle, same,
                                              tidx, gnew);
                    
                    let b = self.add_triangle([vidx1, vidx2, vidx3], 
                                              TriangleType::DartTriangle, same,
                                              tidx, gnew);

                    self.split_triangle(Some(tidx), vidx2, vidx3, a, b);
                    
                }

            };

        }

        self.tris.append(&mut new_tris);
        
        debug_assert!(new_tris.is_empty());
        debug_assert!(self.tris.last().unwrap().generation == gnew);
        
    }

    fn assembly(ttype: TriangleType,
                tside: TriangleSide,
                level: usize,
                rotate: bool) -> PTriangulation {

        let mut pt = match ttype {
            TriangleType::KiteTriangle => PTriangulation::half_kite(),
            TriangleType::DartTriangle => PTriangulation::half_dart()
        };
        
        for v in &mut pt.verts {
            if tside != pt.tris[0].tside {
                v.y = -v.y;
            }
            if rotate {
                *v = Point2d::new(v.y, -v.x);
            }
        }

        pt.tris[0].tside = tside;

        for _ in 0..level {
            pt.subdivide()
        }

        pt


    }


}


//////////////////////////////////////////////////////////////////////


type TriFunc = fn() -> PTriangulation;

const VALID_SOURCES: phf::Map<&'static str, TriFunc> = phf_map! {
    "half_kite" => PTriangulation::half_kite,
    "half_dart" => PTriangulation::half_dart,
    "kite" => PTriangulation::kite
};

static AXIS_LOOKUP: phf::Map<&'static str, Axis> = phf_map! {
    "x" => Axis::PosX,
    "-x" => Axis::NegX,
    "y" => Axis::PosY,
    "-y" => Axis::NegY
};

#[derive(Debug,PartialEq)]
enum CenterType {
    Unset,
    Fit,
    VertIndexArray(Vec<usize>),
    Point(Point2d)
}

#[derive(Debug,PartialEq,Eq,Hash,Clone,Copy)]
enum Axis {
    PosX,
    NegX,
    PosY, 
    NegY
}

#[derive(Debug,PartialEq)]
enum AngleType {
    Unset, 
    Radians(f64),
    VDiff(Axis, usize, usize)
}

#[derive(Debug,PartialEq,Eq,Hash,Clone,Copy)] 
#[allow(dead_code)]
enum DimType {
    Raw,
    Finished,
    Border,
    Binding,
}

#[derive(Debug,PartialEq,Clone,Copy)]
enum CoordinateFrame {
    OrigCoords,
    XformedCoords
}

#[derive(Debug,PartialEq)]
enum RectType {
    Unset, 
    Fit,
    Dims { dtype: DimType, width: f64, height: f64 },
    Verts { dtype: DimType, indices: Vec<usize> }
}
 
#[derive(Debug)]
struct QuiltSpec {
    source: Option<(String, TriFunc)>,
    depth: usize,
    center: CenterType,
    angle: AngleType,
    scale: f64,
    rect_dims: RectType,
    line_inset: f64,
    border_allowance: f64,
    nudge: Option<Vec2d>,
    border_width: f64,
    binding_width: f64,
}

//////////////////////////////////////////////////////////////////////

macro_rules! parse_tokens {

    ($it:ident { } -> { $($tuple:ident,)* } ) => (
        match $it.next() {
            Some(value) => Err(format!("found extra token(s) starting with \"{:}\"", value)),
            None => Ok(($($tuple),*))
        }
    );

    // name, String pair
    ($it:ident { $name:ident : String, $($ts:tt)* } -> { $($tuple:ident,)* } -> { $($code:tt)* } ) => (
        match $it.next() {
            None => Err(format!("missing token for {:}", stringify!($name)) ),
            Some(value) => {
                let $name = value.to_string();
                parse_tokens!( $it { $($ts)* } -> { $($tuple,)* $name, } )
            }
        }
    );

    // string literal
    ($it:ident { $str:literal, $($ts:tt)* } -> { $($tuple:ident,)* } ) => (
        match $it.next() {
            None => Err(format!("missing token for {:}", stringify!($name)) ),
            Some(value) => if value == &$str {
                parse_tokens!( $it { $($ts)* } -> { $($tuple,)* } )
            } else {
                Err(format!("unmatched string literal: expected \"{:}\" but got \"{:}\"", $str, value))
            }
        }
    );

    // string option
    ($it:ident { $name:ident in $arr:expr, $($ts:tt)* } -> { $($tuple:ident,)* } ) => (
        match $it.next() {
            None => Err(format!("missing token for {:}", stringify!($name)) ),
            Some(&value) => {
                if $arr.contains(value) {
                    let $name = value.to_string();
                    parse_tokens!( $it { $($ts)* } -> { $($tuple,)* $name, } )
                } else {
                    Err(format!("unexpected value \"{:}\" for {:}, expected one of: {:?}",
                                 value, stringify!($name), $arr))
                }
            }
        }
    );

    // string map
    ($it:ident { $name:ident from $map:expr, $($ts:tt)* } -> { $($tuple:ident,)* } ) => (
        match $it.next() {
            None => Err(format!("missing token for {:}", stringify!($name)) ),
            Some(&value) => {
                if let Some(&k) = $map.get(value) {
                    let $name = (value.to_string(), k);
                    parse_tokens!( $it { $($ts)* } -> { $($tuple,)* $name, } )
                } else {
                    let mut extended: Vec<&str> = Vec::new();
                    extended.extend($map.keys());
                    Err(format!("unexpected value \"{:}\" for {:}, expected one of: {{\"{:}\"}}",
                                value, stringify!($name), extended.join("\", \"") ))
                }
            }
        }
    );

    // name, type pair
    ($it:ident { $name:ident : $type:ident, $($ts:tt)* } -> { $($tuple:ident,)* } ) => (
        match $it.next() {
            None => Err(format!("missing token for {:}", stringify!($name)) ),
            Some(value) => if let Ok($name) = value.parse::<$type>() {
                parse_tokens!( $it { $($ts)* } -> { $($tuple,)* $name, } )
            } else {
                Err(format!("error parsing {:} as type {:}", 
                                       value, stringify!($type)))
            }
        }
    );

    // Entry point
    ($a:expr, { $($ts:tt)+ }) => (
        {
            let mut it = $a.iter();
            parse_tokens!( it { $($ts)* , } -> { } )
        }
    );

    // Entry point
    ($a:expr, { }) => (
        {
            let mut it = $a.iter();
            parse_tokens!( it { } -> { } )
        }
    )

}
macro_rules! copy_field {

    ($dst:ident, $src:ident, $field:ident, $null:expr) => (
        if $src.$field != $null {
            if $dst.$field != $null {
                bail!("{:} is already set", stringify!($field));
            }
            $dst.$field = $src.$field;
        }
    )

}

macro_rules! ensure_field {

    ($qs:ident, $fvec:ident, $field:ident, $null:expr) => (
        if $qs.$field == $null {
            $fvec.push(stringify!($field));
        }
    )
        
}

//////////////////////////////////////////////////////////////////////

fn parse_indices(tokens: &[&str]) -> Result<Vec<usize>> {

    let mut rval = Vec::new();

    for value in tokens {
        if let Ok(idx) = value.parse::<usize>() {
            rval.push(idx);
        } else {
            bail!("invalid index: {:}", value);
        }
    }

    Ok(rval)
}

//////////////////////////////////////////////////////////////////////

impl QuiltSpec {


    fn new() -> QuiltSpec {
        QuiltSpec {
            source: None,
            depth: usize::MAX,
            center: CenterType::Unset,
            angle: AngleType::Unset,
            scale: -1.0,
            rect_dims: RectType::Unset,
            line_inset: -1.0,
            border_allowance: -1.0,
            nudge: None,
            border_width: 0.0,
            binding_width: 0.0,
        }
    }


    fn update(&mut self, other: QuiltSpec) -> Result<()> {
        
        copy_field!(self, other, source, None);
        copy_field!(self, other, depth, usize::MAX);
        copy_field!(self, other, center, CenterType::Unset);
        copy_field!(self, other, angle, AngleType::Unset);
        copy_field!(self, other, scale, -1.0);
        copy_field!(self, other, rect_dims, RectType::Unset);
        copy_field!(self, other, line_inset, -1.0);
        copy_field!(self, other, border_allowance, -1.0);
        copy_field!(self, other, nudge, None);
        copy_field!(self, other, border_width, 0.0);
        copy_field!(self, other, binding_width, 0.0);

        Ok(())

    }

    fn parse_keyword(keyword: &str,
                     rest: &[&str]) -> Result<QuiltSpec> {

        let mut update = QuiltSpec::new();

        match keyword {
            
            "source" =>  { 

                let (source, depth) = parse_tokens!(rest, { 
                    source from VALID_SOURCES,
                    "depth", 
                    depth : usize 
                })?;

                update.source = Some(source);
                update.depth = depth;

            },

            "center" => {

                let (x, y) = parse_tokens!(rest, {x: f64, y:f64})?;

                update.center = CenterType::Point(Point2d::new(x, y));

            }

            "center_verts" => { 

                let verts = parse_indices(rest)?;

                update.center = CenterType::VertIndexArray(verts);

            },

            "angle_rad" | "angle_deg" => {
                
                let mut angle = parse_tokens!(rest, { angle: f64 })?;

                if keyword.ends_with("deg") {
                    angle *= std::f64::consts::PI / 180.0;
                }

                update.angle = AngleType::Radians(angle);

            },

            "angle_vdiff" => {

                let (axis, vidx0, vidx1) = parse_tokens!(rest, { 
                    axis from AXIS_LOOKUP,
                    vidx0: usize,
                    vidx1: usize
                })?;

                let (_, axis) = axis;
                update.angle = AngleType::VDiff(axis, vidx0, vidx1);
                
            }

            "scale_long_side" | "scale_short_side" => { 

                let mut scale = parse_tokens!(rest, { scale: f64 })?;

                if keyword.ends_with("long_side") {
                    scale /= PHI;
                }

                update.scale = scale;

            },
            
            "finished_rect_dims" | "raw_rect_dims" => { 
                
                let (w, h) = parse_tokens!(rest, { width: f64, height: f64 })?;

                let dtype = if keyword.starts_with("raw") {
                    DimType::Raw
                } else {
                    DimType::Finished
                };

                update.rect_dims = RectType::Dims{dtype: dtype,
                                                  width: w,
                                                  height: h};

            },

            "finished_rect_verts" | "raw_rect_verts" => {

                let verts = parse_indices(rest)?;

                let dtype = if keyword.starts_with("raw") {
                    DimType::Raw
                } else {
                    DimType::Finished
                };

                update.rect_dims = RectType::Verts{dtype: dtype,
                                                   indices: verts};

            }

            "border_allowance" => { 

                let length = parse_tokens!(rest, { length: f64 })?;

                update.border_allowance = length;

            },

            "line_inset" => { 

                let length = parse_tokens!(rest, { length: f64 })?;

                update.line_inset = length;

            },

            "fit_all" => {

                parse_tokens!(rest, { })?;

                update.center = CenterType::Fit;
                update.rect_dims = RectType::Fit;

            },

            "nudge" => {

                let (x, y) = parse_tokens!(rest, { x: f64, y: f64 })?;
                
                update.nudge = Some(Vec2d::new(x, y));

            },

            "border_width" => {
                
                let length = parse_tokens!(rest, { length: f64 })?;

                update.border_width = length;

            },

            "binding_width" => {
                
                let length = parse_tokens!(rest, { length: f64 })?;

                update.binding_width = length;

            },

            _ => {
                bail!("unrecognized keyword");
            }

        };

        Ok(update)
       

    }
    
    fn update_from(&mut self, line: &str) -> Result<()> {

        let mut trimmed = line.trim();
        
        if let Some(pos) = trimmed.find('#') {
            trimmed = &trimmed[0..pos];
        }

        if trimmed.len() == 0 { 
            return Ok(());
        }

        let tokens: Vec<&str> = trimmed.split(' ').collect();

        let keyword = tokens[0];
        let rest = &tokens[1..];

        let update = Self::parse_keyword(keyword, rest).chain_err(
            || format!("while parsing keyword {:}", keyword))?;

        self.update(update)

    }

    fn parse(filename: &str, istr: &mut impl BufRead) -> Result<QuiltSpec> {

        let mut qs = QuiltSpec::new();
        let mut lineno = 0;
        
        loop {

            let mut line = String::new();

            lineno += 1;

            let len = istr.read_line(&mut line).chain_err(|| format!("{:}:{:}: read error", filename, lineno))?;

            if len == 0 {
                break;
            }

            qs.update_from(line.as_str()).chain_err(|| format!("{:}:{:}: parse error", filename, lineno))?;

                    
        }

        let mut unset_fields = Vec::new();
        
        ensure_field!(qs, unset_fields, source, None);
        ensure_field!(qs, unset_fields, depth, usize::MAX);
        ensure_field!(qs, unset_fields, center, CenterType::Unset);
        ensure_field!(qs, unset_fields, angle, AngleType::Unset);
        ensure_field!(qs, unset_fields, scale, -1.0);
        ensure_field!(qs, unset_fields, rect_dims, RectType::Unset);
        ensure_field!(qs, unset_fields, line_inset, -1.0);
        ensure_field!(qs, unset_fields, border_allowance, -1.0);

        if !unset_fields.is_empty() {
            bail!("{:}: the following field(s) were unset: {:}", 
                  filename, unset_fields.as_slice().join(", "));
        }

        Ok(qs)

    }

}

//////////////////////////////////////////////////////////////////////

#[derive(Debug)]
struct Seam {

    child_parts: [SeamPart; 2],
    p0: Point2d,
    p1: Point2d

}

#[allow(dead_code)]
struct Quilt {
    
    qs: QuiltSpec,
    pt: PTriangulation,

    raw_box: Box2d, 
    xform_fwd: Similarity2d,
    xform_inv: Similarity2d,
    
    xformed_verts: Vec<Point2d>,

    child_leafs: Vec<Vec<usize>>,

    tri_overlap_mask: Vec<bool>,

    visible_leaf_tris: Vec<usize>,
    visible_top_level_tris: Vec<usize>,

    seams: Vec<Seam>

}


fn gather_seams_r(pt: &PTriangulation,
                  child_leafs: &Vec<Vec<usize>>, 
                  vvec: &Vec<Point2d>, 
                  tri_overlap_mask: &Vec<bool>,
                  parent: Option<usize>,
                  slookup: &mut HashMap<usize, SeamPart>,
                  seams: &mut Vec<Seam>) -> SeamPart {
    
    debug_assert!(match parent {
        None => true,
        Some(tidx_parent) => !tri_overlap_mask[tidx_parent]
    });

    let mut output = SeamPart::Empty;
    let mut child_output = SeamPart::Empty;

    if let Some(v) = pt.tri_subdiv_lookup.get(&parent) {

        for &(vidx0, vidx1, tidx0, tidx1) in v {
            
            let mut child_parts: Vec<SeamPart> = vec![];

            let v0 = &vvec[vidx0];
            let v1 = &vvec[vidx1];
            let d10 = v1-v0;
            let dd = d10.dot(&d10);

            let mut u0: f64 = 0.0;
            let mut u1: f64 = 1.0;

            for &tidx in [tidx0, tidx1].iter() {

                let child_part = if tri_overlap_mask[tidx] {
                    let t = &pt.tris[tidx];
                    let level = pt.generations.len() - t.generation - 1;
                    SeamPart::BaseTri((t.ttype, t.tside, level))
                } else if let Some(&val) = slookup.get(&tidx) {
                    val
                } else {
                    let x = gather_seams_r(pt, child_leafs, vvec, tri_overlap_mask,
                                           Some(tidx), slookup, seams);
                    slookup.entry(tidx).or_insert(x);
                    x
                };

                if child_part != SeamPart::Empty {

                    child_parts.push(child_part);
                    child_output = child_output.max(child_part);

                    let mut cvidx = HashSet::new();

                    for &child_tidx in &child_leafs[tidx] {

                        if !tri_overlap_mask[child_tidx] { 
                            continue;
                        }

                        for &vidx in &pt.tris[child_tidx].indices {

                            cvidx.insert(vidx);
                            
                        }

                    }

                    let mut me_u0: f64 = 1.0;
                    let mut me_u1: f64 = 0.0;

                    for vidx in cvidx {

                        if pt.vertex_along_edge(vidx, vidx0, vidx1) {

                            let v = &vvec[vidx];

                            let u = (v - v0).dot(&d10) / dd;

                            debug_assert!(u >= -0.00001 && u <= 1.00001);

                            me_u0 = me_u0.min(u);
                            me_u1 = me_u1.max(u);
                            

                        }
                        
                    }
                    
                    let (me_u0, me_u1) = if me_u0 <= me_u1 {
                        (me_u0, me_u1)
                    } else {
                        (me_u1, me_u0)
                    };
                    
                    u0 = u0.max(me_u0);
                    u1 = u1.min(me_u1);


                }

            }

            if child_parts.len() == 2 {

                debug_assert!(u1 > u0);

                let cp0 = child_parts[0];
                let cp1 = child_parts[1];

                let (cp0, cp1) = if cp0 <= cp1 {
                    (cp0, cp1)
                } else {
                    (cp1, cp0)
                };

                output = SeamPart::SeamIndex(seams.len());

                seams.push( Seam { 
                    child_parts: [cp0, cp1],
                    p0: v0 + u0 * d10,
                    p1: v0 + u1 * d10,
                });

            }

        }

    }

    match output {
        SeamPart::Empty => child_output,
        _ => output
    }

}


fn gather_seams(pt: &PTriangulation,
                child_leafs: &Vec<Vec<usize>>,
                vvec: &Vec<Point2d>, 
                tri_overlap_mask: &Vec<bool>) -> Vec<Seam> {

    let parent = if pt.tri_subdiv_lookup.contains_key(&None) {
        None
    } else {
        if tri_overlap_mask[0] {
            return vec![];
        }
        Some(0)
    };

    let mut seams = vec![];
    let mut slookup = HashMap::new();

    gather_seams_r(pt, child_leafs, vvec, tri_overlap_mask, 
                   parent, &mut slookup, &mut seams);

    seams

}


impl Quilt {

    fn scale_factor(&self, frame: CoordinateFrame) -> f64 {

        match frame {
            CoordinateFrame::OrigCoords => {
                self.xform_fwd.scaling()
            },
            CoordinateFrame::XformedCoords => {
                1.0
            }
        }

    }

    fn new(qs: QuiltSpec) -> Result<Quilt> {

        let (_, sfunc) = match &qs.source {
            None => bail!("no source!"),
            Some(source) => source
        };

        let mut pt = sfunc();

        println!("got {:} root tris, {:} vertices", 
                 pt.tris.len(), pt.verts.len());
        
        for _ in 0..qs.depth {
            pt.subdivide();

            println!("got {:} tris ({:} leaf tris), {:} vertices", 
                     pt.tris.len(), 
                     pt.tris.len() - pt.generations.last().unwrap(),
                     pt.verts.len());
        }
        
        println!();

        let angle = match qs.angle {

            AngleType::Radians(r) => { r },

            AngleType::VDiff(axis, idx0, idx1) => { 
                
                let uv = pt.verts[idx1] - pt.verts[idx0];

                match axis {
                    Axis::PosX => uv[1].atan2(uv[0]),
                    Axis::NegX => (-uv[1]).atan2(-uv[0]),
                    Axis::PosY => uv[0].atan2(-uv[1]),
                    Axis::NegY => (-uv[0]).atan2(uv[1])
                }
                
            },
            
            AngleType::Unset => { 
                bail!("angle wasn't set!");
            }

        };
        
        let rotation = Rotation2d::new(-angle);

        let center = match &qs.center {

            CenterType::Fit => { 

                
                let rect = pt.verts.iter().fold(
                    Rect2d::empty(), 
                    |mut rect, &p| { 
                        rect.expand(&(rotation * p));
                        rect
                    });

                rotation.inverse() * rect.center()

            },

            CenterType::VertIndexArray(verts) => { 

                let rect = verts.iter().fold(
                    Rect2d::empty(),
                    |mut rect, &vidx| {
                        rect.expand(&(rotation * &pt.verts[vidx]));
                        rect
                    });

                rotation.inverse() * rect.center()
                    
            },

            CenterType::Point(p) => { *p },

            CenterType::Unset => {
                bail!("center wasn't set!");
            }

        };

        let grow = PHI.powf(qs.depth as f64);
        let z = qs.scale * grow;
        let scl = 1.0 / z;

        let mut xform_fwd = Similarity2d::new(center-Point2d::origin(), angle, scl);

        let mut xform_inv = xform_fwd.inverse();

        if let Some(nudge) = qs.nudge {
            
            let xform_nudge = Translation2d::new(nudge.x, nudge.y);

            xform_inv = xform_nudge * xform_inv;
            xform_fwd = xform_inv.inverse();
            
        }

        let xformed_verts: Vec<Point2d> = pt.verts.iter().map(|x| xform_inv * x).collect();

        let eps = Vec2d::repeat(1e-5*qs.scale);

        let raw_box = Box2d::new( match &qs.rect_dims {

            RectType::Fit => {

                let mut r = Vec2d::new(0.0, 0.0);

                for p in &xformed_verts {
                    r = r.sup(&Vec2d::new(p[0].abs(), p[1].abs()));
                }

                r + Vec2d::repeat(qs.border_allowance)

            },

            RectType::Dims{dtype, width, height}  => {

                let r = 0.5*Vec2d::new(*width, *height);

                match dtype {
                    DimType::Raw => r,
                    DimType::Finished => r + Vec2d::repeat(qs.border_allowance),
                    _ => bail!("only raw and finished bindings supported here")
                        
                }
                
            }

            RectType::Verts{dtype, indices} => {

                let mut r = Vec2d::new(0.0, 0.0);

                for i in indices {
                    let p = xformed_verts[*i];
                    r = r.sup(&Vec2d::new(p[0].abs(), p[1].abs()));
                }

                r = (r - eps).sup(&Vec2d::zeros());

                match dtype {
                    DimType::Raw => r,
                    DimType::Finished => r + Vec2d::repeat(qs.border_allowance),
                    _ => bail!("only raw and finished bindings supported here")
                }

            }

            _ => { bail!("not done yet"); }

        });

        let child_leafs = pt.get_child_leafs();

        let mut tri_overlap_mask = vec![false; pt.tris.len()];
        let mut visible_leaf_tris = Vec::new();

        for (tidx, tri) in pt.final_gen_tris_enumerated() {

            let (v0, v1, v2) = tri.get_verts(&xformed_verts);

            if raw_box.overlaps_tri(v0, v1, v2) {
                tri_overlap_mask[tidx] = true;
                visible_leaf_tris.push(tidx);
            }

        }

        let (tidx_first_gfinal, _) = pt.final_gen_tris();

        for tidx in 0..tidx_first_gfinal {

            if child_leafs[tidx].iter().all(|&i| tri_overlap_mask[i]) {
                tri_overlap_mask[tidx] = true;
            }

        }

        let mut visible_top_level_tris = Vec::new();

        for (i, t) in pt.tris.iter().enumerate() {
            
            let me_overlaps = tri_overlap_mask[i];

            let parent_overlaps = match t.parent {

                None => false,

                Some(parent_tidx) => tri_overlap_mask[parent_tidx]

            };

            if me_overlaps && !parent_overlaps {
                visible_top_level_tris.push(i)
            }
            
        }

        let seams = gather_seams(&pt, &child_leafs,
                                 &xformed_verts, &tri_overlap_mask);


        Ok(Quilt {

            qs: qs,
            pt: pt,

            raw_box: raw_box,
            xform_fwd: xform_fwd,
            xform_inv: xform_inv,
            xformed_verts: xformed_verts,

            child_leafs: child_leafs,

            tri_overlap_mask: tri_overlap_mask,

            visible_leaf_tris: visible_leaf_tris,
            visible_top_level_tris: visible_top_level_tris,

            seams: seams
            
        })

    }

    fn box_verts(&self, dtype: DimType, frame: CoordinateFrame) -> Vec<Point2d> {

        let h = self.raw_box.half_dims + match dtype {
            DimType::Raw => Vec2d::zeros(),
            DimType::Finished => Vec2d::repeat(-self.qs.border_allowance),
            DimType::Border => Vec2d::repeat(self.qs.border_width-self.qs.binding_width-self.qs.border_allowance),
            DimType::Binding => Vec2d::repeat(self.qs.border_width-self.qs.border_allowance)
        };

        let box_verts = vec![
            Point2d::new(h.x, h.y),
            Point2d::new(-h.x, h.y),
            Point2d::new(-h.x, -h.y),
            Point2d::new(h.x, -h.y),
        ];

        match frame {

            CoordinateFrame::XformedCoords => box_verts,
            CoordinateFrame::OrigCoords => box_verts.iter().map(|&p| self.xform_fwd * p).collect()

        }

    }

    


}

//////////////////////////////////////////////////////////////////////

#[allow(dead_code)]
enum HAlign {
    Left,
    Center,
    Right
}

#[allow(dead_code)]
enum VAlign {
    Top,
    Center,
    Bottom
}

    

trait CairoVecOps {

    fn moveto(&self, p: &Point2d);
    fn lineto(&self, p: &Point2d);
    fn setcolor(&self, v: &Vec3d);
    fn drawtri(&self, p0: &Point2d, p1: &Point2d, p2: &Point2d);
    fn drawpoly(&self, poly: &Vec<Point2d>);
    fn aligntext(&self, p: &Point2d, text: &str, ha: HAlign, va: VAlign);
    fn showtext(&self, p: &Point2d, text: &str);
    fn translatep(&self, p: &Point2d);
    fn translatev(&self, p: &Vec2d);

    fn aligntext_extents(&self, p: &Point2d,
                        text: &str,
                        extents: &cairo::TextExtents,
                        ha: HAlign, va: VAlign);

}

impl CairoVecOps for cairo::Context {
    
    fn translatep(&self, p: &Point2d) {
        self.translate(p.x, p.y);
    }

    fn translatev(&self, v: &Vec2d) {
        self.translate(v.x, v.y);
    }

    fn setcolor(&self, v: &Vec3d) {
        self.set_source_rgb(v[0], v[1], v[2]);
    }

    fn moveto(&self, p: &Point2d) {
        self.move_to(p.x, p.y);
    }

    fn lineto(&self, p: &Point2d) {
        self.line_to(p.x, p.y);
    }
    
    fn drawtri(&self, p0: &Point2d, p1: &Point2d, p2: &Point2d) {
        self.move_to(p0[0], p0[1]);
        self.line_to(p1[0], p1[1]);
        self.line_to(p2[0], p2[1]);
        self.close_path();
    }

    fn drawpoly(&self, poly: &Vec<Point2d>) {
        for (i, p) in poly.iter().enumerate() {
            if i == 0 {
                self.move_to(p[0], p[1]);
            } else {
                self.line_to(p[0], p[1]);
            }
        }
        self.close_path();
    }

    fn aligntext(&self, p: &Point2d, text: &str, ha: HAlign, va: VAlign) {

        let extents = self.text_extents(text);

        self.aligntext_extents(p, text, &extents, ha, va);

    }

    fn aligntext_extents(&self, p: &Point2d, text: &str, extents: &cairo::TextExtents, ha: HAlign, va: VAlign) {

        let xoffs = match ha {
            HAlign::Left => 0.0,
            HAlign::Center => 0.5,
            HAlign::Right => 1.0
        } * (extents.width + 2.0 * extents.x_bearing);

        let yoffs = match va {
            VAlign::Top => 1.0,
            VAlign::Center => 0.5,
            VAlign::Bottom => 0.0
        } * (extents.height + 2.0 * extents.y_bearing);

        self.moveto(&(p + Vec2d::new(-xoffs, -yoffs)));
        self.show_text(text);

    }

    fn showtext(&self, p: &Point2d, text: &str) {

        self.moveto(&p);
        self.show_text(text);

    }


}

macro_rules! with_save_restore {

    ($ctx:ident, { $($tree:tt)* }) => { 

        $ctx.save();
        
        {
            
            $($tree)*
                
        }

        $ctx.restore();

    }

}

//////////////////////////////////////////////////////////////////////

#[derive(Debug)]
struct PageSettings {

    frame: CoordinateFrame,
    draw_as_finished: bool,
    two_color_palette: bool,
    label_tris: bool,
    label_points: bool,
    show_seams: bool,
    mirror_x: bool

}

#[derive(Debug)]
struct DrawLengths {
    
    short_side_length: f64,
    line_inset: f64,
    line_width: f64

}

fn get_draw_lengths(quilt: &Quilt,
                    frame: CoordinateFrame,
                    scl: f64) -> DrawLengths {

    let scale_factor = scl * quilt.scale_factor(frame);

    let short_side_length = scale_factor * quilt.qs.scale;
    let line_inset = scale_factor * quilt.qs.line_inset;
    let line_width = (0.05 * short_side_length).max(0.25).min(1.0);

    DrawLengths {
        short_side_length: short_side_length,
        line_inset: line_inset,
        line_width: line_width
    }

}

fn label_tri(ctx: &cairo::Context,
             gmax: usize,
             t: &PTriangle,
             xverts: &Vec<Point2d>) -> Result<()> {

    let (v0, v1, v2) = t.get_verts(&xverts);

    let vdiff = match t.tside {
        TriangleSide::SideLeft => v2 - v0, 
        TriangleSide::SideRight => v0 - v2
    };

    let (center, r) = tri_center(v0, v1, v2)?;

    let level = gmax - t.generation;
    let text = get_tri_string(t.ttype, t.tside, level);
    
    with_save_restore!(ctx, {
                
        ctx.translate(center.x, center.y);
        ctx.rotate(vdiff.y.atan2(vdiff.x));
        ctx.set_font_size(1.3 * r);
        ctx.aligntext(&Point2d::origin(), text.as_str(), 
                      HAlign::Center, VAlign::Center);
        ctx.fill();

    });

    Ok(())

}
                    

fn draw_triangles(ctx: &cairo::Context,
                  pt: &PTriangulation,
                  child_leafs: &Vec<Vec<usize>>,
                  xverts: &Vec<Point2d>,
                  draw_lengths: &DrawLengths,
                  tri_indices: &Vec<usize>,
                  two_color_palette: bool,
                  label_tris: bool,
                  label_points: bool) -> Result<()> {

    let lw = draw_lengths.line_width;

    ctx.setcolor(&Vec3d::repeat(0.6));
    
    with_save_restore!(ctx, { 
        
        ctx.set_line_join(cairo::LineJoin::Round);
        ctx.set_line_cap(cairo::LineCap::Round);

        ctx.set_line_width(lw);

        for &i in tri_indices.iter() {

            let t = &pt.tris[i];
            
            let (v0, v1, v2) = t.get_verts(&xverts);
            
            ctx.drawtri(v0, v1, v2);

        }

        ctx.fill_preserve();
        ctx.stroke();

    });


    for &i in tri_indices.iter() {

        let t = &pt.tris[i];

        let mut cidx = get_cidx(t.ttype, t.tside);

        if two_color_palette {
            cidx -= cidx % 2;
        }

        for &c in &child_leafs[i] {

            let t = &pt.tris[c];
            
            let c = COLORS[cidx];
            let cv = Vec3d::new(c[0], c[1], c[2]);
            let cv = cv * 0.125 + Vec3d::repeat(0.875);

            let (v0, v1, v2) = t.get_verts(&xverts);

            let (b0, b1, b2, _) = tri_border(v0, v1, v2, draw_lengths.line_inset, 0.25*lw)?;

            ctx.setcolor(&(0.375*cv + 0.625*Vec3d::repeat(0.5)));
            ctx.drawtri(v0, v1, v2);
            ctx.fill();

            ctx.setcolor(&cv);
            ctx.drawtri(&b0, &b1, &b2);
            ctx.fill();

        }

    }
    

    if label_tris {

        ctx.set_source_rgba(0.75, 0.0, 0.0, 0.625);

        for &i in tri_indices.iter() {

            label_tri(ctx, pt.generations.len()-1, &pt.tris[i], &xverts)?;

        }
        
    }


    if label_points { 

        with_save_restore!(ctx, {

            ctx.set_source_rgb(0.0, 0.0, 0.0);

            let mut vverts: HashSet<usize> = HashSet::new();

            for &i in tri_indices.iter() {
                let t = &pt.tris[i];
                for &v in &t.indices {
                    vverts.insert(v);
                }

            }

            ctx.set_font_size((0.25 * draw_lengths.short_side_length).min(12.0));

            for &v in vverts.iter() {

                let p = &xverts[v];

                let text = format!("{:}", v);

                ctx.aligntext(p, text.as_str(), HAlign::Center, VAlign::Center); 
                
                
            }
            
        });

    }



    Ok(())

}

fn draw_quilt(ctx: &cairo::Context,
              page_rect: &Rect2d,
              quilt: &Quilt,
              tri_indices: &Vec<usize>,
              pset: PageSettings) -> Result<()> {

    let vvec = match pset.frame {
        CoordinateFrame::OrigCoords => &quilt.pt.verts,
        CoordinateFrame::XformedCoords => &quilt.xformed_verts
    };
    
    let mut verts_rect = Rect2d::empty();

    let mut boxes = HashMap::new();

    let dtypes = [DimType::Raw, DimType::Finished, DimType::Binding, DimType::Border];

    for &dtype in &dtypes {

        let box_verts = quilt.box_verts(dtype, pset.frame);
        boxes.insert(dtype, box_verts);

    }

    if pset.draw_as_finished {

        for p in boxes.get(&DimType::Binding).unwrap() {
            verts_rect.expand(p);
        }

    } else {

        for tri in tri_indices.iter().map(|&i| &quilt.pt.tris[i]) {
            for &i in &tri.indices {
                verts_rect.expand(&vvec[i]);
            }
        }

        for p in boxes.get(&DimType::Raw).unwrap() {
            verts_rect.expand(p);
        }


    }

    let hsign = if pset.mirror_x { -1.0 } else { 1.0 };
    
    let (transform, scl) = get_page_transform(&verts_rect, &page_rect, hsign);

    let draw_lengths = get_draw_lengths(&quilt, pset.frame, scl);

    for dtype in &dtypes {

        let box_verts = boxes.get_mut(dtype).unwrap();

        for p in box_verts { *p = transform * *p; }

    }


    with_save_restore!(ctx, { 

        ctx.setcolor(&Vec3d::repeat(0.6));

        if pset.draw_as_finished {

            ctx.set_line_join(cairo::LineJoin::Miter);

            if quilt.qs.binding_width > 0.0 { 

                with_save_restore!(ctx, {
                    
                    let c = COLORS[0];
                    let cv = Vec3d::new(c[0], c[1], c[2]);
                    let cv = cv * 0.125 + Vec3d::repeat(0.875);
                    
                    ctx.setcolor(&cv);
                    ctx.drawpoly(&boxes.get(&DimType::Binding).unwrap());
                    ctx.set_line_width(2.0);
                    ctx.fill_preserve();
                    ctx.stroke();
                    
                });

            }

            ctx.drawpoly(&boxes.get(&DimType::Border).unwrap());
            ctx.set_line_width(2.0);
            ctx.fill_preserve();
            ctx.stroke();

            ctx.drawpoly(&boxes.get(&DimType::Finished).unwrap());
            ctx.clip();

        }
        
        let xverts: Vec<Point2d> = vvec.iter().map(|x| transform*x).collect();


        draw_triangles(ctx, &quilt.pt,
                       &quilt.child_leafs,
                       &xverts, 
                       &draw_lengths,
                       tri_indices, 
                       pset.two_color_palette,
                       pset.label_tris,
                       pset.label_points)?;

        if !pset.draw_as_finished { 

            with_save_restore!(ctx, {

                let lw = draw_lengths.line_width;

                ctx.set_line_join(cairo::LineJoin::Miter);
                ctx.set_line_cap(cairo::LineCap::Butt);

                ctx.set_line_width(lw);
                ctx.set_source_rgb(0.75, 0., 0.);

                let verts = &boxes.get(&DimType::Finished).unwrap();

                ctx.drawpoly(&verts);
                ctx.stroke();

                ctx.set_dash(&[4.0*lw, 4.0*lw], 0.0);
                ctx.drawpoly(&boxes.get(&DimType::Raw).unwrap());
                
                ctx.stroke();

            });

        }

        if pset.show_seams {

            ctx.set_source_rgb(0.0, 0.0, 0.0);
            ctx.set_line_cap(cairo::LineCap::Round);

            let xform = match pset.frame {
                CoordinateFrame::OrigCoords => transform * quilt.xform_fwd,
                CoordinateFrame::XformedCoords => transform
            };

            for (idx, seam) in quilt.seams.iter().enumerate() {

                
                let p0 = xform * seam.p0;
                let p1 = xform * seam.p1;

                let d10 = p1 - p0;

                let len = d10.norm();

                let ctr = p0 + 0.5*d10;

                let dir = d10 / len;

                let s = format!("{:}", idx+1);
                let text = s.as_str();

                let extents = ctx.text_extents(text);

                let r = Vec2d::new(extents.width, extents.height).norm();

                //ctx.arc(ctr.x, ctr.y, r + 2.0, 0.0, 2.0*std::f64::consts::PI);
                //ctx.fill_preserve();

                let p0in = p0 + 8.0*dir;
                let p1in = p1 - 8.0*dir;

                ctx.set_source_rgb(0.0, 0.0, 0.0);
                ctx.moveto(&p0in);
                ctx.lineto(&p1in);
                ctx.set_line_width(5.0);
                ctx.stroke();

                ctx.set_line_width(1.0);
                ctx.arc(ctr.x, ctr.y, 0.5*r+3.0, 0.0, 2.0*std::f64::consts::PI);
                ctx.fill();
                
                
                ctx.set_source_rgb(1.0, 1.0, 1.0);
                ctx.moveto(&p0in);
                ctx.lineto(&p1in);
                ctx.set_line_width(3.0);
                ctx.stroke();

                ctx.set_line_width(1.0);
                ctx.arc(ctr.x, ctr.y, 0.5*r+2.0, 0.0, 2.0*std::f64::consts::PI);
                ctx.fill();


                ctx.set_source_rgb(0.0, 0.0, 0.0);

                ctx.aligntext_extents(&ctr, text, &extents,
                                      HAlign::Center, VAlign::Center);

                ctx.fill();

                

            }

        }
        


    } );

    Ok(())

}

//////////////////////////////////////////////////////////////////////


enum Drawable<'a> {
    LineBreak,
    Box {
        dims: Vec2d,
        drawfunc: Box<dyn Fn(&cairo::Context) -> Result<()> + 'a>
    }
}

fn vstack<'a>(d1: Drawable<'a>,
              d2: Drawable<'a>,
              spacing: f64) -> Result<Drawable<'a>> {
    
    if let Drawable::Box { dims: dims1, drawfunc: drawfunc1 } = d1 {
        if let Drawable::Box { dims: dims2, drawfunc: drawfunc2 } = d2 {
            
            let dims = Vec2d::new(dims1.x.max(dims2.x),
                                  dims1.y + spacing + dims2.y);

            let drawfunc = move |ctx: &cairo::Context| {
                
                (drawfunc1)(&ctx)?;

                with_save_restore!(ctx, { 
                    ctx.translate(0.0, dims1.y + spacing);
                    (drawfunc2)(&ctx)?;
                });

                Ok(())

            };

            return Ok(Drawable::Box { dims: dims, drawfunc: Box::new(drawfunc) });

        }
    }
    
    bail!("can't stack non-boxes");

}
    
fn label_drawable<'a>(ctx: &cairo::Context,
                      font_size: f64,
                      text: String,
                      force_width: f64,
                      bottom_margin: f64) -> Drawable<'a> {

    let width = if force_width > 0.0 {
        force_width
    } else {
        let extents = ctx.text_extents(text.as_str());
        extents.width
    };

    let f = move |ctx: &cairo::Context| {
        with_save_restore!(ctx, {
            ctx.set_font_size(font_size);
            ctx.set_source_rgb(0.0, 0.0, 0.0);
            ctx.move_to(0.0, font_size);
            ctx.show_text(text.as_str());
            ctx.fill();
        });
        Ok(())
    };

    Drawable::Box { dims: Vec2d::new(width, font_size + bottom_margin), 
                    drawfunc: Box::new(f) }

}

fn layout_page<'a>(ctx: &'a cairo::Context,
                   rect: &Rect2d,
                   drawables: &Vec<Drawable<'a>>,
                   spacing: f64) -> Result<()> {

    let rect_dims = rect.p1 - rect.p0;

    let mut cur_top_left = rect.p0;
    let mut start = 0;

    while start < drawables.len() {

        let mut total_width = 0.0;
        let mut max_height: f64 = 0.0;

        let mut end = start;

        let mut have_break = false;
        
        while end < drawables.len() { 

            let item = &drawables[end];

            match item {

                Drawable::LineBreak => {
                    have_break = true;
                    break;
                }

                Drawable::Box { dims, drawfunc: _ } => {
                    
                    let advance = dims.x + spacing;

                    let bust = total_width + advance >= rect_dims.x + spacing;
                    
                    if bust && end > start {
                        break;
                    }

                    total_width += advance;
                    max_height = max_height.max(dims.y);

                    end += 1;
                    
                }

            };
            
        }

        if cur_top_left.y + max_height >= rect_dims.y {
            ctx.show_page();
            cur_top_left = rect.p0;
        }

        for idx in start..end {

            let item = &drawables[idx];

            if let Drawable::Box { dims, drawfunc } = item { 
                
                with_save_restore!(ctx, {
                    
                    ctx.translate(cur_top_left.x, cur_top_left.y + max_height - dims.y);
                    
                    (drawfunc)(ctx)?;
                    
                        
                });

                cur_top_left.x += dims.x + spacing;
                
            }

        }

        cur_top_left.x = rect.p0.x;
        cur_top_left.y += max_height + spacing;

        if have_break {
            end += 1;
        }

        start = end;
        
    }

    ctx.show_page();

    Ok(())


}

//////////////////////////////////////////////////////////////////////

fn assembly_drawable<'a>(ctx: &cairo::Context,
                         ttype: TriangleType,
                         tside: TriangleSide,
                         level: usize, 
                         count: usize,
                         base_scale: f64,
                         upscale: f64,
                         inset_frac: f64,
                         max_width: f64,
                         spacing: f64) -> Result<Drawable<'a>> {

    let mut pt = PTriangulation::assembly(ttype, tside, level, true);

    let grow = upscale.powf(level as f64);
    let ideal_grow = PHI.powf(level as f64);

    let short_side_length = base_scale * grow;
    let line_inset = base_scale * inset_frac * grow / ideal_grow;
    
    let mut verts_rect = Rect2d::empty();

    for v in &mut pt.verts {
        *v = *v * short_side_length;
        verts_rect.expand(v);
    }

    let mut vdims = verts_rect.p1 - verts_rect.p0;

    if vdims.x > max_width {
        vdims *= max_width / vdims.x;
    }

    let page_rect = Rect2d { 
        p0: Point2d::origin(),
        p1: Point2d::origin() + vdims
    };

    let (transform, scl) = get_page_transform(&verts_rect, &page_rect, -1.0);

    let short_side_length = short_side_length * scl;
    let line_inset = line_inset * scl;

    for v in &mut pt.verts {
        *v = transform * *v;
    }

    let child_leafs = pt.get_child_leafs();

    let indices = if pt.generations.len() == 1 {
        vec![0]
    } else {
        let gstart = pt.generations[1];
        let gend = if pt.generations.len() == 2 {
            pt.tris.len()
        } else {
            pt.generations[2]
        };
        (gstart..gend).collect()
    };

    let drawfunc = move |ctx: &cairo::Context| {
 
        let draw_lengths = DrawLengths {
            short_side_length: short_side_length,
            line_inset: line_inset,
            line_width: 1.0
        };
       
        draw_triangles(ctx, &pt, &child_leafs, &pt.verts, 
                       &draw_lengths, &indices,
                       false, true, false)?;

        if let Some(splits) = pt.tri_subdiv_lookup.get(&Some(0)) {

            with_save_restore!(ctx, { 

                let (v0, v1, v2) = pt.tris[0].get_verts(&pt.verts);

                ctx.moveto(v0);
                ctx.lineto(v1);
                ctx.lineto(v2);
                ctx.close_path();
                ctx.clip();
                
                ctx.set_line_cap(cairo::LineCap::Round);
                ctx.set_source_rgba(0.75, 0.0, 0.0, 0.625);
                ctx.set_line_width(0.02*vdims.x);

                for &(vidx0, vidx1, _, _) in splits.iter() {

                    let v0 = &pt.verts[vidx0];
                    let v1 = &pt.verts[vidx1];

                    ctx.moveto(v0);
                    ctx.lineto(v1);

                    
                }

                ctx.stroke();

            } );

            
        }

        Ok(())
                       
    };

    let b = Drawable::Box { dims: vdims, drawfunc: Box::new(drawfunc) };

    let text = format!("{:}× {:}", count, get_tri_string(ttype, tside, level));

    vstack( b, label_drawable(ctx, 12.0, text, -1.0, 0.0), spacing )


}
              
//////////////////////////////////////////////////////////////////////

fn offset_and_flip(points: &Vec<Point2d>, dist: f64) -> Result<Vec<Point2d>> {

    assert!(points.len() > 2);

    let mut offset = offset_convex_poly(&points, dist, true)?;

    let p0 = &points[0];
    let p1 = &points[1];

    let d10 = p1 - p0;
    let n = Vec2d::new(d10.y, -d10.x) / d10.norm();


    let tfinal = Translation2d::new(p0[0], p0[1]);
    let tinit = tfinal.inverse();
    
    let m = Transform2d::from_matrix_unchecked(
        Matrix3d::new(
            1.0 - 2.0 * n.x * n.x, -2.0 * n.x * n.y, 0.0,
            -2.0 * n.x * n.y, 1.0 - 2.0 * n.y * n.y, 0.0, 
            0.0, 0.0, 1.0)
    );

    let transform = tfinal * m * tinit;

    for p in &mut offset {
        *p = transform * *p;
    }

    Ok(offset)

}

    

//////////////////////////////////////////////////////////////////////

struct StyledPolygon {
    color: Vec3d,
    line_width: f64,
    dash: f64,
    points: Vec<Point2d>
}

fn draw_templates(ctx: &cairo::Context,
                  scale: f64,
                  line_inset: f64) -> Result<()> {
    
    for &ttype in &[TriangleType::KiteTriangle, TriangleType::DartTriangle] {

        for &tside in &[TriangleSide::SideRight, TriangleSide::SideLeft] {

            let mut pt = PTriangulation::assembly(ttype, tside, 0, false);

            let ysign = match tside {
                TriangleSide::SideRight => -1.0,
                TriangleSide::SideLeft => 1.0
            };
 
            let rotate = Rotation2d::new(ysign * 36.0 * std::f64::consts::PI/180.0);

            for v in &mut pt.verts {
                *v = *v * scale * INCH;
                *v = rotate * *v;
            }

            let (v0, v1, v2) = pt.tris[0].get_verts(&pt.verts);

            let outer_tri = vec![*v0, *v1, *v2];


            let (b0, b1, b2, b3) = tri_border(v0, v1, v2, line_inset*INCH, 0.0)?;

            let inner = vec![b0, b1, b2];
            let border1 = vec![b0, b1, b3, *v0];
            let border2 = vec![b2, b3, *v1, *v2];

            let allowance = 0.5 * INCH;

            let border1_flip = offset_and_flip(&border1, allowance)?;
            let border2_flip = offset_and_flip(&border2, allowance)?;

            let outer_offset = offset_convex_poly(&outer_tri, 0.25*INCH, true)?;

            let mut drawstuff = vec![];


            drawstuff.push(StyledPolygon {
                color: Vec3d::repeat(0.5),
                line_width: 1.0,
                dash: 2.0,
                points: offset_convex_poly(&inner, allowance, true)?,
            });

            drawstuff.push(StyledPolygon { 
                color: Vec3d::repeat(0.6), 
                line_width: 0.5,
                dash: 0.0,
                points: outer_offset
            });


            drawstuff.push(StyledPolygon {
                color: Vec3d::repeat(0.5),
                line_width: 1.0,
                dash: 4.0,
                points: border1_flip, 
            });

            drawstuff.push(StyledPolygon {
                color: Vec3d::repeat(0.5),
                line_width: 1.0,
                dash: 4.0,
                points: border2_flip, 
            });

            drawstuff.push(StyledPolygon {
                color: Vec3d::zeros(),
                line_width: 0.5,
                dash: 0.0,
                points: inner
            });

            drawstuff.push(StyledPolygon {
                color: Vec3d::zeros(),
                line_width: 0.5,
                dash: 0.0,
                points: border1,
            });

            drawstuff.push(StyledPolygon {
                color: Vec3d::zeros(),
                line_width: 0.5,
                dash: 0.0,
                points: border2, 
            });

                

            let mut verts_rect = Rect2d::empty(); 
            
            for thing in &drawstuff {
                for point in &thing.points {
                    verts_rect.expand(point)
                }
            }

            let y = 0.5*PAGE_LONG_EDGE + ysign * 0.5*(0.5*PAGE_LONG_EDGE - MARGIN);
           
            let page_center = Point2d::new(0.5*PAGE_SHORT_EDGE, y);
            let hdims = 0.5*verts_rect.dims();
            let page_rect = Rect2d { p0: page_center - hdims, p1: page_center + hdims };
            
            let (transform, _) = get_page_transform(&verts_rect, &page_rect, -1.0);
            
            for thing in &mut drawstuff {

                for point in &mut thing.points {
                    *point = transform * *point;
                }

                with_save_restore!(ctx, {

                    if thing.dash != 0.0 {
                        ctx.set_dash(&[thing.dash, thing.dash], 
                                     0.5*thing.dash);
                    }

                    ctx.set_line_width(thing.line_width);
                    ctx.setcolor(&thing.color);
                    ctx.drawpoly(&thing.points);
                    ctx.stroke();

                });
                
            }
            
        }
 
        ctx.show_page();
        
    }

    Ok(())

}
                  

//////////////////////////////////////////////////////////////////////

fn run() -> Result<()> {

    let args: Vec<String> = std::env::args().collect();

    if args.len() != 2 {
        eprintln!("usage: {:?} PATTERNFILE", args[0]);
        std::process::exit(1);
    }

    let filename = Path::new(&args[1]);

    let basename = match filename.file_stem() { 
        None => "output",
        Some(os_str) => os_str.to_str().unwrap_or("output")
    };

    let pdffile = basename.to_owned() + ".pdf";

    let f = File::open(filename)?;
    let mut reader = BufReader::new(f);

    let qs = QuiltSpec::parse(&args[1], &mut reader)?;
    
    let quilt = Quilt::new(qs)?;

    //////////////////////////////////////////////////////////////////////

    let mm = Vec2d::repeat(MARGIN);

    let landscape_dims = Vec2d::new(PAGE_LONG_EDGE, PAGE_SHORT_EDGE);

    let landscape_min = Point2d::origin() + mm;
    let landscape_max = Point2d::origin() + landscape_dims - mm;

    let landscape_rect = Rect2d::new(landscape_min, landscape_max);

    let all_leaf_tris = quilt.pt.final_gen_tris_enumerated().map(|(i, _)| i).collect();

    let surface = cairo::PdfSurface::new(
        landscape_dims[0], landscape_dims[1], &pdffile)?;
    
    let ctx = cairo::Context::new(&surface);

    let draw = |tri_indices, pset| {
        draw_quilt(&ctx, &landscape_rect, &quilt, tri_indices, pset)
    };

    draw(&all_leaf_tris, PageSettings {
        frame: CoordinateFrame::OrigCoords,
        draw_as_finished: false,
        two_color_palette: true,
        label_tris: false,
        label_points: true,
        show_seams: false,
        mirror_x: false,
    })?;

    ctx.show_page();

    draw(&quilt.visible_leaf_tris, PageSettings {
        frame: CoordinateFrame::XformedCoords,
        draw_as_finished: true,
        two_color_palette: true,
        label_tris: false,
        label_points: false,
        show_seams: false,
        mirror_x: false,
    })?;

    ctx.show_page();

    draw(&quilt.visible_top_level_tris, PageSettings {
        frame: CoordinateFrame::XformedCoords,
        draw_as_finished: false,
        two_color_palette: false,
        label_tris: true,
        label_points: false,
        show_seams: true,
        mirror_x: true
    })?;

    ctx.show_page();

    let portrait_dims = Vec2d::new(PAGE_SHORT_EDGE, PAGE_LONG_EDGE);
    let portrait_min = Point2d::origin() + mm;
    let portrait_max = Point2d::origin() + portrait_dims - mm;

    let portrait_rect = Rect2d::new(portrait_min, portrait_max);

    surface.set_size(PAGE_SHORT_EDGE, PAGE_LONG_EDGE)?;

    ctx.set_font_size(16.0);
    ctx.set_source_rgb(0.0, 0.0, 0.0);

    let mut drawables = vec![];

    ctx.set_source_rgb(0.0, 0.0, 0.0);

    let spacing = 12.0;

    let max_width = 0.999 * ((portrait_rect.p1.x - portrait_rect.p0.x) - 3.0*spacing) / 4.0;

    let inset_frac = quilt.qs.line_inset / quilt.qs.scale;

    drawables.push( label_drawable(&ctx, 16.0, "Modules".to_string(), PAGE_SHORT_EDGE, 8.0) );

    let mut counts: HashMap< (TriangleType, TriangleSide, usize), usize > = HashMap::new();

    for (tri, is_visible) in quilt.pt.tris.iter().zip(quilt.tri_overlap_mask.iter()) {

        if *is_visible {

            let level = quilt.pt.generations.len() - tri.generation - 1;

            let key = (tri.ttype, tri.tside, level);

            *counts.entry(key).or_insert(0) += 1;

        }

    }

    for level in 0..quilt.pt.generations.len() {

        for &ttype in &[TriangleType::KiteTriangle, TriangleType::DartTriangle] {

            for &tside in &[TriangleSide::SideRight, TriangleSide::SideLeft] {

                let key = (ttype, tside, level);

                if let Some(count) = counts.get(&key) {

                    if level == 0 {
                        println!("need {:} of {:}", 
                                 count, get_tri_string(ttype, tside, level));
                    }

                    drawables.push(assembly_drawable(&ctx,
                                                     ttype,
                                                     tside,
                                                     level,
                                                     *count, 
                                                     36.0,
                                                     1.45,
                                                     inset_frac,
                                                     max_width,
                                                     spacing)?);
                }

            }

        }
        
        drawables.push(Drawable::LineBreak);

    }

    layout_page(&ctx, &portrait_rect, &drawables, spacing)?;

    let spacing = 4.0;

    drawables.clear();
    drawables.push( label_drawable(&ctx, 16.0, "Instructions".to_string(), PAGE_SHORT_EDGE, 8.0) );

    for (idx, seam) in quilt.seams.iter().enumerate() {

        let cp0 = &seam.child_parts[0];
        let cp1 = &seam.child_parts[1];
        let s = format!("Sew seam {:} joining {:} and {:}", idx+1, cp0, cp1);

        drawables.push( label_drawable(&ctx, 12.0, s, PAGE_SHORT_EDGE, 0.0) );

    }


                                     
                                     

    layout_page(&ctx, &portrait_rect, &drawables, spacing)?;

    draw_templates(&ctx, 
                   quilt.qs.scale,
                   quilt.qs.line_inset)?;

    println!("");
    println!("wrote {:}", pdffile);

    Ok(())
    
}

quick_main!(run);
