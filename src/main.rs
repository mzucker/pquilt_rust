/*

TODO: colors! textures!

 */

// bunch of standard library stuff
use std::path::Path;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::collections::HashMap;
use std::collections::HashSet;
use std::collections::hash_map::Entry::{Occupied, Vacant};

//////////////////////////////////////////////////////////////////////
// use error chain so we can use Result<> everywhere
// for error handling

#[macro_use]
extern crate error_chain;

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

//////////////////////////////////////////////////////////////////////
// define some statically allocated maps for 
// lookups during parsing

use phf::phf_map;

//////////////////////////////////////////////////////////////////////
// define some constants for Penrose tiles

const PHI: f64 = 1.618033988749895;
const INVPHI: f64 = 0.618033988749895;
const PI: f64 = std::f64::consts::PI;
const DEG: f64 = PI / 180.0;

const COS_HALF_APEX: f64 = 0.8090169943749475;
const SIN_HALF_APEX: f64 = 0.5877852522924731;

//////////////////////////////////////////////////////////////////////
// constants for layout

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
// pull in some types from nalgebra

type Vec2d = nalgebra::Vector2<f64>;
type Vec3d = nalgebra::Vector3<f64>;
type Point2d = nalgebra::geometry::Point2<f64>;
type Similarity2d = nalgebra::Similarity2<f64>;
type Translation2d = nalgebra::Translation2<f64>;
type Rotation2d = nalgebra::Rotation2<f64>;
type Transform2d = nalgebra::Transform2<f64>;
type Matrix3d = nalgebra::Matrix3<f64>;

//////////////////////////////////////////////////////////////////////
// Rect2d type has lower-left p0 and upper-right p1

struct Rect2d {

    p0: Point2d,
    p1: Point2d

}

impl Rect2d {

    // empty rectangle has p0 > p1
    fn empty() -> Self {

        let p0 = Point2d::new(f64::MAX, f64::MAX);
        let p1 = -p0;

        Rect2d { p0: p0, p1: p1 }
        
    }

    // new rect from points
    fn new(p0: Point2d, p1: Point2d) -> Self {
        Rect2d { p0: p0, p1: p1 }
    }

    // expand this rect to include the given point
    fn expand(&mut self, p: &Point2d) {
        self.p0 = self.p0.inf(p);
        self.p1 = self.p1.sup(p);
    }

    // dimensions of this rect
    fn dims(&self) -> Vec2d {
        self.p1 - self.p0
    }

    // center of this rect
    fn center(&self) -> Point2d {
        self.p0 + 0.5*(self.p1 - self.p0)
    }
        

}

//////////////////////////////////////////////////////////////////////
// does the line segment from (-u, v) to (u, v) intersect
// the line segment with midpoint (xmid, yymid) and half-span (hx, hy)?
//
// used by Box2d impl below

fn intersect_hline(u: f64, v: f64, 
                   xmid: f64, ymid: f64,
                   hx: f64, hy: f64) -> bool {

    let contains_v = (ymid - v).abs() < hy.abs();

    if !contains_v {

        false 

    } else {

        let x = xmid + (v - ymid) * hx / hy;

        (x.abs() < u) && ((xmid - x).abs() < hx.abs())

    }
    

}

//////////////////////////////////////////////////////////////////////
// 2D box centered at origin with given half-dimensions

struct Box2d {
    half_dims: Vec2d
}

impl Box2d {

    // new from dims
    fn new(half_dims: Vec2d) -> Self {
        Box2d { half_dims: half_dims }
    }

    // does point p lie inside?
    fn contains_point(&self, p: &Point2d) -> bool {
        p.iter().enumerate().all(
            |(i, &v)| v.abs() <= self.half_dims[i]
        )
    }

    // does segment from a to b overl this box?
    fn intersects_segment(&self, a: &Point2d, b: &Point2d) -> bool {

        let h = 0.5*(b - a);
        let c = a + h;

        let r = &self.half_dims;

        intersect_hline(r.x,  r.y, c.x, c.y, h.x, h.y) ||
            intersect_hline(r.x, -r.y, c.x, c.y, h.x, h.y) ||
            intersect_hline(r.y,  r.x, c.y, c.x, h.y, h.x) ||
            intersect_hline(r.y, -r.x, c.y, c.x, h.y, h.x)
        
    }

    // does triangle (v0, v1, v2) overlap this box?
    fn overlaps_tri(&self, v0: &Point2d, v1: &Point2d, v2: &Point2d) -> bool {

        self.contains_point(v0) ||
            self.contains_point(v1) ||
            self.contains_point(v2) ||
            self.intersects_segment(v0, v1) ||
            self.intersects_segment(v1, v2) || 
            self.intersects_segment(v2, v0)
        
    }

}

//////////////////////////////////////////////////////////////////////
//
// make a Transform2d that will translate and scale the given
// verts_rect (input) to the given page_rect (output).
//
// always includes vertical flip because graphics coordinate system
// is left-handed (y increases going down)
//
// optionally include horizontal flip too
// 

#[derive(Debug,PartialEq)]
enum HFlip {
    Yes,
    No
}

fn get_page_transform(verts_rect: &Rect2d, 
                      page_rect: &Rect2d, 
                      hflip: HFlip) -> (Transform2d, f64) {

    let vdims = verts_rect.dims();
    let pdims = page_rect.dims();

    let scl = (pdims.component_div(&vdims)).min();

    let vmid = verts_rect.center();
    let pmid = page_rect.center();

    let translate_page = Translation2d::new(pmid[0], pmid[1]);
    
    let hsign = if hflip == HFlip::Yes { -1.0 } else { 1.0 };

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

//////////////////////////////////////////////////////////////////////
// get homogeneous coordinates (a, b, c) of line from two points
// line is normalized so that a^2 + b^2 = 1
//
// TODO: maybe return Option<Vec3d> to handle p1 == p2?

fn line_from_points(p1: &Point2d, p2: &Point2d) -> Vec3d {

    let p1 = p1.to_homogeneous();
    let p2 = p2.to_homogeneous();
    
    let l = p1.cross(&p2);

    let p = (l[0]*l[0] + l[1]*l[1]).sqrt();

    l / p

}

//////////////////////////////////////////////////////////////////////
// get intersection of lines from homogeneous coordinates

fn intersect(l1: &Vec3d, l2: &Vec3d) -> Result<Point2d> {
    match Point2d::from_homogeneous(l1.cross(l2)) {
        None => bail!("lines don't intersect!"),
        Some(point) => Ok(point)
    }
}

//////////////////////////////////////////////////////////////////////
// get the incenter of triangle (v0, v1, v2) and the inradius too

fn tri_incenter_radius(v0: &Point2d, 
              v1: &Point2d,
              v2: &Point2d) -> Result<(Point2d, f64)> {

    let l01 = line_from_points(v0, v1);
    let l12 = line_from_points(v1, v2);
    let l20 = line_from_points(v2, v0);

    let b1 = l01 - l12;
    let b2 = l12 - l20;

    let pinter = intersect(&b1, &b2)?;

    Ok((pinter, pinter.to_homogeneous().dot(&l01).abs()))

}

//////////////////////////////////////////////////////////////////////
// given a convex polygon return a convex polygon that is offset by
// the given distance.

#[derive(Debug,PartialEq)]
#[allow(dead_code)] // 
enum SnipCorners {
    Yes,
    No
}

fn offset_convex_poly(verts: &Vec<Point2d>, 
                      dist: f64, 
                      snip: SnipCorners) -> Result<Vec<Point2d>> {

    let n = verts.len();

    assert!(n > 2);

    let mut lines = Vec::new();

    for (idx, v0) in verts.iter().enumerate() {
        let v1 = &verts[(idx + 1) % n];
        lines.push(line_from_points(v0, v1));
    }

    let dp = lines[0].dot(&verts[2].to_homogeneous());
    let s = if dp < 0.0 { -1.0 } else { 1.0 };

    for line in &mut lines {
        line[2] += s * dist;
    }

    let mut rval = Vec::new();

    for (idx, l1) in lines.iter().enumerate() {

        let l0 = &lines[(idx + n - 1) % n];

        if snip == SnipCorners::Yes {

            let mut bisector = l1-l0;
            bisector /= Vec2d::new(bisector.x, bisector.y).norm();

            let tangent = s * Vec2d::new(bisector.y, -bisector.x);

            let v_orig = &verts[idx];
            
            let mut lbisect = Vec3d::new(
                tangent.x, tangent.y, 
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

//////////////////////////////////////////////////////////////////////
// given a triangle (v0, v1, v2) return points (b0, b1, b2, b3)
// as shown here:
//
//                 _* v1 
//            b3 _*  \ 
//             _-  *  \
//           _-  _- \  \
//         _-  _- b1 \  \
//       _-  _-       \  \
//     _-  _-          \  \
// v0 *---*-------------*--* v2
//       b0            b2
//
// dist is the distance from line (v0, v1) to line (b0, b1) and 
// line (v1, v2) to line (b1, b2)
//
// dilate is an extra amount to dilate the resulting triangle
// (b0, b1, b2) to ensure drawing with no gaps

fn tri_border(v0: &Point2d, 
              v1: &Point2d,
              v2: &Point2d, 
              dist: f64,
              dilate: f64) -> Result<(Point2d, Point2d, Point2d, Point2d)> {


    let l01 = line_from_points(v0, v1);
    let l12 = line_from_points(v1, v2);
    let l20 = line_from_points(v2, v0);

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

//////////////////////////////////////////////////////////////////////
// some useful types for constructing Penrose tiles
          
#[derive(Debug, PartialEq, PartialOrd, Eq, Ord, Clone, Copy, Hash)]
enum HalfTileType {
    Kite,
    Dart
}

#[derive(Debug, PartialEq, PartialOrd, Eq, Ord, Clone, Copy, Hash)]
enum TriangleSide {
    Right,
    Left
}

const MODULE_SHAPES: [(HalfTileType, TriangleSide); 4] = [
    (HalfTileType::Kite, TriangleSide::Right),
    (HalfTileType::Kite, TriangleSide::Left),
    (HalfTileType::Dart, TriangleSide::Right),
    (HalfTileType::Dart, TriangleSide::Left)
];

// get opposite side
impl TriangleSide {

    fn opposite(self) -> Self {
        match &self {
            TriangleSide::Left => TriangleSide::Right,
            TriangleSide::Right => TriangleSide::Left
        }
    }

}

// (Kite, Right) -> 0, (Dart, Left) -> 3, etc.
fn tri_type_side_index(ttype: HalfTileType,
                       tside: TriangleSide) -> usize {

    2*(ttype as usize) + (tside as usize)

        
}

// (Kite, Right, 0) -> A1, (Dart, Left 2) -> D3, etc.
fn tri_type_side_level_string(ttype: HalfTileType,
                              tside: TriangleSide,
                              level: usize) -> String {
    
    let i = tri_type_side_index(ttype, tside);
    let abcd = "ABCD";
    let slice: &str = &abcd[i..i+1];
    

    format!("{:}{:}", slice, level+1)

}

//////////////////////////////////////////////////////////////////////
// Penrose tile triangle (half-tiles)

#[derive(Debug, PartialEq, Clone)]
struct HalfTile {

    vidx:       [usize; 3],    // vertex indices within Vec<Point2d>
    lidx:       [usize; 3],    // line indices for [(0, 1), (1, 2), (2, 0)]

    ttype:      HalfTileType,  // Kite or Dart
    tside:      TriangleSide,  // Right or Left
    parent:     Option<usize>, // parent triangle index or None for root
    children:   Vec<usize>,    // child triangle indices
    generation: usize          // 0 = root, 1 = children of root, etc.

}

impl HalfTile {

    fn get_lidx(&self, a: usize, b: usize) -> usize {

        debug_assert!(a < 3);
        debug_assert!(b < 3);
        debug_assert!(a != b);

        let a_next = (a + 1) % 3;
        let is_sequential = b == a_next;

        let i = if is_sequential {
            a
        } else {
            b
        };

        self.lidx[i]

    }

    // get verts from external vec using my indices 
    fn get_verts<'a, 'b>(&'a self, verts: &'b Vec<Point2d>) 
                         -> (&'b Point2d, &'b Point2d, &'b Point2d) 
    {
        (&verts[self.vidx[0]],
         &verts[self.vidx[1]],
         &verts[self.vidx[2]])
    }

    // used for debugging
    fn has_vertex(&self, vidx: usize) -> bool {
        self.vidx[0] == vidx ||
            self.vidx[1] == vidx ||
            self.vidx[2] == vidx
    }

    // num_generations - 1 -> 0, 0 -> num_generations - 1
    fn level(&self, num_generations: usize) -> usize {
        num_generations - self.generation - 1
    }

    fn label(&self, num_generations: usize) -> String {
        let level = self.level(num_generations);
        tri_type_side_level_string(self.ttype, self.tside, level)
    }

}



//////////////////////////////////////////////////////////////////////
// misc data structures for tilings

// lines are straight runs that subdivide tiles - they can have 
// multiple vertices along them
#[derive(Debug)]
struct Line {
    parent:  Option<usize>,        // parent half-tile index
    vidx0:   usize,                // starting vertex index
    vidx1:   usize,                // ending vertex index
    vlookup: HashMap<usize, f64>,  // map from vertex index to dist along line
}

impl Line {

    fn new(parent: Option<usize>, vidx0: usize, vidx1: usize) -> Self {

        let mut vlookup = HashMap::new();

        vlookup.insert(vidx0, 0.0);
        vlookup.insert(vidx1, 1.0);

        Line {
            parent: parent,
            vidx0: vidx0,
            vidx1: vidx1,
            vlookup: vlookup,
        }

    }

}

// index edges by indices of two vertices
type VIdxPair = (usize, usize); 

// split line to subdivide half tiles
#[derive(Debug)]
struct HalfTileSplit {
    lidx:  usize, // line index
    vidx0: usize, // first vertex
    vidx1: usize, // second vertex
    tidx0: usize, // left half-tile index
    tidx1: usize, // right half-tile index
}

impl HalfTileSplit {

    fn new(lidx: usize, 
           vidx0: usize, vidx1: usize, 
           tidx0: usize, tidx1: usize) -> Self {

        HalfTileSplit { 
            lidx: lidx,
            vidx0: vidx0, vidx1: vidx1, 
            tidx0: tidx0, tidx1: tidx1 
        }

    }

}

//////////////////////////////////////////////////////////////////////
// Penrose tiling data structure to hold all of the recursive
// subdivision info

#[derive(Debug)]
struct PenroseTiling {
    
    verts:    Vec<Point2d>,             // all vertices
    esplits:  HashMap<VIdxPair, usize>, // map of parent verts -> child vertex
    lines:    Vec<Line>,                // all lines
    htiles:   Vec<HalfTile>,            // all half-tiles

    // generations[i] is the smallest index such that 
    // htiles[generations[i]].generation = i
    generations: Vec<usize>,              

    // tile splits per half-tile index
    tsplits: HashMap<usize, Vec<HalfTileSplit>>

}

impl PenroseTiling {

    // create a PenroseTiling from a single triangle
    // with given type, side

    fn from_triangle(p0: Point2d, p1: Point2d, p2: Point2d,
                     ttype: HalfTileType,
                     tside: TriangleSide) -> Self {

        let tri = HalfTile {
            vidx: [0, 1, 2],
            lidx: [0, 1, 2],
            ttype: ttype,
            tside: tside,
            parent: None,
            children: Vec::new(),
            generation: 0
        };

        let mut lines = vec![];

        for &(vidx0, vidx1) in &[(0, 1), (1, 2), (2, 0)] {

            lines.push(Line::new(None, vidx0, vidx1));

        }

        PenroseTiling {
            
            verts: vec![p0, p1, p2],
            lines: lines,
            esplits: HashMap::new(),
            htiles: vec![tri],
            generations: vec![0],
            tsplits: HashMap::new()
                
            
        }

    }

    fn label_for_htile_idx(&self, tidx: usize) -> String {
        self.htiles[tidx].label(self.generations.len())
    }
    
    // construct a Penrose tiling with a single half-kite
    fn half_kite() -> PenroseTiling {

        let p0 = Point2d::new(0.0, 0.0);
        let p1 = Point2d::new(PHI*COS_HALF_APEX, PHI*SIN_HALF_APEX);
        let p2 = Point2d::new(PHI, 0.0);

        Self::from_triangle(p0, p1, p2, 
                            HalfTileType::Kite, 
                            TriangleSide::Right)
        
    }
    
    // construct a Penrose tiling with a single half-dart
    fn half_dart() -> PenroseTiling {

        let p0 = Point2d::new(0.0, 0.0);
        let p1 = Point2d::new(PHI*COS_HALF_APEX, PHI*SIN_HALF_APEX);
        let p2 = Point2d::new(1.0, 0.0);

        Self::from_triangle(p0, p1, p2,
                            HalfTileType::Dart,
                            TriangleSide::Right)
        
    }

    //////////////////////////////////////////////////

    // subdivide an edge 
    // input: triangle index, vertex indices within triangle
    // output: new vertex index

    fn split_edge(&mut self, tidx: usize, i0: usize, i1: usize) -> usize {

        let tri = &self.htiles[tidx];

        let vidx0 = tri.vidx[i0];
        let vidx1 = tri.vidx[i1];

        let lidx = tri.get_lidx(i0, i1);

        debug_assert!(lidx < self.lines.len());

        let e = self.esplits.entry( (vidx0, vidx1) );

        match e {

            Occupied(o) => {
                *o.get()
            }

            Vacant(v) => {

                let vidx2 = self.verts.len();

                let p0 = self.verts[vidx0];
                let p1 = self.verts[vidx1];

                let pnew = p0 + INVPHI*(p1 - p0);
                self.verts.push(pnew);

                let line = &mut self.lines[lidx];

                debug_assert!(line.vlookup.contains_key(&vidx0));
                debug_assert!(line.vlookup.contains_key(&vidx1));

                let u0 = line.vlookup.get(&vidx0).unwrap();
                let u1 = line.vlookup.get(&vidx1).unwrap();
                
                let unew = u0 + INVPHI*(u1 - u0);

                line.vlookup.insert(vidx2, unew);

                v.insert(vidx2);

                vidx2

            }

        }
        
    }

    // add a line connecting two vertices (one newly created)
    // input: vertex indices
    // output: line index

    fn add_line(&mut self, parent: usize, vidx0: usize, vidx1: usize) -> usize {

        let lidx = self.lines.len();

        self.lines.push(Line::new(Some(parent), vidx0, vidx1));
        
        lidx

    }
    
    // add a half-tile made from new vertices & lines

    fn add_half_tile(&mut self,
                     vidx: [usize; 3],
                     lidx: [usize; 3],
                     ttype: HalfTileType,
                     tside: TriangleSide,
                     tidx_parent: usize) -> usize {

        let generation = self.generations.len() - 1;

        let tidx_new = self.htiles.len();

        for i0 in 0..3 {

            let i1 = (i0 + 1) % 3;
            
            let vidx0 = vidx[i0];
            let vidx1 = vidx[i1];

            let lidx = lidx[i0];

            debug_assert!(self.lines[lidx].vlookup.contains_key(&vidx0));
            debug_assert!(self.lines[lidx].vlookup.contains_key(&vidx1));


        }

        self.htiles[tidx_parent].children.push(tidx_new);
        
        self.htiles.push( HalfTile {
            vidx: vidx,
            lidx: lidx,
            ttype: ttype,
            tside: tside,
            parent: Some(tidx_parent),
            children: Vec::new(),
            generation: generation
        } );

        tidx_new

    }

    // new internal edge within a half-tile between two new half-tiles

    fn split_half_tile(&mut self,
                       parent: usize,
                       lidx: usize,
                       vidx0: usize, vidx1: usize,
                       tidx0: usize, tidx1: usize) {

        let (vidx0, vidx1) = if vidx0 <= vidx1 {
            (vidx0, vidx1)
        } else {
            (vidx1, vidx0)
        };

        debug_assert!(vidx0 < vidx1);
        debug_assert!(tidx0 < tidx1);

        debug_assert!(self.htiles[tidx0].parent == Some(parent));
        debug_assert!(self.htiles[tidx1].parent == Some(parent));

        debug_assert!(self.htiles[tidx0].has_vertex(vidx0));
        debug_assert!(self.htiles[tidx0].has_vertex(vidx1));
        debug_assert!(self.htiles[tidx1].has_vertex(vidx0));
        debug_assert!(self.htiles[tidx1].has_vertex(vidx1));

        debug_assert!(self.lines[lidx].vlookup.contains_key(&vidx0));
        debug_assert!(self.lines[lidx].vlookup.contains_key(&vidx1));

        self.tsplits.entry(parent).or_insert(vec![]).push(
            HalfTileSplit::new(lidx, vidx0, vidx1, tidx0, tidx1) );

    }

    //////////////////////////////////////////////////

    // return index of first tile in most recent generation
    fn first_tile_in_last_gen(&self) -> usize {

        let gfinal = self.generations.len()-1;
        let tidx_first_gfinal = self.generations[gfinal];

        debug_assert!(self.htiles.last().unwrap().generation == gfinal);
        debug_assert!(tidx_first_gfinal == 0 || 
                      self.htiles[tidx_first_gfinal-1].generation == gfinal - 1);

        tidx_first_gfinal

    }

    // return enumeration of half-tiles in most recent generation
    fn last_gen_htiles<'a>(&'a self) -> impl Iterator<Item=(usize, &'a HalfTile)> {

        let tidx_first_gfinal = self.first_tile_in_last_gen();
        
        let slice = &self.htiles[tidx_first_gfinal..];
        
        slice.iter().enumerate().map(move |(i, t)| (i + tidx_first_gfinal, t))

    }

    //////////////////////////////////////////////////

    // subdivide by splitting each half-kite into 3 half-tiles
    // and each half-dart into 2 half-tiles

    fn subdivide(&mut self) {

        if self.htiles.is_empty() || self.generations.is_empty() {
            return;
        }

        let tidx_first_gfinal = self.first_tile_in_last_gen();

        //println!("at start of subdivide got {:} verts, {:} htiles",
        //self.verts.len(), self.htiles.len());

        let last_htiles = self.htiles[tidx_first_gfinal..].to_vec();
        
        self.generations.push(self.htiles.len());

        for (tidx_offset, tri) in last_htiles.iter().enumerate() {

            let tidx = tidx_offset + tidx_first_gfinal;

            let vidx0 = tri.vidx[0];
            let vidx1 = tri.vidx[1];
            let vidx2 = tri.vidx[2];

            let lidx0 = tri.lidx[0];
            let lidx1 = tri.lidx[1];
            let lidx2 = tri.lidx[2];

            let flip = tri.tside.opposite();
            let same = tri.tside;

            match tri.ttype {

                HalfTileType::Kite => {

                    let vidx3 = self.split_edge(tidx, 0, 2);
                    let vidx4 = self.split_edge(tidx, 1, 0);

                    let lidx3 = self.add_line(tidx, vidx1, vidx3);
                    let lidx4 = self.add_line(tidx, vidx3, vidx4);

                    let a = self.add_half_tile([vidx0, vidx3, vidx4],
                                               [lidx2, lidx4, lidx0],
                                               HalfTileType::Dart, flip,
                                               tidx);

                    let b = self.add_half_tile([vidx1, vidx4, vidx3],
                                               [lidx0, lidx4, lidx3],
                                               HalfTileType::Kite, flip,
                                               tidx);

                    let c = self.add_half_tile([vidx1, vidx2, vidx3],
                                               [lidx1, lidx2, lidx3],
                                               HalfTileType::Kite, same,
                                               tidx);

                    self.split_half_tile(tidx, lidx3, vidx1, vidx3, b, c);
                    self.split_half_tile(tidx, lidx4, vidx3, vidx4, a, b);

                }

                HalfTileType::Dart => {

                    let vidx3 = self.split_edge(tidx, 0, 1);

                    let lidx3 = self.add_line(tidx, vidx2, vidx3);

                    let a = self.add_half_tile([vidx0, vidx3, vidx2], 
                                               [lidx0, lidx3, lidx2],
                                               HalfTileType::Kite, same,
                                               tidx);
                    
                    let b = self.add_half_tile([vidx1, vidx2, vidx3],
                                               [lidx1, lidx3, lidx0],
                                               HalfTileType::Dart, same,
                                               tidx);

                    self.split_half_tile(tidx, lidx3, vidx2, vidx3, a, b);
                    
                }

            };

        }

    }

    //////////////////////////////////////////////////

    // for each half-tile, leaf half-tiles that descend from it
    fn get_child_leafs(&self) -> Vec<Vec<usize>> {

        let mut child_leafs = vec![ vec![] ; self.htiles.len() ];

        for (orig_tidx, _) in self.last_gen_htiles() {
            let mut tidx = orig_tidx;
            loop { 
                child_leafs[tidx].push(orig_tidx);
                match self.htiles[tidx].parent {
                    None => { break; }
                    Some(parent_tidx) => { tidx = parent_tidx; }
                }
            }
        }

        child_leafs

    }

    //////////////////////////////////////////////////
    // make a module of the given type, size, &level

    fn construct_module(ttype: HalfTileType,
                        tside: TriangleSide,
                        level: usize,
                        rotate: bool) -> PenroseTiling {

        let mut pt = match ttype {
            HalfTileType::Kite => PenroseTiling::half_kite(),
            HalfTileType::Dart => PenroseTiling::half_dart()
        };
        
        for v in &mut pt.verts {
            if tside != pt.htiles[0].tside {
                v.y = -v.y;
            }
            if rotate {
                *v = Point2d::new(v.y, -v.x);
            }
        }

        pt.htiles[0].tside = tside;

        for _ in 0..level {
            pt.subdivide()
        }

        pt

    }

}

//////////////////////////////////////////////////////////////////////

// module visibility 
#[derive(Debug,PartialEq,Clone,Copy)]
enum Visibility {
    Invisible, // no child leaf modules overlap view rect
    Partial,   // some child leaf modules overlap view rect
    Full       // all child leaf modules overlap view rect
}

#[derive(Debug, Clone)]
struct VertexInterval {
    u0: f64,
    u1: f64,
    vidx0: usize,
    vidx1: usize
}

impl VertexInterval {

    fn is_valid(&self) -> bool {
        
        self.u1 > self.u0 && 
            self.vidx0 != usize::MAX && 
            self.vidx1 != usize::MAX
        
    }

    fn empty() -> Self {
        VertexInterval {
            u0: 1.0,
            u1: 0.0,
            vidx0: usize::MAX,
            vidx1: usize::MAX
        }
    }

    fn expand(&mut self, u: f64, vidx: usize) {

        if u < self.u0 {
            self.u0 = u;
            self.vidx0 = vidx;
        }

        if u > self.u1 {
            self.u1 = u;
            self.vidx1 = vidx;
        }

    }

    fn intersection(&self, other: &Self) -> Self {

        if !self.is_valid() {
            
            self.clone()

        } else if !other.is_valid() {

            other.clone()

        } else {

            let (u0, vidx0) = if self.u0 > other.u0 {
                (self.u0, self.vidx0)
            } else {
                (other.u0, other.vidx0)
            };

            let (u1, vidx1) = if self.u1 < other.u1 {
                (self.u1, self.vidx1)
            } else {
                (other.u1, other.vidx1)
            };

            VertexInterval { u0: u0, vidx0: vidx0, u1: u1, vidx1: vidx1 }

        }
            

    }
            
}

#[derive(Debug)]
struct SeamDrawInfo {
    lidx: usize,
    vidx0: usize,
    vidx1: usize,
}

#[derive(Debug)]
struct Seam {

    lidx:  usize,
    interval: VertexInterval,

    tidx:  usize,

    length: usize,

    line_coeffs: Vec3d,
    centroid: Vec3d,

    key: (i64, i64, f64, f64),

}

impl Seam {

    fn new(pt: &PenroseTiling, 
           vvec: &Vec<Point2d>, 
           pmid: &Point2d,
           precis: f64,
           lidx: usize, 
           interval: VertexInterval) -> Self {

        debug_assert!(interval.is_valid());

        let mut length = 0;

        let line = &pt.lines[lidx];

        debug_assert!(line.parent != None);

        for (_, &u) in &line.vlookup {

            if u >= interval.u0 && u <= interval.u1 {
                length += 1;
            }

        }

        let v0 = &vvec[interval.vidx0];
        let v1 = &vvec[interval.vidx1];

        let v = v0 + 0.5*(v1 - v0);

        let q = (v - pmid) / precis;

        let key = (q.y as i64, q.x as i64, q.y, q.x);


        Seam {
            lidx: lidx,
            interval: interval,
            tidx: line.parent.unwrap(),
            line_coeffs: line_from_points(v0, v1),
            centroid: Point2d::from(0.5 * (v0.coords + v1.coords)).to_homogeneous(),
            key: key,
            length: length - 1,
        }

    }

}

struct LineSideInfo {
    interval: VertexInterval
}

impl LineSideInfo {

    fn new() -> Self {
        LineSideInfo {
            interval: VertexInterval::empty(),
        }
    }

    fn insert(&mut self, u0: f64, vidx0: usize, u1: f64, vidx1: usize) {

        self.interval.expand(u0, vidx0);
        self.interval.expand(u1, vidx1);
        
    }
    
}

impl Default for LineSideInfo {

    fn default() -> Self { LineSideInfo::new() }

}

struct ModuleInfo {
    tidx: usize,
    centroid: Vec3d,
    edge_list: Vec<VIdxPair>
}

fn get_external_vertices(module_info: &Vec<ModuleInfo>,
                         available_modules: &Vec<usize>) -> Vec<usize>

{


    let mut edge_map: HashMap<VIdxPair, usize> = HashMap::new();

    for &midx in available_modules {
        for &key in &module_info[midx].edge_list {
            *edge_map.entry(key).or_insert(0) += 1;
        }
    }

    let mut vset = HashSet::new();

    for ((vidx0, vidx1), cnt) in edge_map {
        debug_assert!(cnt == 1 || cnt == 2);
        if cnt == 1 {
            vset.insert(vidx0);
            vset.insert(vidx1);
        }
    }

    let mut v = vset.iter().cloned().collect::<Vec<usize>>();
    v.sort();

    v

}


fn get_module_info(pt: &PenroseTiling,
                   child_leafs: &Vec<Vec<usize>>,
                   visible_top_level_htiles: &Vec<usize>,
                   vvec: &Vec<Point2d>) ->
    Vec<ModuleInfo>
{

    let mut module_info = vec![];

    for &tidx in visible_top_level_htiles {

        let htile = &pt.htiles[tidx];

        let (v0, v1, v2) = htile.get_verts(&vvec);

        let centroid = Point2d::from((v0.coords + v1.coords + v2.coords) / 3.0);

        let mut edge_map: HashMap<VIdxPair, usize> = HashMap::new();

        for &cidx in &child_leafs[tidx] {

            let child_htile = &pt.htiles[cidx];

            for i0 in 0..3 {
                let i1 = (i0 + 1) % 3;
                let vidx0 = child_htile.vidx[i0];
                let vidx1 = child_htile.vidx[i1];
                let (vidx0, vidx1) = if vidx0 < vidx1 {
                    (vidx0, vidx1)
                } else {
                    (vidx1, vidx0)
                };
                debug_assert!(vidx0 < vidx1);
                let key = (vidx0, vidx1);
                *edge_map.entry(key).or_insert(0) += 1;
            }

        }

        let mut edge_list: Vec<VIdxPair> = vec![];

        for (key, cnt) in edge_map {
            debug_assert!(cnt == 1 || cnt == 2);
            if cnt == 1 {
                edge_list.push(key)
            }
        }

        edge_list.sort();

        module_info.push(ModuleInfo {
            tidx: tidx,
            centroid: centroid.to_homogeneous(),
            edge_list: edge_list
        });

    }

    module_info
    
}

fn find_best_seam(seams: &Vec<Seam>,
                  lines: &Vec<Line>,
                  external_verts: &Vec<usize>,
                  available_seams: &Vec<usize>) ->
    (usize, [Vec<usize>; 2])

{

    let mut best_sidx = usize::MAX;
    let mut best_score = (0, 0);
    let mut best_splits = [vec![], vec![]];

    if available_seams.len() == 1 {
        return (available_seams[0], best_splits);
    }

    for &sidx in available_seams {

        let seam = &seams[sidx];
        
        let line = &lines[seam.lidx];
        let interval = &seam.interval;

        let mut num_external = 0;
        
        for vidx in external_verts {
            if let Some(&u) = line.vlookup.get(vidx) {
                if u >= interval.u0 && u <= interval.u1 {
                    num_external += 1;
                }
            }
        }

        debug_assert!(num_external >= 0 && num_external <= 2);

        if num_external == 2 {

            let mut splits = [vec![], vec![]];
            
            for &other_sidx in available_seams {

                if other_sidx == sidx { continue; }

                let is_right = seam.line_coeffs.dot(&seams[other_sidx].centroid) > 0.0;
                let side_idx = is_right as usize;

                splits[side_idx].push(other_sidx);


            }

            let min_seams = splits[0].len().min(splits[1].len());

            let score = (min_seams, seam.length);

            /*
            println!("seam L{:} has score ({:},{:}) with {:} seams to left, {:} to right",
                     seams[sidx].lidx, min_seams, seam.length, splits[0].len(), splits[1].len());
            */

            if best_sidx == usize::MAX || score > best_score {
                best_score = score;
                best_sidx = sidx;
                best_splits = splits;
            }

        }

    }

    /*
    println!("best was seam L{:} with score ({:}, {:})",
             seams[best_sidx].lidx, best_score.0, best_score.1);
    */

    debug_assert!(best_sidx != usize::MAX);

    (best_sidx, best_splits)

}

#[derive(Debug,Clone,Copy)]
enum TreeChild {
    Empty,
    Module(usize),
    Seam(usize)
}

#[derive(Debug,Clone,Copy)]
struct TreeSeamInfo {
    depth: usize,
    parent_sidx: usize,
    children: [TreeChild; 2]
}

impl Default for TreeSeamInfo {
    fn default() -> Self { 
        TreeSeamInfo { 
            depth: usize::MAX, 
            parent_sidx: usize::MAX, 
            children: [ TreeChild::Empty, TreeChild::Empty ]
        }
    }
}

fn subdivide_r(pt: &PenroseTiling,
               seams: &Vec<Seam>,
               module_info: &Vec<ModuleInfo>,
               available_modules: Vec<usize>,
               available_seams: Vec<usize>,
               depth: usize, 
               parent_sidx: usize,
               tree_seam_info: &mut Vec<TreeSeamInfo>) -> usize

{

    debug_assert!(!available_seams.is_empty());
    debug_assert!(available_modules.len() >= 2);

    // get external vertices
    let external_verts = get_external_vertices(
        module_info, &available_modules);


    // find all the seams that contain external vertices
    let (sidx, mut seam_splits) = find_best_seam(seams, &pt.lines, 
                                             &external_verts, &available_seams);

    
    let indent = "  ".repeat(depth);

    println!("{:}L{:}{:}", indent, seams[sidx].lidx,
             if parent_sidx == usize::MAX {
                 "".to_string()
             } else {
                 format!(" (child of L{:})", seams[parent_sidx].lidx)
             });

    let seam = &seams[sidx];

    let mut module_splits = [vec![], vec![]];

    for &midx in &available_modules {

        let is_right = seam.line_coeffs.dot(&module_info[midx].centroid) > 0.0;

        let side_idx = is_right as usize;

        module_splits[side_idx].push(midx);

    }

    let mut children = [ TreeChild::Empty, TreeChild::Empty ];

    for side in 0..2 {

        let side_available_modules = std::mem::take(&mut module_splits[side]);
        let side_available_seams = std::mem::take(&mut seam_splits[side]);

        if side_available_seams.is_empty() {

            debug_assert!(side_available_modules.len() == 1);

            let midx = side_available_modules[0];

            let indent = "  ".repeat(depth+1);

            println!("{:}module {:}", 
                     indent, pt.label_for_htile_idx(module_info[midx].tidx));

            children[side] = TreeChild::Module(midx);

        } else {

            debug_assert!(side_available_modules.len() > 1);

            let child_sidx = subdivide_r(pt, seams, module_info,
                                         side_available_modules,
                                         side_available_seams,
                                         depth+1, sidx, 
                                         tree_seam_info);
            
            children[side] = TreeChild::Seam(child_sidx);

        }

    }

    tree_seam_info[sidx] = TreeSeamInfo { 
        depth: depth,
        parent_sidx: parent_sidx,
        children: children
    };

    sidx


}

fn subdivide(pt: &PenroseTiling,
             seams: &Vec<Seam>,
             module_info: &Vec<ModuleInfo>) ->
    Vec<TreeSeamInfo>

{

    let available_modules = (0..module_info.len()).collect::<Vec<usize>>();
    let available_seams = (0..seams.len()).collect::<Vec<usize>>();

    let mut tree_seam_info: Vec<TreeSeamInfo> = 
        vec![TreeSeamInfo::default(); seams.len()];

    println!("subdividing along seams:\n");

    subdivide_r(pt, seams, module_info, 
                available_modules, available_seams,
                0, usize::MAX,
                &mut tree_seam_info);


    println!("");

    tree_seam_info

}

fn batch_seams(pt: &PenroseTiling,
               seams: Vec<Seam>,
               module_info: Vec<ModuleInfo>,
               tree_seam_info: Vec<TreeSeamInfo>) ->
    (Vec<SeamDrawInfo>, Vec<Vec<String>>) 
{

    let mut sidx_by_depth = (0..seams.len()).collect::<Vec<usize>>();
    
    sidx_by_depth.sort_by_key(|sidx| (usize::MAX - tree_seam_info[*sidx].depth,
                                      *sidx));

    let mut seams_remaining = seams.len();
    let mut seam_is_sewn = vec![false; seams.len()];

    let mut seams_reordered = vec![];
    let mut instructions = vec![];

    let mut output_idx = vec![usize::MAX; seams.len()];

    while seams_remaining > 0 {

        let mut seam_busy = vec![false; seams.len()];
        let mut batch = vec![];

        for &sidx in &sidx_by_depth {
            if !seam_is_sewn[sidx] && !seam_busy[sidx] {
                batch.push(sidx);
                seam_is_sewn[sidx] = true;
                seams_remaining -= 1;
                let mut sidx = sidx;
                while sidx != usize::MAX {
                    seam_busy[sidx] = true;
                    sidx = tree_seam_info[sidx].parent_sidx;
                }
            }
        }
        
        batch.sort_by(|sidx_a, sidx_b| {
            seams[*sidx_b].key.partial_cmp(&seams[*sidx_a].key).unwrap()
        });

        let mut batch_insructions = vec![];

        for sidx in batch {

            let seam = &seams[sidx];

            output_idx[sidx] = seams_reordered.len();

            seams_reordered.push(
                SeamDrawInfo {
                    lidx: seam.lidx,
                    vidx0: seam.interval.vidx0,
                    vidx1: seam.interval.vidx1
                }
            );

            let mut child_names = [ String::new(), String::new() ];

            for side in 0..2 {

                child_names[side] = match tree_seam_info[sidx].children[side] {
                    
                    TreeChild::Empty => { "???".to_string() },

                    TreeChild::Module(midx) => {
                        pt.label_for_htile_idx(module_info[midx].tidx)
                    },

                    TreeChild::Seam(other_sidx) => {
                        format!("assembly{:}",
                                output_idx[other_sidx]+1)
                    }

                };

            }

            let instruction = format!("Sew seam {:} joining {:} and {:}", 
                                      seams_reordered.len(),
                                      child_names[0], child_names[1]);

            batch_insructions.push(instruction);

        }

        instructions.push(batch_insructions);

    }

    (seams_reordered, instructions)
    
}
               


fn gather_seams(pt: &PenroseTiling,
                child_leafs: &Vec<Vec<usize>>,
                visible_top_level_htiles: &Vec<usize>,
                vvec: &Vec<Point2d>, 
                precis: f64) ->
    (Vec<SeamDrawInfo>, Vec<Vec<String>>)
{

    let mut line_sides: HashMap<usize, [LineSideInfo; 2]> = HashMap::new();

    let module_info = get_module_info(pt, child_leafs, 
                                      visible_top_level_htiles, vvec);

    let mut verts_rect = Rect2d::empty();
    
    for minfo in module_info.iter() {

        let htile = &pt.htiles[minfo.tidx];

        for i0 in 0..3 {

            let tri_vidx0 = htile.vidx[i0];
            let tri_vidx1 = htile.vidx[(i0 + 1) % 3];

            let lidx = htile.lidx[i0];
            let line = &pt.lines[lidx];

            verts_rect.expand(&vvec[tri_vidx0]);

            let line_coeffs = line_from_points(&vvec[line.vidx0], 
                                               &vvec[line.vidx1]);

            let is_right = line_coeffs.dot(&minfo.centroid) > 0.0;

            debug_assert!(line.vlookup.contains_key(&tri_vidx0));
            debug_assert!(line.vlookup.contains_key(&tri_vidx1));
            debug_assert!(!line.vlookup.contains_key(&htile.vidx[(i0 + 2) % 3]));

            let u0 = *line.vlookup.get(&tri_vidx0).unwrap();
            let u1 = *line.vlookup.get(&tri_vidx1).unwrap();

            let side_idx = is_right as usize;

            line_sides.entry(lidx).or_default()[side_idx].insert(
                u0, tri_vidx0, u1, tri_vidx1);

        }
        

    }

    let mut seams = vec![];
   
    for (lidx, sides) in line_sides.iter_mut() {

        let interval = sides[0].interval.intersection(&sides[1].interval);

        if interval.is_valid() {

            seams.push(Seam::new(pt, vvec, &verts_rect.p1, precis, *lidx, interval));

        }

    }

    seams.sort_by_key(|seam| seam.lidx);

    let tree_seam_info = subdivide(pt, &seams, &module_info);

    batch_seams(pt, seams, module_info, tree_seam_info)

}

//////////////////////////////////////////////////////////////////////


type TriFunc = fn() -> PenroseTiling;

const VALID_SOURCES: phf::Map<&'static str, TriFunc> = phf_map! {
    "half_kite" => PenroseTiling::half_kite,
    "half_dart" => PenroseTiling::half_dart,
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
    Verts { dtype: DimType, vidx: Vec<usize> }
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
                    angle *= DEG;
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
                                                   vidx: verts};

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



#[allow(dead_code)]
struct Quilt {
    
    qs: QuiltSpec,
    pt: PenroseTiling,

    raw_box: Box2d, 
    xform_fwd: Similarity2d,
    xform_inv: Similarity2d,
    
    xformed_verts: Vec<Point2d>,

    child_leafs: Vec<Vec<usize>>,

    htile_visibility: Vec<Visibility>,

    visible_leaf_htiles: Vec<usize>,
    visible_top_level_htiles: Vec<usize>,

    seams: Vec<SeamDrawInfo>,
    instructions: Vec<Vec<String>>
        

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

        println!("got {:} root htiles, {:} vertices", 
                 pt.htiles.len(), pt.verts.len());
        
        for _ in 0..qs.depth {
            pt.subdivide();

            println!("got {:} htiles ({:} leaf htiles), {:} vertices", 
                     pt.htiles.len(), 
                     pt.htiles.len() - pt.generations.last().unwrap(),
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

            RectType::Verts{dtype, vidx} => {

                let mut r = Vec2d::new(0.0, 0.0);

                for i in vidx {
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

        let mut htile_visibility = vec![Visibility::Invisible; pt.htiles.len()];
        let mut visible_leaf_htiles = Vec::new();

        for (tidx, tri) in pt.last_gen_htiles() {

            let (v0, v1, v2) = tri.get_verts(&xformed_verts);

            if raw_box.overlaps_tri(v0, v1, v2) {

                htile_visibility[tidx] = Visibility::Full;

                visible_leaf_htiles.push(tidx);
                
            }

        }

        let tidx_first_gfinal = pt.first_tile_in_last_gen();

        for tidx in 0..tidx_first_gfinal {

            let mut all_visible = true;
            let mut any_visible = false;

            for &cidx in child_leafs[tidx].iter() {

                if htile_visibility[cidx] == Visibility::Full {
                    any_visible = true;
                } else {
                    all_visible = false;
                }

            }

            if all_visible {
                htile_visibility[tidx] = Visibility::Full;
            } else if any_visible {
                htile_visibility[tidx] = Visibility::Partial;
            }

        }

        let mut visible_top_level_htiles = Vec::new();

        for (i, t) in pt.htiles.iter().enumerate() {
            
            let me_overlaps = htile_visibility[i] == Visibility::Full;

            let parent_overlaps = match t.parent {

                None => false,

                Some(parent_tidx) => htile_visibility[parent_tidx] == Visibility::Full

            };

            if me_overlaps && !parent_overlaps {
                visible_top_level_htiles.push(i)
            }
            
        }

        let precis = qs.scale * 2.0;

        let (seams, instructions) = gather_seams(&pt, 
                                                 &child_leafs,
                                                 &visible_top_level_htiles, 
                                                 &xformed_verts,
                                                 precis);


        Ok(Quilt {

            qs: qs,
            pt: pt,

            raw_box: raw_box,
            xform_fwd: xform_fwd,
            xform_inv: xform_inv,
            xformed_verts: xformed_verts,

            child_leafs: child_leafs,

            htile_visibility: htile_visibility,

            visible_leaf_htiles: visible_leaf_htiles,
            visible_top_level_htiles: visible_top_level_htiles,

            seams: seams,
            instructions: instructions
            
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
    label_htiles: bool,
    label_points: bool,
    show_seams: bool,
    hflip: HFlip

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
             num_generations: usize,
             t: &HalfTile,
             xverts: &Vec<Point2d>) -> Result<()> {

    let (v0, v1, v2) = t.get_verts(&xverts);

    let vdiff = match t.tside {
        TriangleSide::Left => v2 - v0, 
        TriangleSide::Right => v0 - v2
    };

    let (center, r) = tri_incenter_radius(v0, v1, v2)?;

    let level = t.level(num_generations);
    let text = tri_type_side_level_string(t.ttype, t.tside, level);
    
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
                  pt: &PenroseTiling,
                  child_leafs: &Vec<Vec<usize>>,
                  xverts: &Vec<Point2d>,
                  draw_lengths: &DrawLengths,
                  tri_indices: &Vec<usize>,
                  two_color_palette: bool,
                  label_htiles: bool,
                  label_points: bool) -> Result<()> {

    let lw = draw_lengths.line_width;

    ctx.setcolor(&Vec3d::repeat(0.6));
    
    with_save_restore!(ctx, { 
        
        ctx.set_line_join(cairo::LineJoin::Round);
        ctx.set_line_cap(cairo::LineCap::Round);

        ctx.set_line_width(lw);

        for &i in tri_indices.iter() {

            let t = &pt.htiles[i];
            
            let (v0, v1, v2) = t.get_verts(&xverts);
            
            ctx.drawtri(v0, v1, v2);

        }

        ctx.fill_preserve();
        ctx.stroke();

    });


    for &i in tri_indices.iter() {

        let t = &pt.htiles[i];

        let mut cidx = tri_type_side_index(t.ttype, t.tside);

        if two_color_palette {
            cidx -= cidx % 2;
        }

        for &c in &child_leafs[i] {

            let t = &pt.htiles[c];
            
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
    

    if label_htiles {

        ctx.set_source_rgba(0.75, 0.0, 0.0, 0.625);

        for &i in tri_indices.iter() {

            label_tri(ctx, pt.generations.len(), &pt.htiles[i], &xverts)?;

        }
        
    }


    if label_points { 

        with_save_restore!(ctx, {

            ctx.set_source_rgb(0.0, 0.0, 0.0);

            let mut vverts: HashSet<usize> = HashSet::new();

            for &i in tri_indices.iter() {
                let t = &pt.htiles[i];
                for &v in &t.vidx {
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

        for tri in tri_indices.iter().map(|&i| &quilt.pt.htiles[i]) {
            for &i in &tri.vidx {
                verts_rect.expand(&vvec[i]);
            }
        }

        for p in boxes.get(&DimType::Raw).unwrap() {
            verts_rect.expand(p);
        }


    }

    
    let (transform, scl) = get_page_transform(&verts_rect, &page_rect, pset.hflip);

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
                       pset.label_htiles,
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
            ctx.set_font_size(10.0);

            let xform = match pset.frame {
                CoordinateFrame::OrigCoords => transform * quilt.xform_fwd,
                CoordinateFrame::XformedCoords => transform
            };

            for (sidx, seam) in quilt.seams.iter().enumerate() {
                
                let p0 = xform * vvec[seam.vidx0];
                let p1 = xform * vvec[seam.vidx1];

                let d10 = p1 - p0;

                let len = d10.norm();

                let ctr = p0 + 0.5*d10;

                let dir = d10 / len;

                let s = format!("{:}", sidx+1);
                //let s = format!("L{:}", seam.lidx);
                let text = s.as_str();

                let extents = ctx.text_extents(text);

                let r = Vec2d::new(extents.width, extents.height).norm();

                //ctx.arc(ctr.x, ctr.y, r + 2.0, 0.0, 2.0*PI);
                //ctx.fill_preserve();

                let p0in = p0 + 8.0*dir;
                let p1in = p1 - 8.0*dir;

                ctx.set_source_rgb(0.0, 0.0, 0.0);
                ctx.moveto(&p0in);
                ctx.lineto(&p1in);
                ctx.set_line_width(5.0);
                ctx.stroke();

                ctx.set_line_width(1.0);
                ctx.arc(ctr.x, ctr.y, 0.5*r+3.0, 0.0, 2.0*PI);
                ctx.fill();
                
                
                ctx.set_source_rgb(1.0, 1.0, 1.0);
                ctx.moveto(&p0in);
                ctx.lineto(&p1in);
                ctx.set_line_width(3.0);
                ctx.stroke();

                ctx.set_line_width(1.0);
                ctx.arc(ctr.x, ctr.y, 0.5*r+2.0, 0.0, 2.0*PI);
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

fn vstack<'a>(dvec: Vec<Drawable<'a>>,
              spacing: f64) -> Drawable<'a> {

    let mut output_dims = Vec2d::new(0.0, 0.0);

    for (i, d) in dvec.iter().enumerate() {
        if let Drawable::Box { dims, .. } = d {
            output_dims.x = output_dims.x.max(dims.x);
            output_dims.y += dims.y;
            if i > 0 { output_dims.y += spacing; }
        }
    }

    let drawfunc = move |ctx: &cairo::Context| {
        let mut ty = 0.0;
        for d in dvec.iter() { 
            if let Drawable::Box { dims, drawfunc } = d {
                with_save_restore!(ctx, {
                    ctx.translate(0.0, ty);
                    drawfunc(ctx)?;
                });
                ty += dims.y + spacing;
            }
        }
        Ok(())
    };

    Drawable::Box { dims: output_dims, 
                    drawfunc: Box::new(drawfunc) }

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
                         ttype: HalfTileType,
                         tside: TriangleSide,
                         level: usize, 
                         count: usize,
                         base_scale: f64,
                         upscale: f64,
                         inset_frac: f64,
                         max_width: f64,
                         spacing: f64) -> Drawable<'a> {

    let mut pt = PenroseTiling::construct_module(ttype, tside, level, true);

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

    let (transform, scl) = get_page_transform(&verts_rect, &page_rect, HFlip::Yes);

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
            pt.htiles.len()
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

        if let Some(splits) = pt.tsplits.get(&0) {

            with_save_restore!(ctx, { 

                let (v0, v1, v2) = pt.htiles[0].get_verts(&pt.verts);

                ctx.moveto(v0);
                ctx.lineto(v1);
                ctx.lineto(v2);
                ctx.close_path();
                ctx.clip();
                
                ctx.set_line_cap(cairo::LineCap::Round);
                ctx.set_source_rgba(0.75, 0.0, 0.0, 0.625);
                ctx.set_line_width(0.02*vdims.x);

                for s in splits.iter() {

                    let v0 = &pt.verts[s.vidx0];
                    let v1 = &pt.verts[s.vidx1];

                    ctx.moveto(v0);
                    ctx.lineto(v1);

                    
                }

                ctx.stroke();

            } );

            
        }

        Ok(())
                       
    };

    let b = Drawable::Box { dims: vdims, drawfunc: Box::new(drawfunc) };

    let text = format!("{:}× {:}", count, tri_type_side_level_string(ttype, tside, level));

    vstack( vec![b, label_drawable(ctx, 12.0, text, -1.0, 0.0)], spacing )


}
              
//////////////////////////////////////////////////////////////////////

fn offset_and_flip(points: &Vec<Point2d>, dist: f64) -> Result<Vec<Point2d>> {

    assert!(points.len() > 2);

    let mut offset = offset_convex_poly(&points, dist, SnipCorners::Yes)?;

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

    for &(ttype, tside) in &MODULE_SHAPES {
        
        let mut pt = PenroseTiling::construct_module(ttype, tside, 0, false);

        let ysign = match tside {
            TriangleSide::Right => -1.0,
            TriangleSide::Left => 1.0
        };
        
        let rotate = Rotation2d::new(ysign * 36.0 * DEG);

        for v in &mut pt.verts {
            *v = *v * scale * INCH;
            *v = rotate * *v;
        }

        let (v0, v1, v2) = pt.htiles[0].get_verts(&pt.verts);

        let outer_tri = vec![*v0, *v1, *v2];


        let (b0, b1, b2, b3) = tri_border(v0, v1, v2, line_inset*INCH, 0.0)?;

        let inner = vec![b0, b1, b2];
        let border1 = vec![b0, b1, b3, *v0];
        let border2 = vec![b2, b3, *v1, *v2];

        let allowance = 0.5 * INCH;

        let border1_flip = offset_and_flip(&border1, allowance)?;
        let border2_flip = offset_and_flip(&border2, allowance)?;

        let outer_offset = offset_convex_poly(&outer_tri, 0.25*INCH, SnipCorners::Yes)?;

        let mut drawstuff = vec![];


        drawstuff.push(StyledPolygon {
            color: Vec3d::repeat(0.5),
            line_width: 1.0,
            dash: 2.0,
            points: offset_convex_poly(&inner, allowance, SnipCorners::Yes)?,
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
        
        let (transform, _) = get_page_transform(&verts_rect, &page_rect, HFlip::Yes);
        
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
        
        if tside == TriangleSide::Left {
            ctx.show_page();
        }
        
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

    let tidx0 = quilt.pt.first_tile_in_last_gen();
    let tidx1 = quilt.pt.htiles.len();

    let all_leaf_htiles = (tidx0..tidx1).collect();

    let surface = cairo::PdfSurface::new(
        landscape_dims[0], landscape_dims[1], &pdffile)?;
    
    let ctx = cairo::Context::new(&surface);

    let draw = |tri_indices, rect, pset| {
        draw_quilt(&ctx, rect, &quilt, tri_indices, pset)
    };

    draw(&all_leaf_htiles, &landscape_rect, PageSettings {
        frame: CoordinateFrame::OrigCoords,
        draw_as_finished: false,
        two_color_palette: true,
        label_htiles: false,
        label_points: true,
        show_seams: false,
        hflip: HFlip::No
    })?;

    ctx.show_page();

    draw(&quilt.visible_leaf_htiles, &landscape_rect, PageSettings {
        frame: CoordinateFrame::XformedCoords,
        draw_as_finished: true,
        two_color_palette: true,
        label_htiles: false,
        label_points: false,
        show_seams: false,
        hflip: HFlip::No
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

    let mut counts: HashMap< (HalfTileType, TriangleSide, usize), usize > = HashMap::new();

    for (tri, visibility) in quilt.pt.htiles.iter().zip(quilt.htile_visibility.iter()) {

        if *visibility == Visibility::Full {

            let level = tri.level(quilt.pt.generations.len());

            let key = (tri.ttype, tri.tside, level);

            *counts.entry(key).or_insert(0) += 1;

        }

    }

    for level in 0..quilt.pt.generations.len() {

        for &(ttype, tside) in &MODULE_SHAPES {

            let key = (ttype, tside, level);

            if let Some(count) = counts.get(&key) {

                if level == 0 {
                    println!("need {:} of {:}", 
                             count, tri_type_side_level_string(ttype, tside, level));
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
                                                 spacing));
            }

        }

        drawables.push(Drawable::LineBreak);

    }

    layout_page(&ctx, &portrait_rect, &drawables, spacing)?;

    draw(&quilt.visible_top_level_htiles, &portrait_rect, PageSettings {
        frame: CoordinateFrame::XformedCoords,
        draw_as_finished: false,
        two_color_palette: false,
        label_htiles: true,
        label_points: false,
        show_seams: true,
        hflip: HFlip::Yes
    })?;

    ctx.show_page();

    let spacing = 4.0;

    drawables.clear();
    drawables.push( label_drawable(&ctx, 16.0, "Instructions".to_string(), PAGE_SHORT_EDGE, 8.0) );

    for (batch_idx, batch_instructions) in quilt.instructions.iter().enumerate() {

        let s = format!("Seam batch {:} of {:}:", 
                        batch_idx+1, quilt.instructions.len());

        let mut dvec = vec![];

        dvec.push( label_drawable(&ctx, 12.0, s, PAGE_SHORT_EDGE, 4.0) );

        for (i, s) in batch_instructions.iter().enumerate() {
            
            let ss = "    ".to_string() + s;

            let after = if i + 1 == batch_instructions.len() {
                8.0
            } else {
                0.0
            };
            
            dvec.push( label_drawable(&ctx, 12.0, ss, PAGE_SHORT_EDGE, after) );

        }
        
        drawables.push(vstack(dvec, spacing));

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
