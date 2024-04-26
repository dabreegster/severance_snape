use std::fmt;

use geo::{LineString, Point, Polygon};

use crate::Mercator;

pub struct Graph<RoadData, IntersectionData> {
    pub roads: Vec<Road<RoadData>>,
    pub intersections: Vec<Intersection<IntersectionData>>,
    // All geometry is stored in world-space
    pub mercator: Mercator,
    pub boundary_polygon: Polygon,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq, PartialOrd, Ord)]
pub struct RoadID(pub usize);
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq, PartialOrd, Ord)]
pub struct IntersectionID(pub usize);

impl fmt::Display for RoadID {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Road #{}", self.0)
    }
}

impl fmt::Display for IntersectionID {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Intersection #{}", self.0)
    }
}

pub struct Road<RoadData> {
    pub id: RoadID,
    pub src_i: IntersectionID,
    pub dst_i: IntersectionID,

    pub way: osm_reader::WayID,
    pub node1: osm_reader::NodeID,
    pub node2: osm_reader::NodeID,

    pub linestring: LineString,

    pub data: RoadData,
}

pub struct Intersection<IntersectionData> {
    pub id: IntersectionID,
    pub roads: Vec<RoadID>,

    pub node: osm_reader::NodeID,

    pub point: Point,

    pub data: IntersectionData,
}
