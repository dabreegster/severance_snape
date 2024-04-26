use std::collections::HashSet;

use geo::{Coord, Densify, Line, LineString};
use geojson::FeatureCollection;
use rstar::{primitives::GeomWithData, RTree};

use crate::{CompareRouteRequest, IntersectionID, MapModel, RoadKind};

// Walk along severances. Every X meters, try to cross from one side to the other.
//
// We could focus where footways connect to severances, but that's probably a crossing. Ideally we
// want to find footpaths parallel(ish) to severances. If we had some kind of generalized edge
// bundling...
pub fn along_severances(map: &mut MapModel) -> FeatureCollection {
    let mut requests = Vec::new();
    for r in &map.graph.roads {
        if r.data.kind != RoadKind::Severance {
            continue;
        }
        for line in make_perpendicular_offsets(&r.linestring, 25.0, 15.0) {
            requests.push(line.into());
        }
    }
    calculate(map, requests)
}

// For every intersection involving a footway, look for any other nearby intersection and see how
// hard it is to walk there.
pub fn nearby_footway_intersections(map: &mut MapModel, dist_meters: f64) -> FeatureCollection {
    // Look for intersections we want to connect
    let mut footway_intersections = HashSet::new();
    for r in &map.graph.roads {
        if r.data.kind == RoadKind::Footway {
            footway_intersections.insert(r.src_i);
            footway_intersections.insert(r.dst_i);
        }
    }

    // Make an rtree
    let mut points: Vec<GeomWithData<[f64; 2], IntersectionID>> = Vec::new();
    for i in &footway_intersections {
        points.push(GeomWithData::new(
            map.graph.intersections[i.0].point.into(),
            *i,
        ));
    }
    let rtree = RTree::bulk_load(points);

    // For every intersection, try to go to every nearby intersection
    let mut requests = Vec::new();
    for i1 in &footway_intersections {
        let i1_pt = map.graph.intersections[i1.0].point;
        for i2 in rtree.locate_within_distance(i1_pt.into(), dist_meters) {
            // TODO Skip trivial things connected by a road
            let i2_pt = map.graph.intersections[i2.data.0].point;
            requests.push(CompareRouteRequest {
                x1: i1_pt.x(),
                y1: i1_pt.y(),
                x2: i2_pt.x(),
                y2: i2_pt.y(),
            });
        }
    }
    calculate(map, requests)
}

fn calculate(map: &mut MapModel, requests: Vec<CompareRouteRequest>) -> FeatureCollection {
    let mut samples = Vec::new();
    let mut max_score = 0.0_f64;
    for req in requests {
        if let Ok((mut f, fc)) = crate::route::do_route(map, req) {
            let direct = fc
                .foreign_members
                .as_ref()
                .unwrap()
                .get("direct_length")
                .unwrap()
                .as_f64()
                .unwrap();
            let route = fc
                .foreign_members
                .unwrap()
                .get("route_length")
                .unwrap()
                .as_f64()
                .unwrap();
            let score = route / direct;
            max_score = max_score.max(score);
            f.set_property("score", score);
            samples.push(f);
        }
    }
    info!("Max score is {max_score}");
    FeatureCollection {
        features: samples,
        bbox: None,
        foreign_members: None,
    }
}

// TODO canvas_geometry needs this too
fn make_perpendicular_offsets(
    linestring: &LineString,
    walk_every_m: f64,
    project_away_m: f64,
) -> Vec<Line> {
    let mut output = Vec::new();
    // Using lines instead of coords so we can get the angle -- but is this hard to reason about?
    // angle_at_point instead?
    for orig_line in linestring.densify(walk_every_m).lines() {
        // TODO For the last line, use the last point too
        let angle_degs = (orig_line.end.y - orig_line.start.y)
            .atan2(orig_line.end.x - orig_line.start.x)
            .to_degrees();
        let projected_left = project_away(orig_line.start, angle_degs - 90.0, project_away_m);
        let projected_right = project_away(orig_line.start, angle_degs + 90.0, project_away_m);
        output.push(Line::new(projected_left, projected_right));
    }
    output
}

fn project_away(pt: Coord, angle_degs: f64, dist_away_m: f64) -> Coord {
    let (sin, cos) = angle_degs.to_radians().sin_cos();
    Coord {
        x: pt.x + dist_away_m * cos,
        y: pt.y + dist_away_m * sin,
    }
}
