use anyhow::{bail, Result};
use fast_paths::{FastGraph, InputGraph};
use geo::{EuclideanLength, LineString};
use geojson::{Feature, FeatureCollection};
use rstar::RTree;

use osm_graph::NodeMap;

use crate::{
    CompareRouteRequest, Intersection, IntersectionID, IntersectionLocation, MapModel, Road,
    RoadKind, RoadLike,
};

pub fn build_router(
    intersections: &Vec<Intersection>,
    roads: &Vec<Road>,
) -> (
    RTree<IntersectionLocation>,
    NodeMap<IntersectionID>,
    FastGraph,
) {
    let mut input_graph = InputGraph::new();
    let mut node_map = NodeMap::new();

    for r in roads {
        if r.data.kind == RoadKind::Severance {
            continue;
        }
        let node1 = node_map.get_or_insert(r.src_i);
        let node2 = node_map.get_or_insert(r.dst_i);
        // Use units of cm for comparing edges
        let cost = (100.0 * r.linestring.euclidean_length()).round() as usize;
        input_graph.add_edge(node1, node2, cost);
        input_graph.add_edge(node2, node1, cost);
    }
    input_graph.freeze();
    let ch = fast_paths::prepare(&input_graph);

    let closest_intersection = build_closest_intersection(intersections, &node_map);
    (closest_intersection, node_map, ch)
}

fn build_closest_intersection(
    intersections: &Vec<Intersection>,
    node_map: &NodeMap<IntersectionID>,
) -> RTree<IntersectionLocation> {
    let mut points = Vec::new();
    for i in intersections {
        // If the intersection only involves severances, exclude
        if let Some(node) = node_map.get(i.id) {
            points.push(IntersectionLocation::new(i.point.into(), node));
        }
    }
    RTree::bulk_load(points)
}

// Also returns the line of the snapped request (in WGS84)
pub fn do_route(
    map: &mut MapModel,
    req: CompareRouteRequest,
) -> Result<(Feature, FeatureCollection)> {
    let start = map
        .closest_intersection
        .nearest_neighbor(&[req.x1, req.y1])
        .unwrap()
        .data;
    let end = map
        .closest_intersection
        .nearest_neighbor(&[req.x2, req.y2])
        .unwrap()
        .data;
    if start == end {
        bail!("start = end");
    }

    if let Some(path) = map.path_calc.calc_path(&map.ch, start, end) {
        let direct_line = LineString::new(vec![
            map.graph.intersections[map.node_map.translate_id(start).0]
                .point
                .into(),
            map.graph.intersections[map.node_map.translate_id(end).0]
                .point
                .into(),
        ]);
        let direct_feature = Feature::from(geojson::Geometry::from(
            &map.graph.mercator.to_wgs84(&direct_line),
        ));

        let mut features = Vec::new();
        let mut route_length = 0.0;
        for pair in path.get_nodes().windows(2) {
            let i1 = map.node_map.translate_id(pair[0]);
            let i2 = map.node_map.translate_id(pair[1]);
            let road = map.graph.find_edge(i1, i2).unwrap();
            features.push(road.to_gj(&map.graph.mercator));
            route_length += road.linestring.euclidean_length();
        }
        let direct_length = direct_line.euclidean_length();
        return Ok((
            direct_feature,
            FeatureCollection {
                features,
                bbox: None,
                foreign_members: Some(
                    serde_json::json!({
                        "direct_length": direct_length,
                        "route_length": route_length,
                    })
                    .as_object()
                    .unwrap()
                    .clone(),
                ),
            },
        ));
    }
    bail!("No path");
}
