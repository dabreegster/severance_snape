use std::collections::HashMap;

use anyhow::Result;
use geo::{ConvexHull, Coord, Geometry, GeometryCollection, LineString, Point};
use osm_reader::{Element, NodeID, WayID};

use crate::{Graph, Intersection, IntersectionID, Mercator, Road, RoadID, Tags};

struct Way {
    id: WayID,
    node_ids: Vec<NodeID>,
    tags: Tags,
}

impl<RoadData, IntersectionData> Graph<RoadData, IntersectionData> {
    pub fn new<
        KeepRoad: Fn(&Tags) -> bool,
        MakeRoadData: Fn(&Tags) -> RoadData,
        MakeIntersectionData: Fn() -> IntersectionData,
    >(
        input_bytes: &[u8],
        keep_road: KeepRoad,
        make_road_data: MakeRoadData,
        make_intersection_data: MakeIntersectionData,
    ) -> Result<Self> {
        let mut node_mapping = HashMap::new();
        let mut highways = Vec::new();
        osm_reader::parse(input_bytes, |elem| match elem {
            Element::Node { id, lon, lat, .. } => {
                let pt = Coord { x: lon, y: lat };
                node_mapping.insert(id, pt);
            }
            Element::Way { id, node_ids, tags } => {
                let tags: Tags = tags.into();
                if keep_road(&tags) {
                    highways.push(Way { id, node_ids, tags });
                }
            }
            Element::Relation { .. } => {}
        })?;

        let (mut roads, mut intersections) = split_edges(
            &node_mapping,
            highways,
            make_road_data,
            make_intersection_data,
        );

        // TODO expensive
        let mut collection: GeometryCollection = roads
            .iter()
            .map(|r| Geometry::LineString(r.linestring.clone()))
            .chain(
                intersections
                    .iter()
                    .map(|i| Geometry::Point(i.point.clone())),
            )
            .collect::<Vec<_>>()
            .into();
        let mercator = Mercator::from(collection.clone()).unwrap();
        for r in &mut roads {
            mercator.to_mercator_in_place(&mut r.linestring);
        }
        for i in &mut intersections {
            mercator.to_mercator_in_place(&mut i.point);
        }

        mercator.to_mercator_in_place(&mut collection);
        let boundary_polygon = collection.convex_hull();

        Ok(Self {
            roads,
            intersections,
            mercator,
            boundary_polygon,
        })
    }
}

fn split_edges<
    RoadData,
    IntersectionData,
    MakeRoadData: Fn(&Tags) -> RoadData,
    MakeIntersectionData: Fn() -> IntersectionData,
>(
    node_mapping: &HashMap<NodeID, Coord>,
    ways: Vec<Way>,
    make_road_data: MakeRoadData,
    make_intersection_data: MakeIntersectionData,
) -> (Vec<Road<RoadData>>, Vec<Intersection<IntersectionData>>) {
    // Count how many ways reference each node
    let mut node_counter: HashMap<NodeID, usize> = HashMap::new();
    for way in &ways {
        for node in &way.node_ids {
            *node_counter.entry(*node).or_insert(0) += 1;
        }
    }

    // Split each way into edges
    let mut node_to_intersection: HashMap<NodeID, IntersectionID> = HashMap::new();
    let mut intersections = Vec::new();
    let mut roads = Vec::new();
    for way in ways {
        let mut node1 = way.node_ids[0];
        let mut pts = Vec::new();

        let num_nodes = way.node_ids.len();
        for (idx, node) in way.node_ids.into_iter().enumerate() {
            pts.push(node_mapping[&node]);
            // Edges start/end at intersections between two ways. The endpoints of the way also
            // count as intersections.
            let is_endpoint =
                idx == 0 || idx == num_nodes - 1 || *node_counter.get(&node).unwrap() > 1;
            if is_endpoint && pts.len() > 1 {
                let road_id = RoadID(roads.len());

                let mut i_ids = Vec::new();
                for (n, point) in [(node1, pts[0]), (node, *pts.last().unwrap())] {
                    let intersection = if let Some(i) = node_to_intersection.get(&n) {
                        &mut intersections[i.0]
                    } else {
                        let i = IntersectionID(intersections.len());
                        intersections.push(Intersection {
                            id: i,
                            node: n,
                            point: Point(point),
                            roads: Vec::new(),
                            data: make_intersection_data(),
                        });
                        node_to_intersection.insert(n, i);
                        &mut intersections[i.0]
                    };

                    intersection.roads.push(road_id);
                    i_ids.push(intersection.id);
                }

                roads.push(Road {
                    id: road_id,
                    src_i: i_ids[0],
                    dst_i: i_ids[1],
                    way: way.id,
                    node1,
                    node2: node,
                    linestring: LineString::new(std::mem::take(&mut pts)),
                    data: make_road_data(&way.tags),
                });

                // Start the next edge
                node1 = node;
                pts.push(node_mapping[&node]);
            }
        }
    }

    (roads, intersections)
}
