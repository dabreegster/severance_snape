#[macro_use]
extern crate log;

use std::sync::Once;

use osm_graph::{Mercator, NodeMap, Tags};

use fast_paths::{FastGraph, PathCalculator};
use geo::{Coord, Line};
use geojson::{Feature, GeoJson, Geometry};
use osm_graph::{Graph, IntersectionID, RoadID};
use rstar::{primitives::GeomWithData, RTree};
use serde::Deserialize;
use wasm_bindgen::prelude::*;

mod heatmap;
mod isochrone;
mod route;
mod scrape;

static START: Once = Once::new();

#[wasm_bindgen]
pub struct MapModel {
    graph: Graph<RoadData, ()>,
    // Only snaps to walkable roads
    closest_intersection: RTree<IntersectionLocation>,
    node_map: NodeMap<IntersectionID>,
    ch: FastGraph,
    path_calc: PathCalculator,
}

pub struct RoadData {
    tags: Tags,
    kind: RoadKind,
}

#[derive(Debug, PartialEq)]
pub enum RoadKind {
    Footway,
    Indoors,
    BridgeOrTunnel,
    WithTraffic,
    Crossing,
    Severance,
    // TODO other types of road?
}

pub type Road = osm_graph::Road<RoadData>;
pub type Intersection = osm_graph::Intersection<()>;

// fast_paths ID representing the OSM node ID as the data
// TODO No, I think this is a fast paths NodeID?
type IntersectionLocation = GeomWithData<[f64; 2], usize>;

#[wasm_bindgen]
impl MapModel {
    /// Call with bytes of an osm.pbf or osm.xml string
    #[wasm_bindgen(constructor)]
    pub fn new(
        input_bytes: &[u8],
        import_streets_without_sidewalk_tagging: bool,
    ) -> Result<MapModel, JsValue> {
        // Panics shouldn't happen, but if they do, console.log them.
        console_error_panic_hook::set_once();
        START.call_once(|| {
            console_log::init_with_level(log::Level::Info).unwrap();
        });

        let graph = Graph::new(
            input_bytes,
            |tags| scrape::classify(tags, import_streets_without_sidewalk_tagging).is_some(),
            |tags| RoadData {
                tags: tags.clone(),
                kind: scrape::classify(tags, import_streets_without_sidewalk_tagging).unwrap(),
            },
            || (),
        )
        .map_err(err_to_js)?;

        let (closest_intersection, node_map, ch) =
            crate::route::build_router(&graph.intersections, &graph.roads);
        let path_calc = fast_paths::create_calculator(&ch);

        Ok(Self {
            graph,
            closest_intersection,
            node_map,
            ch,
            path_calc,
        })
    }

    /// Returns a GeoJSON string. Just shows the full ped network
    #[wasm_bindgen()]
    pub fn render(&self) -> Result<String, JsValue> {
        let mut features = Vec::new();

        for r in &self.graph.roads {
            features.push(r.to_gj(&self.graph.mercator));
        }

        let gj = GeoJson::from(features);
        let out = serde_json::to_string(&gj).map_err(err_to_js)?;
        Ok(out)
    }

    #[wasm_bindgen(js_name = compareRoute)]
    pub fn compare_route(&mut self, input: JsValue) -> Result<String, JsValue> {
        let req: CompareRouteRequest = serde_wasm_bindgen::from_value(input)?;
        let pt1 = self.graph.mercator.pt_to_mercator(Coord {
            x: req.x1,
            y: req.y1,
        });
        let pt2 = self.graph.mercator.pt_to_mercator(Coord {
            x: req.x2,
            y: req.y2,
        });
        let (_, gj) = route::do_route(
            self,
            CompareRouteRequest {
                x1: pt1.x,
                y1: pt1.y,
                x2: pt2.x,
                y2: pt2.y,
            },
        )
        .map_err(err_to_js)?;
        let out = serde_json::to_string(&gj).map_err(err_to_js)?;
        Ok(out)
    }

    #[wasm_bindgen(js_name = makeHeatmap)]
    pub fn make_heatmap(&mut self) -> Result<String, JsValue> {
        let samples = heatmap::along_severances(self);
        // TODO unit here is weird or wrong or something
        //let samples = heatmap::nearby_footway_intersections(self, 500.0);
        let out = serde_json::to_string(&samples).map_err(err_to_js)?;
        Ok(out)
    }

    /// Return a polygon covering the world, minus a hole for the boundary, in WGS84
    #[wasm_bindgen(js_name = getInvertedBoundary)]
    pub fn get_inverted_boundary(&self) -> Result<String, JsValue> {
        let f = Feature::from(Geometry::from(&self.graph.get_inverted_boundary()));
        let out = serde_json::to_string(&f).map_err(err_to_js)?;
        Ok(out)
    }

    #[wasm_bindgen(js_name = getBounds)]
    pub fn get_bounds(&self) -> Vec<f64> {
        let b = &self.graph.mercator.wgs84_bounds;
        vec![b.min().x, b.min().y, b.max().x, b.max().y]
    }

    #[wasm_bindgen(js_name = isochrone)]
    pub fn isochrone(&self, input: JsValue) -> Result<String, JsValue> {
        let req: IsochroneRequest = serde_wasm_bindgen::from_value(input)?;
        let start = self
            .graph
            .mercator
            .pt_to_mercator(Coord { x: req.x, y: req.y });
        isochrone::calculate(&self, start).map_err(err_to_js)
    }
}

trait RoadLike {
    fn to_gj(&self, mercator: &Mercator) -> Feature;
}

impl RoadLike for Road {
    fn to_gj(&self, mercator: &Mercator) -> Feature {
        let mut f = Feature::from(Geometry::from(&mercator.to_wgs84(&self.linestring)));
        f.set_property("id", self.id.0);
        f.set_property("kind", format!("{:?}", self.data.kind));
        f.set_property("way", self.way.to_string());
        f.set_property("node1", self.node1.to_string());
        f.set_property("node2", self.node2.to_string());
        for (k, v) in &self.data.tags.0 {
            f.set_property(k, v.to_string());
        }
        f
    }
}

// Mercator worldspace internally, but not when it comes in from the app
// TODO only use this on the boundary
#[derive(Deserialize)]
pub struct CompareRouteRequest {
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
}

impl From<Line> for CompareRouteRequest {
    fn from(line: Line) -> Self {
        Self {
            x1: line.start.x,
            y1: line.start.y,
            x2: line.end.x,
            y2: line.end.y,
        }
    }
}

#[derive(Deserialize)]
pub struct IsochroneRequest {
    x: f64,
    y: f64,
}

fn err_to_js<E: std::fmt::Display>(err: E) -> JsValue {
    JsValue::from_str(&err.to_string())
}
