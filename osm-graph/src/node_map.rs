//! Some helpers for working with fast_paths, adapted from A/B Street
// TODO This is a version with no serialization

use std::collections::BTreeMap;
use std::fmt::Debug;

use fast_paths::NodeId;

/// A bidirectional mapping between fast_paths NodeId and some custom ID type.
#[derive(Clone)]
pub struct NodeMap<T: Copy + Ord + Debug> {
    node_to_id: BTreeMap<T, NodeId>,
    id_to_node: Vec<T>,
}

impl<T: Copy + Ord + Debug> NodeMap<T> {
    pub fn new() -> NodeMap<T> {
        NodeMap {
            node_to_id: BTreeMap::new(),
            id_to_node: Vec::new(),
        }
    }

    pub fn get_or_insert(&mut self, node: T) -> NodeId {
        if let Some(id) = self.node_to_id.get(&node) {
            return *id;
        }
        let id = self.id_to_node.len();
        self.node_to_id.insert(node, id);
        self.id_to_node.push(node);
        id
    }

    pub fn get(&self, node: T) -> Option<NodeId> {
        self.node_to_id.get(&node).cloned()
    }

    pub fn translate_id(&self, id: usize) -> T {
        self.id_to_node[id]
    }
}
