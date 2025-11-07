use std::collections::HashMap;

use bevy::ecs::{
    bundle::Bundle,
    entity::Entity,
    hierarchy::Children,
    system::{Commands, Query},
};

pub fn collect_entities_by<F>(name_map: &HashMap<String, Entity>, mut predicate: F) -> Vec<Entity>
where
    F: FnMut(&str) -> bool,
{
    let mut entries: Vec<_> = name_map
        .iter()
        .filter_map(|(name, &entity)| {
            if predicate(name) {
                Some((name.clone(), entity))
            } else {
                None
            }
        })
        .collect();
    entries.sort_by(|(a, _), (b, _)| a.cmp(b));
    entries.into_iter().map(|(_, entity)| entity).collect()
}

pub fn insert_recursively<F, B>(
    commands: &mut Commands,
    root: Entity,
    query: &Query<&Children>,
    bundle: &F,
) where
    F: Fn() -> B,
    B: Bundle,
{
    commands.entity(root).insert(bundle());
    if let Ok(children) = query.get(root) {
        for child in children {
            insert_recursively(commands, *child, query, bundle);
        }
    }
}
