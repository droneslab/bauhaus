use std::sync::Arc;
use axiom::prelude::*;
use parking_lot::{
    RwLock,
    MappedRwLockReadGuard,
    MappedRwLockWriteGuard,
    RwLockReadGuard,
    RwLockWriteGuard
};
use crate::map::map::Map;

pub struct MapActor {
    map: ReadWriteMap
}

impl MapActor {
    pub fn new(map: ReadWriteMap) -> Self {
        MapActor { map }
    }

    pub async fn handle(self, _context: Context, _message: Message) -> ActorResult<Self> {
        Ok(Status::done(self))
    }
}


/// *** STRUCTS THAT WRAP MAP AND MANAGE READ/WRITE ACCESS *** ///
// Read-only map that is used by all actors but the map actor.
// Trying to write on this will give a compilation error.
// To create a ReadOnlyMap, first create a ReadWriteMap and then call
// `read_only()` on it to create a cloned ReadOnlyMap that can be passed
// to other actors.
#[derive(Debug, Clone)]
pub struct ReadOnlyMap {
    map: Arc<RwLock<Map>>,
}
impl ReadOnlyMap {
    pub fn read(&self) -> MappedRwLockReadGuard<Map> {
        RwLockReadGuard::map(self.map.read(), |unlocked| unlocked)
    }
}

// Read-write map that is used ONLY by the map actor.
// When creating this object, writing is on by default.
pub struct ReadWriteMap {
    map: Arc<RwLock<Map>>,
}
impl ReadWriteMap {
    pub fn new() -> Self {
        Self {
            map: Arc::new(RwLock::new(Map::new())),
        }
    }

    pub fn write(&self) -> MappedRwLockWriteGuard<Map> {
        RwLockWriteGuard::map(self.map.write(), |unlocked| unlocked)
    }

    pub fn read(&self) -> MappedRwLockReadGuard<Map> {
        RwLockReadGuard::map(self.map.read(), |unlocked| unlocked)
    }

    pub fn read_only(&self) -> ReadOnlyMap {
        ReadOnlyMap {
            map: self.map.clone(),
        }
    }
}
