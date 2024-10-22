/// *** Structs to wrap map to manage read/write access *** ///
// ReadOnlyWrapper is used by all actors but the map actor.
// Trying to write on read-only will give a compilation error.
// To create a ReadOnlyWrapper, first create a ReadWriteMap and then call
// `read_only()` on it to create a cloned ReadOnlyWrapper that can be passed
// to other actors.
// Currently only used for map objects but is generic so you can use it
// for other objects.
// use std::sync::{
//     RwLock,
//     RwLockReadGuard, //MappedRwLockReadGuard,
//     RwLockWriteGuard, //MappedRwLockWriteGuard,
// };
use std::{backtrace::Backtrace, sync::Arc, time::Instant};
use log::{debug, warn};
use parking_lot::{
    RwLock,
    MappedRwLockReadGuard,
    MappedRwLockWriteGuard,
    RwLockReadGuard,
    RwLockWriteGuard
};

use super::map::Map;

pub type UnlockedToRead<'a, Map> = MappedRwLockReadGuard<'a, Map>;

#[derive(Debug, Clone)]
pub struct ReadOnlyMap<T> {
    inner: Arc<RwLock<T>>,
    version: u64,
}
impl<T> ReadOnlyMap<T> {
    pub fn read(&self) -> MappedRwLockReadGuard<T> {
        let now = Instant::now();
        let guard = RwLockReadGuard::map(self.inner.read(), |unlocked| unlocked);
        let elapsed = now.elapsed().as_millis();
        if elapsed > 5 {
            debug!("LOCKS...Read acquire: {} ms", now.elapsed().as_millis());
        }

        guard
    }
}

// Read-write map that is used ONLY by the map actor.
// When creating this object, writing is on by default.
#[derive(Debug, Clone)]
pub struct ReadWriteMap {
    inner: Arc<RwLock<Map>>,
    version: u64,
}
impl ReadWriteMap {
    pub fn new(inner: Map) -> Self {
        Self {
            inner: Arc::new(RwLock::new(inner)),
            version: 0,
        }
    }

    pub fn write(&self) -> Result<MappedRwLockWriteGuard<Map>, Box<dyn std::error::Error>> {
        // If the current thread's version of the map is different (older) than the map's version,
        // then it means the map has been reset by another thread. Send this info back to the caller
        // so they can figure out what to do with it (they probably want to abort the loop)
        let inner = self.inner.write();
        if inner.version == self.version {
            Ok(RwLockWriteGuard::map(inner, |unlocked| unlocked))
        } else { 
            warn!("Map version mismatch: I have {}, map has {}. Backtrace: {:?}", self.version, inner.version, Backtrace::capture());
            Err("Map version mismatch".into())
        }
    }

    pub fn read(&self) -> Result<MappedRwLockReadGuard<Map>, Box<dyn std::error::Error>> {
        // If the current thread's version of the map is different (older) than the map's version,
        // then it means the map has been reset by another thread. Send this info back to the caller
        // so they can figure out what to do with it (they probably want to abort the loop)
        let inner = self.inner.read();
        if inner.version == self.version {
            Ok(RwLockReadGuard::map(inner , |unlocked| unlocked))
        } else { 
            warn!("Map version mismatch: I have {}, map has {}. Backtrace: {:?}", self.version, inner.version, Backtrace::capture());
            Err("Map version mismatch".into())
        }
    }

    pub fn match_map_version(&mut self) {
        self.version = self.inner.read().version;
    }

    pub fn get_version(&self) -> u64 {
        self.version
    }

    pub fn create_read_only(&self) -> ReadOnlyMap<Map> {
        ReadOnlyMap {
            inner: self.inner.clone(),
            version: self.version,
        }
    }
}