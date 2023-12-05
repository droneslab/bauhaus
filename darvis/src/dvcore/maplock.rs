use log::{trace, debug};
/// *** Structs to wrap map to manage read/write access *** ///
// ReadOnlyWrapper is used by all actors but the map actor.
// Trying to write on read-only will give a compilation error.
// To create a ReadOnlyWrapper, first create a ReadWriteMap and then call
// `read_only()` on it to create a cloned ReadOnlyWrapper that can be passed
// to other actors.
// Currently only used for map objects but is generic so you can use it
// for other objects.
use std::sync::{
    RwLock,
    RwLockReadGuard, //MappedRwLockReadGuard,
    RwLockWriteGuard, //MappedRwLockWriteGuard,
    // RwLockReadGuard,
    // RwLockWriteGuard
};
use std::{sync::Arc, time::Instant};

#[derive(Debug, Clone, Default)]
pub struct ReadOnlyMap<T> {
    inner: Arc<RwLock<T>>,
}
impl<T> ReadOnlyMap<T> {
    pub fn read(&self) -> RwLockReadGuard<T> {
        let now = Instant::now();
        let guard = self.inner.read().unwrap();
        let elapsed = now.elapsed().as_millis();
        if elapsed > 5 {
            debug!("LOCKS...Read acquire: {} ms", now.elapsed().as_millis());
        }
        guard
    }
}

// Read-write map that is used ONLY by the map actor.
// When creating this object, writing is on by default.
pub struct ReadWriteMap<T> {
    inner: Arc<RwLock<T>>,
}
impl<T> ReadWriteMap<T> {
    pub fn new(inner: T) -> Self {
        Self {
            inner: Arc::new(RwLock::new(inner)),
        }
    }

    pub fn write(&self) -> RwLockWriteGuard<T> {
        let now = Instant::now();
        let write = self.inner.write().unwrap();
        let elapsed = now.elapsed().as_millis();
        if elapsed > 5 {
            debug!("LOCKS...Write acquire: {} ms", elapsed);
        }
        write
    }

    pub fn read(&self) -> RwLockReadGuard<T> {
        debug!("retrieving read lock in map");
        self.inner.read().unwrap()
    }

    pub fn create_read_only(&self) -> ReadOnlyMap<T> {
        ReadOnlyMap {
            inner: self.inner.clone(),
        }
    }
}
