use log::{trace, debug};
use parking_lot::{
    RwLock,
    MappedRwLockReadGuard,
    MappedRwLockWriteGuard,
    RwLockReadGuard,
    RwLockWriteGuard
};
use std::backtrace::Backtrace;

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
//     // RwLockReadGuard,
//     // RwLockWriteGuard
// };
use std::{sync::Arc, time::Instant};

#[derive(Debug, Clone)]
pub struct ReadOnlyMap<T> {
    inner: Arc<RwLock<T>>,
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
pub struct ReadWriteMap<T> {
    inner: Arc<RwLock<T>>,
}
impl<T> ReadWriteMap<T> {
    pub fn new(inner: T) -> Self {
        Self {
            inner: Arc::new(RwLock::new(inner)),
        }
    }

    pub fn write(&self) -> MappedRwLockWriteGuard<T> {
        let now = Instant::now();
        // let write = self.inner.write().unwrap();
        let write = RwLockWriteGuard::map(self.inner.write(), |unlocked| unlocked);

        let elapsed = now.elapsed().as_millis();
        if elapsed > 5 {
            debug!("LOCKS...Write acquire: {} ms", elapsed);
            // println!("Custom backtrace: {}", Backtrace::capture());

        }
        write
    }

    pub fn read(&self) -> MappedRwLockReadGuard<T> {
        RwLockReadGuard::map(self.inner.read(), |unlocked| unlocked)

    }

    pub fn create_read_only(&self) -> ReadOnlyMap<T> {
        ReadOnlyMap {
            inner: self.inner.clone(),
        }
    }
}
