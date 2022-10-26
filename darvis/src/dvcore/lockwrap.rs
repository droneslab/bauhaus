/// *** Structs to wrap map to manage read/write access *** ///
// ReadOnlyWrapper is used by all actors but the map actor.
// Trying to write on read-only will give a compilation error.
// To create a ReadOnlyWrapper, first create a ReadWriteMap and then call
// `read_only()` on it to create a cloned ReadOnlyWrapper that can be passed
// to other actors.
// Currently only used for map objects but is generic so you can use it
// for other objects.
use parking_lot::{
    RwLock,
    MappedRwLockReadGuard,
    MappedRwLockWriteGuard,
    RwLockReadGuard,
    RwLockWriteGuard
};
use std::sync::Arc;

#[derive(Debug, Clone, Default)]
pub struct ReadOnlyWrapper<T> {
    inner: Arc<RwLock<T>>,
}
impl<T> ReadOnlyWrapper<T> {
    pub fn read(&self) -> MappedRwLockReadGuard<T> {
        RwLockReadGuard::map(self.inner.read(), |unlocked| unlocked)
    }
}

// Read-write map that is used ONLY by the map actor.
// When creating this object, writing is on by default.
pub struct ReadWriteWrapper<T> {
    inner: Arc<RwLock<T>>,
}
impl<T> ReadWriteWrapper<T> {
    pub fn new(inner: T) -> Self {
        Self {
            inner: Arc::new(RwLock::new(inner)),
        }
    }

    pub fn write(&self) -> MappedRwLockWriteGuard<T> {
        RwLockWriteGuard::map(self.inner.write(), |unlocked| unlocked)
    }

    pub fn read(&self) -> MappedRwLockReadGuard<T> {
        RwLockReadGuard::map(self.inner.read(), |unlocked| unlocked)
    }

    pub fn read_only(&self) -> ReadOnlyWrapper<T> {
        ReadOnlyWrapper {
            inner: self.inner.clone(),
        }
    }
}
