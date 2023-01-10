# g2o-bindings

## Using

To compile:
``cargo build``

To run examples:
``cargo run --example pose_optimization``

## Making changes
- `g2o/` has the g2o code, including:
    - `g2o/CMakeLists.txt` -- used to compile the g2o library. You can compile g2o on its own with `mkdir build; cd build; cmake ..; make`. The rust crate uses the CMakeLists file to build the library as part of the rust crate compilation and bindings generation.
    - `g2o/rust/rust_helper.cc` and `g2o/rust/rust_helper.h` -- declare the C++ targets that Rust should have bindings to. If making a change here, need to also change the corresponding functions in `src/lib.rs`.
- `src/lib.rs` -- the Rust code that declares the C++ targets for which there are bindings. Should match `g2o/rust/rust_helper.cc` and `g2o/rust/rust_helper.h`.
- `build.rs` -- the compilation instructions to compile g2o, generate bindings, and link. Used automatically by cargo when you run `cargo build`.
- `examples/` has examples for how to use the g2orust crate.