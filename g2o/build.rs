fn main() {
    use cmake::Config;

    let _ = cxx_build::bridge("src/lib.rs");

    // Pranay: Can use the following flag to set cmake::Config .cxxflag to include directory path for resolving the lib.rs .h file search path issue
    // But Fix the CMakeList.txt instead to include the path itself
    let manifest_dir = concat!("-I",concat!(env!("CARGO_MANIFEST_DIR"),"/g2o/g2o/rust/"));

    let _dst = Config::new("g2o")
                        .cxxflag(manifest_dir)
                        .build_target("g2o")
                        .profile("3") // 0 = debug, 1, 2, or 3 = Release
                        .build();

    println!("cargo:rustc-link-search=native=g2o/lib");
    println!("cargo:rustc-link-lib=static=g2o");
}
