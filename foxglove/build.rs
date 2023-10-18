// fn main() {
//     cxx_build::bridge("src/lib.rs")
//         .file("src/JSONWriter.cc")
//         .flag_if_supported("-std=c++14")
//         .compile("foxglove");

//     println!("cargo:rerun-if-changed=src/lib.rs");
//     println!("cargo:rerun-if-changed=src/JSONWriter.cc");
//     println!("cargo:rerun-if-changed=include/JSONWriter.h");
// }


fn main() {
    use cmake::Config;

    let _ = cxx_build::bridge("src/lib.rs");

    let manifest_dir= concat!(" -I",concat!(env!("CARGO_MANIFEST_DIR"),"/   -I /usr/local/include/"));


    let _dst = Config::new("foxglove")
                    .cxxflag(manifest_dir)
                    // .cxxflag("-fsanitize=address")
                    // .cflag("-fsanitize=address")
                    .build_target("foxglove")
                    .profile("3") // 0 = debug, 1, 2, or 3 = Release
                    .build();


    println!("cargo:rustc-link-search=native=foxglove/lib");
    println!("cargo:rustc-link-lib=static=foxglove");
    // println!("cargo:rustc-link-arg=-fsanitize=address");
}
