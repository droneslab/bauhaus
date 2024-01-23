fn main() {
    use cmake::Config;

    let _ = cxx_build::bridge("src/lib.rs");

    let sophus = concat!(" -I",concat!(env!("CARGO_MANIFEST_DIR"),"/orb_slam3/Sophus/"));
    let bow = concat!(" -I",concat!(env!("CARGO_MANIFEST_DIR"),"/orb_slam3/DBoW2/DBoW2/"));
    let manifest_dir_root= concat!(" -I",concat!(env!("CARGO_MANIFEST_DIR"),"/   -I /usr/local/include/"));
    let cxxflags = sophus.to_string() +  bow + manifest_dir_root;

    let _dst = Config::new("orb_slam3")
                    .cxxflag(cxxflags)
                    .cxxflag("-DCMAKE_CXX_COMPILER=/usr/bin/clang-cpp-14")
                    // .cxxflag("-fsanitize=address")
                    // .cflag("-fsanitize=address")
                    .build_target("orb_slam3")
                    .profile("RelWithDebInfo")
                    .build();


    println!("cargo:rustc-link-search=native=orb_slam3/lib");
    println!("cargo:rustc-link-lib=static=orb_slam3");
    // println!("cargo:rustc-link-arg=-fsanitize=address");
}
