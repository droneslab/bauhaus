fn main() {
    use cmake::Config;

    cxx_build::bridge("src/lib.rs");

    let manifest_dir_sophus = concat!(" -I",concat!(env!("CARGO_MANIFEST_DIR"),"/orb_slam3/Sophus/"));
    let manifest_dir_bow = concat!(" -I",concat!(env!("CARGO_MANIFEST_DIR"),"/orb_slam3/DBoW2/DBoW2/"));

    let manifest_dir_root= concat!(" -I",concat!(env!("CARGO_MANIFEST_DIR"),"/   -I /usr/local/include/"));
    
    let manifest_dir_1 = manifest_dir_sophus.to_string() +  manifest_dir_bow;

    let manifest_dir= manifest_dir_1 + manifest_dir_root;
    let dst = Config::new("orb_slam3")
    .cxxflag(manifest_dir)
                    .build_target("orb_slam3")
                    .build();


    println!("cargo:rustc-link-search=native=orb_slam3/lib");
    println!("cargo:rustc-link-lib=static=orb_slam3");
    //println!("cargo:rustc-link-search=native=orb_slam3/DBoW2/lib");
    //println!("cargo:rustc-link-lib=dylib=DBoW2");
  
}
