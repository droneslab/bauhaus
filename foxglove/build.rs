use std::{io::Result, path::PathBuf, env};
fn main() -> Result<()> {

    let mut config = prost_build::Config::new();
    config.file_descriptor_set_path(
        PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR environment variable not set"))
            .join("file_descriptor_set.bin"));

    config.compile_protos(&[
        "foxglove/SceneUpdate.proto", 
        "foxglove/SceneEntityDeletion.proto",
        "foxglove/SpherePrimitive.proto",
        "foxglove/FrameTransform.proto",
        "foxglove/RawImage.proto",
        "foxglove/MapInfo.proto"
        ], &[""])?;
    Ok(())
}
