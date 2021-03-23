mod base;
mod orb;

fn main() {
    let img_paths = ["data/1.png".to_string(), "data/2.png".to_string()];

    orb::orb_extract(img_paths.to_vec());
}
