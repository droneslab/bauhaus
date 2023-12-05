# darvis
Distributed, modulAR Visual SLAM

## Installation
```bash
# Install system dependencies 
sudo apt-get update
sudo apt-get install wget git cmake vim clang libclang-dev pkg-config libc6-dbg gdb valgrind libgtk2.0-dev libboost-all-dev

# Install Rust
wget https://raw.githubusercontent.com/rust-lang/rustup/master/rustup-init.sh
chmod +x rustup-init.sh
./rustup-init.sh -y

# Create directories for darvis project, clone repo
mkdir ~/darvis-home
git clone git@github.com:droneslab/darvis.git
mkdir ~/darvis-home/depends

# Upgrade version of cmake (needed for C++ bindings)
cd ~/darvis-home/depends
sudo apt remove --purge cmake && hash -r
sudo apt install build-essential libssl-dev
sudo wget https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2.tar.gz
tar -zxvf cmake-3.20.2.tar.gz
cd cmake-3.20.2 && ./bootstrap && make && make install

# Install eigen
# This installs 3.3: sudo apt-get install libeigen3-dev 
cd ~/darvis-home/depends/
wget https://gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.tar.gz
tar –xvzf eigen-3.2.10.tar.gz
cd eigen-3.2.10 && mkdir build && cd build
cmake ..
make
sudo make install
export EIGEN3_INCLUDE_DIR=/usr/local/include

# Install OpenCV
cd ~/darvis-home/depends
git clone https://github.com/opencv/opencv.git
cd opencv && git checkout tags/4.5.4 && cd ..
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib && git checkout tags/4.5.4 && cd .. 
cd opencv && mkdir build && cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DBUILD_opencv_xfeatures2d=ON -DOPENCV_ENABLE_NONFREE=ON -DWITH_GTK=ON -DWITH_EIGEN=OFF ..
make -j4 install

# Install Pangolin
cd ~/darvis-home/depends
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
./scripts/install_prerequisites.sh recommended
cmake -B build

# Build darvis
cd ~/darvis-home/darvis/darvis/
cargo build --release
```

To instead install with docker...
1. [Install docker](https://docs.docker.com/get-docker/)
2. (Macs only) [Download Xquartz](https://www.xquartz.org/) 
3. Create darvis container (linux):
    ```
        cd darvis
        ./docker_run.sh [pathToDatasetDirectory]
    ```
    Create darvis container (mac):
    ```
        cd darvis
        ./docker_run_mac.sh [pathToDatasetDirectory]
    ```
4. Using rust-analyzer in VSCode typically gives errors about not finding opencv. To resolve these errors, you need to open VSCode inside the container:
    1. Download the [Remote - Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) on VSCode
    2. Follow [instructions for quick start](https://code.visualstudio.com/docs/remote/containers#_quick-start-open-an-existing-folder-in-a-container) to open VSCode inside container. This opens a new VSCode window
    3. For compile-time code checking, in the new VSCode window, install [rust-analyzer extension](https://marketplace.visualstudio.com/items?itemName=matklad.rust-analyzer) 

### Common Problems
- **Segfaults inside orbslam-bindings** 
    - If you have two versions of OpenCV (for example, one from apt that gets downloaded when you install ROS and the one you installed in the instructions above) then you can get segfaults inside the C++ OpenCV code in `orbslam-bindings`. This occurs because C++ finds one OpenCV version (usually the ROS version) and Rust finds a different one (usually the system-installed one). The easiest workaround is just to uninstall ROS and the apt version of OpenCV. It should be possible to inform the rust OpenCV package to look for the right version with [environment variables like this](https://github.com/twistedfall/opencv-rust#environment-variables) but I have not tested it.

---
## Running
### Options for running/building
- RELEASE mode (slow building, fast execution). This is probably what you want to use
    ```bash
    cargo build --release
    cargo run --release [DATASET] config.yaml
    ```
- DEBUG mode (fast building, slow execution)
    ```bash
    cargo build
    cargo run [DATASET] config.yaml
    ```

- Debugging
    - Use GDB
        ```bash
        cargo build    # Either debug build or release build works
        rust-gdb --args target/debug/bindarvis [DATASET] config.yaml
        ```
        You can also use regular gdb instead of rust-gdb, but [it doesn't work in all cases](https://users.rust-lang.org/t/printing-single-vector-elements-in-gdb/16890/4).
    - Check memory errors (useful for verifying C++ code in ffi bindings)
        - Run with address sanitizer:
            ```bash
            RUSTFLAGS="-Z sanitizer=address" cargo run --target x86_64-unknown-linux-gnu [DATASET] config.yaml
            ```
        - Run with valgrind and save output to `log.txt`:
            ```bash
            cargo build # Either debug or release build works
            valgrind target/debug/bindarvis  [DATASET] config.yaml > log.txt 2>&1
            ```
- Performance and profiling
    - Print timing of some functions (functions using ``#[time()]`` proc macro and calls to ``timer!`` macro)
        ```bash
        RUST_BACKTRACE=1 RUST_LOG=debug cargo run --release ~/datasets/kitti_00_0/ config.yaml
        ```
    - Generating flamegraphs (requires some setup, [see here](https://www.justanotherdot.com/posts/profiling-with-perf-and-dhat-on-rust-code-in-linux.html)):
        ```bash
            cargo flamegraph -o flamegraph.svg --root --release --ignore-status  -- ~/datasets/kitti_00_0/ config.yaml
        ```

- Misc:
    - Compile C++ bindings with clang instead of g++, set these environment variables:
        ```bash
        export CXX=/usr/bin/clang++
        export CC=/usr/bin/clang

### Using the Visualizer
1. Download the [foxglove application](https://foxglove.dev/). This can be on any device (does not need to be the test device).
2. Open foxglove, click on `layout` in the top right corner, then `import from file`. Load the file in `foxglove/foxglovelayout.json`.
3. Make sure the `visualizer` actor in `config.yaml` is not commented out.
4. Run the system. After the program is done, it will write the file `darvis/results/out.mcap`. 
5. In foxglove, click `file > open local file` and load `darvis/results/out.mcap`.

Currently there is no way to stream the visualization in real-time. To do that, you would need to use the [foxglove websocket](https://foxglove.dev/docs/studio/connection/custom#live-connection) but this requires compiling a custom server in C++, Python, or TypeScript. You should be able to do it with C++ with ffi bindings, but we have not implemented this (yet?).

For longer datasets, the mcap file size can get unreasonably large because it saves all the images. You can turn this off by setting the `image_draw_type` setting in the visualizer actor to `none`. The other valid options are `plain` (unmodified images), `features` (detected features), and `featuresandmatches` (current and previous image with feature matches highlighted).

---
## Creating new modules

All the steps below reference the module ``Visualizer`` as an example.

1.  **Add module in config**
    - In the ``config.yaml`` file, add the settings for your new module under ``modules``
    - Visualizer example:
        ```yaml
        -
        name: VISUALIZER
        file: vis
        actor_message: VisMsg
        actor_function: vis
        address: localhost
        port: !!str 7779
        multithreaded: false
        threads: 1
        possible_paths:
            -
            from: FeatureDetection
            to: KFDecision
            input: feature vectors
            output: frame, feature points
        ```
2. **Create a file for the new module**
    - Create a new file in ``src/modules/`` to holds your module code. The filename should match that which you declared in the config file. Unless otherwise specific, you will be using this file for all the following steps.
    - Visualizer example: ``src/modules/vis.rs``.
3. **Create the actor**
    - Implement the constructor for the actor and the trait ``Function``. This trait tells axiom which function to run when the actor processes an incoming message.
    - Visualizer example (from ``vis.rs``):
        ```Rust
        /// Constructor
        impl DarvisVis {
            pub fn new(id: String) -> DarvisVis {
                DarvisVis {
                    traj_img: Mat::new_rows_cols_with_default(376, 500, core::CV_8UC3, core::Scalar::all(0.0)).unwrap(),
                    cam_img: Mat::default(),
                    traj_pos: DVVector3::zeros(),
                    traj_rot: DVMatrix3::zeros(),
                    id: id
                }
            }
        }

        /// Function trait
        impl Function for DarvisVis {
            fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
            {
                self.visualize(_context, message).unwrap();
                Ok(Status::done(()))
            }
        }

        ```
4. **Create the message that the actor processes**
    - Create the struct and constructor for the message. Do this in ``src/modules/yourmodulename.rs`` or ``src/modules/messages/newmessagename.rs``.
    - Visualizer example:
        ```Rust
        /// Struct
        #[derive(Debug, Serialize, Deserialize)]
        pub struct VisMsg {
            new_pose: Pose,
        }

        /// Constructor
        impl VisMsg {
            pub fn new(pose: Pose, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
                Self {
                    new_pose: pose,
                }
            }
        }
        ```
5. **Implement the actual actor functionality**
    - Finally, you can implement the functionality of the actor. This is the code that the actor runs on a new message.
    - For the visualizer, this function is:
        ```Rust
        visualize(&mut self, context: Context, message: Message) -> ActorResult<()>
        ```
6. **Register the module in the system**
    - Tip: All the code sections in the following steps can be found by searching for the string ``REGISTER MODULE``
    - In ``src/registered_modules.rs``, add a string to refer to your module. Visualizer example:
    ```Rust
    pub static VISUALIZER: &str = "VISUALIZER";
    ```
    - In ``src/registered_modules.rs``, under ``getmethod``, add a reference to your object. Visualizer example:
    ```Rust
    "vis" => FunctionProxy {function: Box::new(crate::modules::vis::DarvisVis::new(id.clone()))}
    ```
    - In ``src/modules/mod.rs``, add your module. Visualizer example:
    ```Rust
    pub mod vis;
    ```

## Adding new settings in the config file
You can add custom settings to the ``config.yaml`` file. These can be anything, but an example that is already implemented are the settings for camera calibration and ORB feature extraction.

To add a new setting, under ``system`` in the config file, add a new key-value pair. Supported types are ``String``, ``bool``, ``f64``, and ``i32``. You do not have to declare the type in the config file.

To load the new config file to memory when initializing the system, modify the function ``load_config()`` in ``src/main.rs`` to parse for your setting and insert it into the global params. This function already has examples so just copy one of them.

To use the setting in your code, reference it like:
```Rust
let max_features: i32 = SETTINGS.get(SYSTEM, "max_features");
```
Where ``max_features`` is the key you added in the config file.


---
## Key for strings

Strings are formatted like: ``TODO (string)``

- Feature implementation TO DOs
    - **MVP**
    - Sensor types
        - **IMU**
        - **Stereo**
        - **RGBD**
    - Modules
        - **relocalization**
        - **local mapping**
        - **loop closing**
    - Features
        - **multimaps**
        - **reset**
- Design TO DOs
    - **design**
    - **memory** 
- Notes
    - **note**
    - **paper note**
    - **testing**