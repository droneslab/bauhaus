# darvis
Distributed, modulAR Visual SLAM

## Quick Start
- **Install**
    - **Normal install**
        1. System dependencies
            ```bash
            sudo apt-get update
            sudo apt-get install wget git cmake vim clang libclang-dev pkg-config libc6-dbg gdb valgrind libgtk2.0-dev 
            ```
        2. Rust 
            ```bash
            wget https://raw.githubusercontent.com/rust-lang/rustup/master/rustup-init.sh
            chmod +x rustup-init.sh
            ./rustup-init.sh -y
            ```
        3. OpenCV
            ```bash
            git clone https://github.com/opencv/opencv.git
            cd opencv && git checkout tags/4.5.4 && cd ..
            git clone https://github.com/opencv/opencv_contrib.git
            cd opencv_contrib && git checkout tags/4.5.4 && cd .. 
            cd opencv && mkdir build && cd build
            cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DBUILD_opencv_xfeatures2d=ON -DOPENCV_ENABLE_NONFREE=ON -DWITH_GTK=ON ..
            make -j4 install
            ```
        4. Upgrade version of cmake (needed for C++ bindings)
            ```bash
            sudo apt remove --purge cmake && hash -r
            sudo apt install build-essential libssl-dev
            sudo wget https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2.tar.gz
            tar -zxvf cmake-3.20.2.tar.gz
            cd cmake-3.20.2 && ./bootstrap && make && make install
            ```
        5. g2o bindings package
            ```bash
            sudo apt install libeigen3-dev
            git clone https://github.com/ssemenova/g2o-bindings.git
            cd g2o-bindings/g2orust/ && cargo build
            ```
    - **With docker**
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
    - **Using VSCode**
        - Using compile-time code checking in VSCode typically gives errors about not finding opencv. To resolve these errors, you need to open VSCode inside the container:
            1. Download the [Remote - Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) on VSCode
            2. Follow [instructions for quick start](https://code.visualstudio.com/docs/remote/containers#_quick-start-open-an-existing-folder-in-a-container) to open VSCode inside container. This opens a new VSCode window
            3. For compile-time code checking, in the new VSCode window, install [rust-analyzer extension](https://marketplace.visualstudio.com/items?itemName=matklad.rust-analyzer) 

- **Compile**
    ```bash
        cd darvis
        cargo build
    ```
- **Run**
    ```bash
        cd darvis
        cargo run /datasets/ config.yaml
    ```

### Different options for running/building
- DEBUG mode (fast building, slow execution):
    ```bash
    cargo build
    cargo run [DATASET] config.yaml
    ```

- RELEASE mode (slow building, fast execution):
    ```bash
    cargo build --release
    cargo run --release [DATASET] config.yaml
    ```

- To use GDB:
    ```bash
    cargo build    # Either debug build or release build works
    rust-gdb --args target/debug/bindarvis [DATASET] config.yaml
    ```
    (You can also use regular gdb, instead of rust-gdb, but [it doesn't work in all cases](https://users.rust-lang.org/t/printing-single-vector-elements-in-gdb/16890/4).)

- To use address sanitizer (check memory errors from the ffi bindings):
    ```bash
    RUSTFLAGS="-Z sanitizer=address" cargo run --target x86_64-unknown-linux-gnu [DATASET] config.yaml
    ```





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

To add a new setting, under ``system_settings`` in the config file, add a new key-value pair. Supported types are ``String``, ``bool``, ``f64``, and ``i32``. You do not have to declare the type in the config file.

To load the new config file to memory when initializing the system, modify the function ``load_config()`` in ``src/main.rs`` to parse for your setting and insert it into the global params. This function already has examples so just copy one of them.

To use the setting in your code, reference it like:
```Rust
let max_features: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "max_features");
```
Where ``max_features`` is the key you added in the config file.


---
## Future work, key for strings

Strings are formatted like: ``TODO (string)``

### Implementation
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

### Design
- Axiom framework
    - **msg copy**
