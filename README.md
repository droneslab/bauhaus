# darvis
Distributed, modulAR Visual SLAM

## Install
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
4. Once inside the container, compile with:
    ```
        cd darvis
        cargo build
    ```

### Working with VSCode
Using compile-time code checking in VSCode typically gives errors about not finding opencv. To resolve these errors, you need to open VSCode inside the container:
1. Download the [Remote - Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) on VSCode
2. Follow [instructions for quick start](https://code.visualstudio.com/docs/remote/containers#_quick-start-open-an-existing-folder-in-a-container) to open VSCode inside container. This opens a new VSCode window
3. For compile-time code checking, in the new VSCode window, install [rust-analyzer extension](https://marketplace.visualstudio.com/items?itemName=matklad.rust-analyzer) 

## Run basic example
```
    cd darvis
    cargo run /datasets/ config.yaml
```

## How to create new modules

All the steps below reference the module ``Visualizer`` as an example.

### Add module in config
In the ``config.yaml`` file, add the settings for your new module under ``modules``. An example from the visualizer:

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

### Writing the module code
For the following steps, you will be working in a new file you create under ``src/modules/`` that holds your module code. The filename should match that which you declared in the config file. Example for visualizer: ``src/modules/vis.rs``.

#### Create the actor
1. Implement the constructor for the actor. Example:
```Rust
impl DarvisVis {
    /// Constructor
    pub fn new(id: String) -> DarvisVis {
        DarvisVis {
            traj_img: Mat::new_rows_cols_with_default(376, 500, core::CV_8UC3, core::Scalar::all(0.0)).unwrap(),
            cam_img: Mat::default(),
            traj_pos: DVVector3::zeros(),
            traj_rot: DVMatrix3::zeros(),
            id: id
        }
    }
```
2. Implement the trait ``Function``. This trait tells axiom which function to run when the actor processes an incoming message. An example from ``vis.rs``:
    ```Rust
    impl Function for DarvisVis {

        fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
        {
            self.visualize(_context, message).unwrap();
            Ok(Status::done(()))
        }

    }
    ```

#### Create the message that the actor processes
1. Create the struct for the message. Example:
    ```Rust
    #[derive(Debug, Serialize, Deserialize)]
    pub struct VisMsg {
        new_pose: Pose,
        actor_ids: std::collections::HashMap<String, axiom::actors::Aid>
    }
    ```
    ==TODO: What are actor ids?==
3. Implement the constructor for the message. Example:
    ```Rust
    impl VisMsg {
        pub fn new(pose: Pose, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
            Self {
                new_pose: pose,
                actor_ids: ids,
            }
        }
    }
    ```

#### Implement the actual actor functionality
Finally, you can implement the functionality of the actor. This is the code that the actor runs on a new message. For the visualizer, this function is:
```Rust
visualize(&mut self, context: Context, message: Message) -> ActorResult<()>
```

### Registering the module in the system
(All the code sections in the following steps can be found by searching for the string ``REGISTER MODULE``)
1. In ``src/registered_modules.rs``, add a string to refer to your module. Example:
    ```Rust
    pub static VISUALIZER: &str = "VISUALIZER";
    ```
2. In ``src/registered_modules.rs``, under ``getmethod``, add a reference to your object. Example: 
    ```Rust
    "vis" => FunctionProxy {function: Box::new(crate::modules::vis::DarvisVis::new(id.clone()))}
    ```
3. In ``src/modules/mod.rs``, add your module like:
    ```Rust
    pub mod vis;
    ```

## How to add new settings in the config file
You can add custom settings to the ``config.yaml`` file. These can be anything, but an example that is already implemented are the settings for camera calibration and ORB feature extraction.

To add a new setting, under ``system_settings`` in the config file, add a new key-value pair. Supported types are ``String``, ``bool``, ``f64``, and ``i32``. You do not have to declare the type in the config file.

To load the new config file to memory when initializing the system, modify the function ``load_custom_settings()`` in ``src/main.rs`` to parse for your setting and insert it into the global params. This function already has examples so just copy one of them.

To use the setting in your code, reference it like:
```Rust
let max_features: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS.to_string(), "max_features".to_string());
```
Where ``max_features`` is the key you added in the config file.