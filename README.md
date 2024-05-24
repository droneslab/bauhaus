# darvis
Distributed, modulAR Visual SLAM

# 1. Installation
```bash
# Install system dependencies 
sudo apt-get update
sudo apt-get install wget git cmake vim clang libclang-dev pkg-config libc6-dbg gdb valgrind libgtk2.0-dev libboost-all-dev

# Install Rust
wget https://raw.githubusercontent.com/rust-lang/rustup/master/rustup-init.sh
chmod +x rustup-init.sh
./rustup-init.sh -y

# Create directories for darvis project, clone repo
mkdir ~/darvis-home && cd ~/darvis-home
git clone git@github.com:droneslab/darvis.git
mkdir ~/darvis-home/depends

# Upgrade version of cmake (needed for C++ bindings)
# Only do this for ubuntu 20, ignore for ubuntu 22! Didn't test on earlier versions than 20.
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
cmake --build build

# Build darvis
cd ~/darvis-home/darvis/darvis/
cargo build --release
```

**Common Build Problems**
- Segfaults inside orbslam-bindings
    - If you have two versions of OpenCV (for example, one from apt that gets downloaded when you install ROS and the one you installed in the instructions above) then you can get segfaults inside the C++ OpenCV code in `orbslam-bindings`. This occurs because C++ finds one OpenCV version (usually the ROS version) and Rust finds a different one (usually the system-installed one). The easiest workaround is just to uninstall ROS and the apt version of OpenCV. It should be possible to inform the rust OpenCV package to look for the right version with [environment variables like this](https://github.com/twistedfall/opencv-rust#environment-variables) but I have not tested it.

## Optional: Using docker
If you want to run darvis inside a docker container instead:
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


## Optional: Set up ORB_SLAM3
If you want to set up ORB_SLAM3 to compare its performance against Darvis You can also just follow the instructions in the ORB_SLAM3 repo, but if darvis is already set up then you only need to do these steps. Tested on Ubuntu 22.

```bash
sudo apt-get install libssl-dev
git clone git@github.com:droneslab/darvis-orbslam3.git
cd ORB_SLAM3
# If using darvis-orbslam3, don't have to do this.
# sed -i 's/++11/++14/g' CMakeLists.txt
./build.sh
```

# 2. Building and Running
RELEASE mode (slow building, fast execution). This is probably what you want to use
```bash
cargo build --release
cargo run --release [DATASET] config.yaml
```
DEBUG mode (fast building, slow execution)
```bash
cargo build
cargo run [DATASET] config.yaml
```

**Using the Visualizer**
1. Download the [foxglove application](https://foxglove.dev/). This can be on any device (does not need to be the test device).
2. Open foxglove, click on `layout` in the top right corner, then `import from file`. Load the file in `foxglove/foxglovelayout.json`.
3. Make sure the `visualizer` actor in `config.yaml` is not commented out. In the actor settings, change the setting `stream" to either true or false (true will stream to a websocket, false will save to an mcap file for later replay).
4. Run the system. If you are streaming, the foxglove application should update with the visualization in a second or two. If you are not streaming, the system will write the file `darvis/results/out.mcap` after execution is over. To open this file in foxglove, click `file > open local file`.

For longer datasets, the mcap file size can get unreasonably large because it saves all the images. You can turn this off by setting the `image_draw_type` setting in the visualizer actor to `none`. The other valid options are `plain` (unmodified images), `features` (detected features), and `featuresandmatches` (current and previous image with feature matches highlighted).

## Debuggers, Memory Checks, and Profiling

### Use GDB
```bash
cargo build    # Either debug or release build works
rust-gdb --args target/debug/bindarvis [DATASET] config.yaml
```
You can also use regular gdb instead of rust-gdb, but [it doesn't work in all cases](https://users.rust-lang.org/t/printing-single-vector-elements-in-gdb/16890/4).

### Check memory errors (useful for verifying C++ code in ffi bindings)
Run with address sanitizer:
```bash
RUSTFLAGS="-Z sanitizer=address" cargo run --target x86_64-unknown-linux-gnu [DATASET] config.yaml
```
Run with valgrind and save output to `log.txt`:
```bash
cargo build # Either debug or release build works
valgrind target/debug/bindarvis  [DATASET] config.yaml > log.txt 2>&1
```

### Check for deadlocks
The RwLock will deadlock if you take two locks in one scope, but this just shows up as an infinite loop on the command line. To find deadlocks, look inside main.rs for the word ``deadlock``, uncomment the lines of code that spawn a thread. This runs a separate thread that periodically wakes up, checks for a deadlock, and prints out the thread info if there is one. Additionally (or alternatively) you can change all RwLocks to Mutexes, which should give an error if you try to lock twice. This should be pretty straightforward by changing ``pub type MapLock`` to ``Arc<Mutex<Map>>`` and then changing occurences of ``read()`` and ``write()`` to ``lock()``.

### Log basic timing info
Use this for very basic timing information. It is not very accurate, so if you need to dig deep into performance you should follow the steps below to use the Tracy profiler instead.

- Print time of certain functions
    - Add ``#[time()]`` before the function name like this:
        ```rust
        #[time()]
        fn function() {}
        ```
    - You can also customize the name of the timer like this: ``#[time("MyFunction")]``. This is useful if you have similarly-named functions.
- To print time of arbitrary sections of code
    - Add ``let timer = timer!()`` when you want to start timing.
    - It will print the time when ``timer`` goes out of scope. Alternatively, you can set the end point by calling ``finish!(timer)``
- Compile/run the normal way, but make sure that ``log_level`` in ``config.yaml`` is set to ``debug``.

### Generate flamegraphs
This requires some setup, [see here](https://www.justanotherdot.com/posts/profiling-with-perf-and-dhat-on-rust-code-in-linux.html)

```bash
cargo flamegraph -o flamegraph.svg --root --release --ignore-status  -- ~/datasets/kitti_00_0/ config.yaml
```

### Profiling with Tracy
**Set up**
1. Set up C++
    ```bash
    apt install libdbus-glib-1-dev libcapstone-dev
    cd ~/darvis-home
    git clone https://github.com/wolfpld/tracy.git
    cd tracy
    git checkout 897aec5 # Set to version 0.9.1
    cd profiler/build/unix && make && cd ../../../
    cd capture/build/unix && make && cd ../../../
    cd csvexport/build/unix && make && cd ../../../
    ```
    Tracy C++ version has to be compatible with tracy-client version in Rust. [See table here](https://github.com/nagisa/rust_tracy_client).
2. Try to run ``./tracy/profiler/build/unix/Tracy-release``. If you get [this error](https://github.com/wolfpld/tracy/issues/567), you need to make [this change](https://github.com/wolfpld/tracy/commit/c57b8994f6dcee2e3312b1a7aec9e055f7a0bb01) to the tracy source code.
2. In darvis ``Cargo.toml``, set tracy-client features to "enable", like this:
    ```rust
    tracy-client = {version = "0.16.0", features = ["enable"] }
    ```
3. Uncomment this line of code in ``main.rs`` :
    ```rust
    let _client = tracy_client::Client::start();
    ```

**Mark up code**
- To time a region, add this line of code. Then the timing will be displayed when the variable goes out of scope.
    ```rust
    let _span = tracy_client::span!("search_for_triangulation");
    ```

**Run**
- Two ways to run: showing the GUI while the program is running, or logging the output and viewing the results in the GUI afterward. Instructions below are for logging and viewing afterward, but if you want it in real-time you should be able to just run the ``Tracy-release`` command in step 3 instead of the ``capture-release`` command in step 1.
1. Start the tracy client (this is from darvis-home directory)
    ```bash
    ./tracy/capture/build/unix/capture-release -o output.tracy
    ```
2. Run darvis normally
3. After darvis ends, tracy capture should have created `output.tracy`. To view this, open the GUI application:
    ```bash
    ./tracy/profiler/build/unix/Tracy-release
    ```
4. Select ``open saved trace`` in the GUI
- If you're on a mac and sshing into the darvis computer or running it in docker, ``brew install tracy`` will actually work to show the tracy GUI. I could not get wayland forwarding to work.
5. To export tracy statistics to a csv, run:
    ```bash
    ./tracy/csvexport/build/unix/csvexport-release output.tracy
    ```
    You can then further process this with the `tracy.py` script in the scripts directory.

**Helpful links**
- [blog post on using tracy with rust](https://www.abhirag.com/blog/tracy/)
- [tracy documentation(download documentation pdf here)](https://github.com/wolfpld/tracy?tab=readme-ov-file)
- [Rust tracy-client crate](https://docs.rs/tracy-client/latest/tracy_client/)

## Misc. Build Options
To compile C++ bindings with clang instead of g++, set these environment variables:
```bash
export CXX=/usr/bin/clang++
export CC=/usr/bin/clang
``````

# 3. Modifying the Code

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


# 4. Key for strings

Strings are formatted like: ``TODO (string)``

- Feature implementation TO DOs
    - **MVP**
    - Sensor types
        - **IMU**
        - **Stereo**
        - **RGBD**
    - Modules
        - **relocalization**
        - **loop closing**
    - Features
        - **multimaps**
        - **reset**
        - **timestamps** -- playing frames based on timestamps in timestamps file, instead of a set fps
        - **visualizer** 
- Design TO DOs
    - **design**
        - **fine-grained locking** -- locking on singular keyframes
        - **concurrency** -- concurrency control between local mapping and loop closing
        - **variable locations** -- variables stored somewhere that isn't ideal ... e.g. global variables that could be messages, member variables that could be local to a function
        - **code organization** -- messy code that could be cleaner
        - **rust issues** -- things rust won't let us do. mostly for references to the map, like getting simultaneous mutable and immutable references to different parts of the map
        - **map connections** -- places where mappoints or keyframes have connections to other parts of the map. it would be nice if we could guarantee that the connections are updated/correct rather than duplicating all these connections across all the objects and hoping we remember to update them correctly after a map modification
    - **timing** -- things that take too long. mostly clones, sometimes LTO for ffi bindings
- Notes
    - **note**
    - **paper note**
    - **testing**

design
timestamps
 
# 5. Link Time Optimization
 **Detailed Information** 
 
To get link time optimization performed on the binary created by rustc (the Rust compiler), it is necessary to use the same versions of LLVM/clang that the current Rust toolchain (in the rust toolchain file in Darvis) uses to compile the C++ code.
If the Rust toolchain is updated to the latest stable/nightly offered by the Rust language maintainers, chances are that most Linux distributions would not have the correct versions of LLVM/clang. This is because the Rust toolchain is updated quite frequently.
If the versions do not match, then **lld** (the linker) will not be able to link the code successfully at the final step. 

Now this poses another problem, namely having multiple versions of clang. One that is installed from the repositories of the distribution and the one that we just compiled. The trick here is to simply build the compiler and not install it. The install operation
normally represented by make install does not have to be run. If it were to be installed, there is a good chance that the operating system might break. All the CMake files for compiling the C++ code need to be modified to accomodate the use of a specific compiler.
This is done using ```set(CMAKE_CXX_COMPILER "path/to/compiler")```.

The linker lld, will also have to be compiled along with clang and llvm. The path to the linker must also be mentioned in the CMake file. This is done using ```add_link_options(-fuse-ld=/path/to/linker/ld.lld)```
Along with this, ```set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)``` must also be specified in the CMake files to enable link time optimization. The version of CMake has to be 3.29 or above for this flag to work. Here we again arrive at the problem of packages in a distribution's repository lagging behind
the package development. Passing the -flto flag in the build.rs file does not work. This will most likely mean that cmake would have to be compiled from source or the binary with the right version would have to be used. This has to be added to the path by modifying ```$PATH``` to accomodate the location where the cmake binary lives (using ```export $PATH```). Also one must pass the following ```RUSTFLAGS``` to cargo for enabling cross language link time optimization - ```RUSTFLAGS="-Clinker-plugin-lto -Clinker=clang -Clink-arg=-fuse-ld=lld" cargo build --release```. 

If running the binary on a standalone computer which is different from the computer that compiled the binary, the standalone computer will require a glibc that matches the version required by clang. If there is no match, the binary will not run. 
To get the binary running, the right version of glibc's shared library (libc.so) has to be compiled from scratch and must be loaded using ```export LD_LIBRARY_PATH=/path/to/newly/built/libc.so```. 
The dynamic linker as well (ie something that looks like ld-linux-x86-64.so.2) in built the glibc directory has to used on the binary. This is accomplished with **patchelf**, a tool that facilitates the use of a different dynamic linker apart from the one in the system's /usr. 
The patchelf tool can be installed from the distribution's repos. For a Debian based system, this would be ```sudo apt-get install patchelf```. The executable for darvis would lie within /target/release, it should be called bindarvis. 

**Specific Steps**

**Building Darvis with LTO**
1. Build the corresponding llvm tools/clang from source. Detailed instructions are given at the following link - https://llvm.org/docs/CMake.html
2. The source files for the above are found here - https://github.com/llvm/llvm-project. Sometimes, volunteers may upload the binaries which may be used, however if no binaries exist, the above mentioned will have to be built from source.
3. Specify the correct compiler and linker path (path specification mentioned above) in all the CMakeLists.txt - These would be the ones corresponding to g2o,orb_slam3 and DBoW2.
4. Build a version of cmake above 3.29 or get a copy of the binary from the following link - https://github.com/Kitware/CMake/releases
5. Add the location of the cmake executable to the $PATH variable using ```export $PATH```. (To do this, use ```echo $PATH``` and add the location at the beginning using the syntax used in the ```$PATH``` variable).
6. For compiling the project, invoke cargo as follows:
   ```RUSTFLAGS="-Clinker-plugin-lto -Clinker=/path/to/clang -Clink-arg=-fuse-ld=lld" cargo build --release```

**Executing the Darvis binary on a non-developmental device** **(if running it normally fails)**
1. Get a copy of the glibc required by device from the following website - https://ftp.gnu.org/gnu/glibc/. This information (version) will be seen in the terminal if bindarvis is unable to execute.
2. Compile glibc from source, the instructions are available at the following website - https://sourceware.org/glibc/wiki/Testing/Builds.
3. Get the bindarvis executable to use the new dynamic linker that we just built using ```patchelf```.
4. Install patchelf using ```sudo apt-get install patchelf``` for Debian based distros or ```sudo pacman -S patchelf``` for Arch based distros.
5. Use the following command in the terminal - ```patchelf --set-interpreter /path/where/newly/compiled/ld-linux-x86-64.so/lives /path/of/darvis_executable(bindarvis)```.
7. Add the directory containing libc.so to ```export LD_LIBRARY_PATH=/path/of/library/with/libc```, this would be where the glibc that we built from source exists.





