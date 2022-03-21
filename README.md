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

