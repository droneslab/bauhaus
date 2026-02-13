Library to compile and use foxglove proto files

## Example 
1. Run the example code:
    > cargo run --example straight_trajectory 
2. The mcap file gets saved to `results/out.mcap`
3. Open foxglove, click `open local file`, and select `out.mcap`

## Running with Bauhaus
The `visualizer` actor in bauhaus contains the code for writing bauhaus visualization to the mcap file. By default it will go to `bauhaus/results/out.mcap` but you can change this in the config file.
To open up the layout for the bauhaus visualization, click on `layout` in the top right corner of foxglove, then `import from file`. Load the file `foxglovelayout.json`.
