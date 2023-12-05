Library to compile and use foxglove proto files

## Example 
1. Run the example code:
    > cargo run --example straight_trajectory 
2. The mcap file gets saved to `results/out.mcap`
3. Open foxglove, click `open local file`, and select `out.mcap`

## Running with Darvis
The `visualizer` actor in darvis contains the code for writing darvis visualization to the mcap file. By default it will go to `darvis/results/out.mcap` but you can change this in the config file.
To open up the layout for the darvis visualization, click on `layout` in the top right corner of foxglove, then `import from file`. Load the file `foxglovelayout.json`.
