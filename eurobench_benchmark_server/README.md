EUROBENCH Benchmark Server
=================================================

Benchmark server and benchmark scripts for the EUROBENCH platform.

## Installation

Install the ROS package as usual:
```
cd catkin_workspace/src
git clone https://github.com/eurobench/eurobench_benchmarking_software.git
cd ../
catkin_make
```

You should create an output directory to store benchmark results. The default location is `~/eurobench_output`, and can be configured in `config/general.yaml`.

## Running benchmarks

Start the benchmark server in either MADROB or BEAST mode:
```
roslaunch eurobench_benchmark_server madrob.launch 
OR
roslaunch eurobench_benchmark_server beast.launch
```

Use the GUI to select the current robot, the benchmark to run, and specific settings for either the MADROB or BEAST environment.
Benchmarks can be started, stopped prematurely, and the results can be viewed on the right.

When a benchmark finishes, its results are written to the output directory.

Two example benchmark scripts (with ID `TEST_BENCHMARK_MADROB` and `TEST_BENCHMARK_BEAST`) are included. They listen to a simple On/Off sensor to determine if a robot passed through it. A rosbag with mock sensor readings is played in a loop by the launch file.