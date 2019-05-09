EUROBENCH Benchmark Server
=================================================

Benchmark server and benchmark scripts for the EUROBENCH framework.

## Installation

Install the ROS package as usual:
```
cd catkin_workspace/src
git clone https://github.com/joaomacp/eurobench_benchmark_server.git
cd ../
catkin_make
```

An output directory should be created to store benchmark results. The default location is `~/eurobench_output`, and can be configured in `config/general.yaml`.

## Running benchmarks

Start the benchmark server:
```
roslaunch eurobench_benchmark_server benchmark_server.launch 
```

A GUI for running benchmarks doesn't exist yet. Start a benchmark by calling the `/bmserver/start_benchmark` service:
```
rosservice call /bmserver/start_benchmark "benchmark_code: 'TEST_BENCHMARK' robot_name: 'TestRobot'"
```

`rosservice call /bmserver/stop_benchmark` can be called to stop a benchmark prematurely.

The `/bmserver/state` topic is published at 10Hz with the state of the server and the currently running benchmark, if there is one. To watch it, run `rostopic echo /bmserver/state`.

When a benchmark finishes, its results are written to the output directory.

An example benchmark script (with ID `TEST_BENCHMARK`) is included. It listens to a simple On/Off sensor to determine if a robot passed through it. A rosbag with fake sensor readings is played in a loop by the launch file.
