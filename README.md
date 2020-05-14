EUROBENCH Benchmark Software
=================================================

Benchmark core, benchmark scripts and graphical interface for the MADROB and BEAST benchmarks.

## Dependencies
This package currently depends on:
```
https://github.com/madrob-beast/madrob_beast_pi.git
https://github.com/madrob-beast/madrob_msgs.git
https://github.com/madrob-beast/madrob_srvs.git
https://github.com/madrob-beast/beast_msgs.git
https://github.com/madrob-beast/beast_srvs.git
```

## Installation
The required ROS packages and the [madrob_beast_pi](https://github.com/madrob-beast/madrob_beast_pi.git) python module should be cloned and installed:
```
cd ~/catkin_ws/src  # or another ROS workspace
git clone https://github.com/madrob-beast/madrob_beast_pi.git
python -m pip install -e madrob_beast_pi/src/madrob_beast_pi

git clone https://github.com/madrob-beast/eurobench_benchmarking_software.git
git clone https://github.com/madrob-beast/madrob_msgs.git
git clone https://github.com/madrob-beast/madrob_srvs.git
git clone https://github.com/madrob-beast/beast_msgs.git
git clone https://github.com/madrob-beast/beast_srvs.git
cd ../
catkin_make

```

**Note** To keep the software up-to-date, run `git pull`, reinstall `madrob_beast_pi` and rebuild the catkin workspace.

## Running benchmarks

- To start the core and GUI in the same machine:
```
roslaunch eurobench_benchmark_core madrob_all.launch 
```

- To start the core and GUI in separate machines:
```
[Launch the core in PC1]
roslaunch eurobench_benchmark_core madrob.launch 

[Launch the GUI in PC2, connected to ROS Master in PC1]
roslaunch benchmark_gui madrob_gui.launch
```

- To launch a visualization GUI, showing sensor plots and camera images:
```
roslaunch benchmark_gui madrob_sensors_gui.launch
```

The same procedures apply for the BEAST benchmark (replace `madrob` with `beast`), however BEAST is currently not implemented.

Use the GUI to select the current robot, the benchmark to run, and specific settings for either the MADROB or BEAST environment.
Benchmarks can be started, stopped prematurely, and the results can be viewed on the right.

When a benchmark finishes, its results are written to the output directory.

**Note**: The default output directory is `~/eurobench_output`, and can be configured in `config/general.yaml`.