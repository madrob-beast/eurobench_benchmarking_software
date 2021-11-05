EUROBENCH Benchmark Software
=================================================

Benchmark core, benchmark scripts and graphical interface for the MADROB and BEAST benchmarks.

## Running benchmarks

- BEAST Cart
```
ssh beast@beast-cart.local
beast_cart_start
```
In a seperate terminal:
```
rosrun eurobench_benchmark_core beast_cart_live_benchmark.sh
```
The following lines should be in ~/.bashrc on the cart onboard computer:
```
export ROS_MASTER_URI='http://10.0.0.213:11311'
export ROS_IP='10.0.0.213'
alias beast_cart_start='roslaunch eurobench_benchmark_core beast_cart_onboard.launch'
```

- Walker
```
ssh beast@beast-walker.local
beast_walker_start
```
In a seperate terminal:
```
rosrun eurobench_benchmark_core beast_walker_live_benchmark.sh
```
The following lines should be in ~/.bashrc on the onboard computer:
```
export ROS_MASTER_URI='http://10.0.0.128:11311'
export ROS_IP='10.0.0.128'
alias beast_walker_start='roslaunch eurobench_benchmark_core beast_walker_onboard.launch'
```

- Madrob TODO


**Note**: The default output directory on the onboard computers is `~/eurobench_output`, and can be configured in `config/general.yaml`.

## Acknowledgements

<a href="http://eurobench2020.eu">
  <img src="http://eurobench2020.eu/wp-content/uploads/2018/06/cropped-logoweb.png"
       alt="rosin_logo" height="60" >
</a>

Supported by Eurobench - the European robotic platform for bipedal locomotion benchmarking.
More information: [Eurobench website][eurobench_website]

<img src="http://eurobench2020.eu/wp-content/uploads/2018/02/euflag.png"
     alt="eu_flag" width="100" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 779963.

The opinions and arguments expressed reflect only the author‘s view and
reflect in no way the European Commission‘s opinions.
The European Commission is not responsible for any use that may be made
of the information it contains.

[eurobench_logo]: http://eurobench2020.eu/wp-content/uploads/2018/06/cropped-logoweb.png
[eurobench_website]: http://eurobench2020.eu "Go to website"
