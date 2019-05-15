# Overview

This package contains an model validation setup for the ROS executor behaviour. It corresponds to _Section 3_ of [the paper](https://t-blass.de/papers/response-time-analysis-of-ros2.pdf).

It consists of two components:
- A C++ validation node in src/arbitrary_cb_time.cpp, that provides callbacks that block for a user-defined amount of time.
- A python node in bin/send_cb_sequence that sends commands to the validation node and evaluates its output.

# Instructions

* Install ROS 2 *Crystal Clemmys* as described in the [installation manual](https://index.ros.org/doc/ros2/Installation/). The ROS base install suffices.
  We recommend to use the binary debian packages installation on Ubuntu; for other Linux distributions, we recommend the ["fat archive" installation](https://index.ros.org/doc/ros2/Installation/Linux-Install-Binary/).

  *Note*: we have only tried the programs on Linux. While they are likely to work on Mac or even on Windows, we cannot guarantee this.

* Install `colcon` as described in the [colcon manual](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#id7)


* Create a workspace and copy the given directory into it
  ```
  mkdir -p ros_ws/src && cd ros_ws
  cp -r <path to model_validation> src
  ```
* Build the package and update the environment
  ```
  colcon build --symlink-install
  source install/local_setup.sh
  ```
* Run the C++ validation node
   `arbitrary_cb_time`
* In a new shell, source the local_setup.sh script again and run the cb sequence sender
  `send_cb_sequence`

The `send_cb_sequence` reports the order in which it sends the
callbacks. Consecutive callbacks are sent without delay; use the _delay_ pseudo-callback to wait for one callback duration before continuing. The order `low high delay high`, for example, means that
the script publishes on the _low_ and the _high_ topic, waits for one
callback duration and then publishes another message on the _high_
topic.

In the end, `send_cb_sequence` reports the order in which the
callbacks were executed. Topics and services are identified by their
name, timers by their index (i.e., _timer3_ is the fourth timer
created in this `arbitrary_cb_time` instance).

Since callbacks are executed non-preemptively and do not self-suspend,
the resulting callback sequence uniquely determines the execution
sequence shown in the paper. To verify Figure 3, compare the reported
callback sending order with the one given in Section 3, and the timer
release points with the marked times T1 and T3. Ensure, that the
`delay` commands in the callback sequence cause the second batch of
callbacks to be released at T2, and that the callbacks take 500
milliseconds each. Finally, compare the execution order reported by
`send_cb_sequence` to the execution order in the Gantt-Chart.

# Troubleshooting

## Timers complete before the first _high_ callback

This sometimes happens on virtual machines, and is caused by the
validation node taking over 200ms to notice the _high_ message. If
that happens, the timer overtakes the initial _high_ message and the
entire execution sequence changes because of it. Please note that this
new sequence is still exactly as predicted by our model and supports
our claims. Nevertheless, it is a different scenario than the one
described in our paper.

To avoid this issue, you can adjust the scale_factor variable in
`bin/send_cb_sequence`. This scales all the execution times and timer
releases up by the given factor. Since this experiment only depends on
the relative position of the callback arrivals, this change has no
effect on the validity of the experiment or the correctness of our
claims. It only increases the resiliency of the experiment against
timing uncertainties, at the cost of a longer execution time of the
experiment.

## CMake cannot find ament_cmake_auto

A CMake error message like

```
CMake Error at CMakeLists.txt:29 (find_package):
  By not providing "Findament_cmake_auto.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "ament_cmake_auto", but CMake did not find one.
```

suggests that you haven't set up the ROS environment properly. Please
make sure that you always [source the ROS setup script](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/#sourcing-the-setup-script)

