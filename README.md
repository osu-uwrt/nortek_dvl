# Nortek DVL1000 ROS2 Interface Layer

ROS Interface layer for Nortek DVL1000 by your friends at [ARVP](https://arvp.org) from University of Alberta

Connects to DVL over Ethernet (port:9004) and publishes DVL data + status.
Velocities are output in m/s (or NaN if invalid).
Custom messages are used for both publishers.

**Launching the node:** `ros2 launch nortek_dvl dvl.launch`

### Parameters

* **address** IP address / host name of DVL
* **port** port number for TCP connection
* **timeout** TCP connection timeout waiting for DVL response
* **max_connect_time** maximum time spent waiting for the DVL server to become availiable before the node gives up
* **frame_id** dvl position frame id
* **sonar_frame_id** dvl sonar frame id name(s)
* **use_enu** Whether to report twist in ENU frame


*please note that both publishers are in the nodes private namespace*

### DVL configuation

check the [DVLConfig.txt](DVLConfig.txt) file for our DVL configuration.
