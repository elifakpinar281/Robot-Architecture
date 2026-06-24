# Lab 4b - Robot Architecture (ROS / TurtleBot3)

This lab controls a TurtleBot3 (waffle_pi) with ROS. The application is split
into several small nodes. The nodes only talk to each other through ROS topics
and one service. ROS works as the middleware. It carries the sensor data, the
drive commands and our own status message between the nodes. Gazebo runs the
simulation for the whole system.

We built two behaviours. The wall follower keeps an even distance to a wall and follows it. The move-between node drives back and forth between two objects. Both behaviours use the same supporting nodes, which are the camera, the status publisher, the logger and the service. Only the last node in the chain, the behaviour node, changes. 

## Running the Project

The robot model has to be `waffle_pi`. Only the waffle models carry a camera.
Both launch files set the model themselves. `move_between.launch` sets it when it
spawns the robot. `wall_follower.launch` sets it by passing `model:=waffle_pi`
into the included world. So the environment variable is not needed for our nodes.
You only need it if you also use TurtleBot3 tools that read it directly, such as
`turtlebot3_teleop`. In that case set it once in the shell or in `~/.bashrc`:

```shell
export TURTLEBOT3_MODEL=waffle_pi
```

Build the workspace and source it:
 
```shell
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```


Then start one of the two behaviours:
 
```shell
roslaunch turtlebot3_control move_between.launch
roslaunch turtlebot3_control wall_follower.launch
roslaunch turtlebot3_control wall_follower_stage1.launch
roslaunch turtlebot3_control wall_follower_stage4.launch
```

Each launch file brings up Gazebo, spawns the robot and starts all nodes. The
camera node opens two OpenCV windows. The logger writes a CSV file into the
`logs/` folder. The robot then starts to move on its own.

## Architecture

The application does not run as one program. It runs as a set of separate
processes (nodes). ROS connects them. Nodes never call each other directly. A
node publishes messages on a named topic. Any node that is interested in that
topic subscribes to it. Neither side knows the other. They only agree on the
topic name and the message type. This is the publish and subscribe model. It is
the reason the nodes are replaceable. A behaviour node only has to read `/scan`
and write `/cmd_vel`. So the two behaviours plug into the exact same
infrastructure and no other node has to change.
 
There is one case where publish and subscribe is the wrong tool. Switching
logging on and off is a request that expects an answer. It is not a stream of
data. So we modelled it as a ROS service and not as a topic.
 
The node graph is the same for both behaviours. They share the same supporting nodes. Only the behaviour node and one detail in the setup differ

*** Wall follower ***

<p align="center">
  <img src="assets/wall_follower.png" alt="Wall Follower Graph" width="1000">
</p>

*** Move Between ***

<p align="center">
  <img src="assets/move_between.png" alt="Move Between Graph" width="950">
</p>

The move-between graph also shows `/robot_state_publisher` and `/joint_states`. Both behaviours actually run that node. The wall follower gets it from the included `turtlebot3_world.launch`. The move-between launch builds an empty world and spawns the robot itself, so it has to start the `robot_state_publisher` on its own. Otherwise nothing would publish the robot frames. The graphs look a little different only because the wall follower capture hides the leaf topics. 


The following table lists every node with the topics it reads and writes. The
topic names and message types are the contract between the nodes.
 
| Node               | Subscribes / calls               | Publishes / serves        |
|--------------------|----------------------------------|---------------------------|
| `camera_node`      | `/camera/rgb/image_raw`          | OpenCV windows (no topic) |
| `status_publisher` | `/scan`, `/cmd_vel`              | `/robot_status`           |
| `logger_node`      | `/robot_status`, `/set_logging`  | CSV file                  |
| `wall_follower`    | `/scan`                          | `/cmd_vel`                |
| `move_between`     | `/scan`                          | `/cmd_vel`                |
 
The service `/set_logging` does not show up in the node graph. That is normal.
`rqt_graph` draws topics, not services.


## Building Blocks

### Custom Message: `RobotStatus`

The assignment asks for a custom message. It has to carry the current velocity
command and the relevant LiDAR distances. We defined it with five `float64`
fields plus a standard header.

```
std_msgs/Header header
float64 linear_speed
float64 angular_speed
float64 distance_front
float64 distance_left
float64 distance_right
```

The velocity part is the actual command on `/cmd_vel`. The three distances are
the front, left and right readings of the LiDAR. We added the header on purpose.
It carries the time stamp of the measurement, so the logger can write the time
the data was taken and not the time it happened to be written. The message is
generated at build time by `message_generation`. This produces the
`turtlebot3_control::RobotStatus` C++ type that the publisher and the logger
share.
 

### Service: `/set_logging`

Logging is switched with the standard `std_srvs/SetBool` service. We did not need
a custom service type. The request is a single boolean. The response is just a
success flag and a short message. The service lives inside the logger node and
not in a node of its own. It only flips a flag that the logger owns. So it needs
no separate node and no extra entry in the launch files. Starting the logger
starts the service with it.


### `camera_node`

The camera node subscribes to `/camera/rgb/image_raw`. It converts each incoming
ROS image to an OpenCV `cv::Mat` with `cv_bridge`. The conversion uses the `bgr8`
encoding. That is the colour layout OpenCV expects. We wrapped the conversion in
a `try` and `catch` block. A wrong encoding throws an exception and would
otherwise stop the node.
 
The image processing is a grayscale conversion followed by Canny edge detection.
Both the original frame and the edge image are shown in their own OpenCV window.
The `cv::waitKey(1)` at the end is not optional. Without it the windows never
process their GUI events and stay blank.


### `status_publisher`

The status publisher is the only node that produces the `RobotStatus` message. It
subscribes to `/scan` and `/cmd_vel`. It keeps the latest values in plain
variables. It then publishes a fresh `RobotStatus` at a fixed 10 Hz in its main
loop. We decoupled the callbacks from the publish rate on purpose. This way the
output rate is steady and does not depend on how fast the sensor happens to fire.
 
The three LiDAR readings are taken at configurable angles. The defaults are 0
degrees for the front, 90 degrees for the left and 270 degrees for the right.
Each reading is not a single beam. It is the minimum over a small window around
the angle. A helper called `minRange` does this. It also throws out invalid
values like `inf`, `NaN` or anything outside the sensor range. If the whole
window is invalid it returns `range_max`. So the message never carries garbage
distances.


### `logger_node`

The logger subscribes to `/robot_status`. It appends one CSV line per message.
Each line starts with the time stamp from the message header, which is the time
the measurement was taken. The file is created once at start up. It gets a header
row and `std::fixed` formatting, so the time stamp is written as a plain decimal
and not in scientific notation.
 
The file name carries the current date and time. It is built with `strftime`. So
every run gets its own log and old logs are never overwritten.

```
log_24_06_2026_190646.csv
```

The path is resolved at runtime with
`ros::package::getPath("turtlebot3_control")`. The file is then placed in the
`logs/` folder of the package. We do this instead of a relative name for a
reason. When a node is started through `roslaunch`, the working directory is
`~/.ros`. A relative name would put the log there and not in the package.
 
The service callback only sets the `logging_enabled` flag. The real gate is one
line at the top of the message callback.
 
```cpp
if (!logging_enabled) return; // when off, write nothing
```
 
This meets the requirement that no new entries are written while logging is off.
Logging starts enabled.


### `wall_follower`

The wall follower keeps the wall on its right and follows it with a simple
rule-based controller. It reads three sectors of the LiDAR. Each sector is the
minimum over a small window and not a single beam. So one missing ray cannot
upset the decision.
 
| Sector      | Centre | Purpose                       |
|-------------|--------|-------------------------------|
| front       | 0°     | detect a wall straight ahead  |
| right       | 270°   | hold the distance to the wall |
| front-right | 315°   | see an inside corner early    |
 
The decision each cycle is a short cascade. If there is a wall in front, or the
front-right is already too close, the robot stops driving forward and turns in
place to escape the corner. Otherwise it drives forward and only corrects its
heading. If the right distance is larger than the target, it steers towards the
wall. If it is smaller, it steers away. Inside the tolerance band it goes
straight. The target distance is 0.3 m with a 0.1 m tolerance. This is the band
that keeps the motion from swinging back and forth.


### `move_between`

The move-between node drives back and forth between the two boxes. It looks at a
front cone of plus and minus 90 degrees. Instead of using one beam, it groups the
LiDAR returns into objects. This is a small clustering step. Two neighbouring
returns belong to the same object when their distance is close. A jump in
distance larger than `cluster_gap` starts a new object. Objects with fewer than
`min_points` returns are dropped as noise.
 
Each object is reduced to two numbers. The first is its distance, taken as the
nearest point of the object. The second is its bearing, taken as the mean angle
of the object. A positive bearing means the object is to the left. From all
objects in the cone, the node picks the one with the smallest absolute bearing.
That is the object most directly ahead.
 
The node is a two-state machine.
 
- **DRIVE.** Drive forward and steer the chosen object to the centre with a
  proportional term, `angular.z = steer_gain * bearing`. So the robot approaches
  the box head on and does not drift past it. If no object is ahead, the node
  turns slowly in place to search for one. When the object gets closer than the
  stop distance, switch to TURN.
- **TURN.** Rotate in place. First turn until the box just reached is no longer
  ahead. This is the `cleared` step. Then keep turning until the other box is
  centred ahead again. A timeout of one full rotation acts as a fallback, so the
  node can never get stuck.
The important design point is that the turn is closed by the sensor and not by a
timed open-loop 180 degrees. A fixed turn time drifts a little every cycle
because of acceleration ramps and physics. The error then adds up until the robot
faces empty space. Turning until the next object is actually centred removes that
drift. The steering term in DRIVE corrects whatever small error is left.


## Simulation Environment

The move-between behaviour uses its own world, `worlds/two_objects.world`. It
contains a ground plane, a light and two static boxes of 0.5 m. One box sits at
`x = +2` and the other at `x = -2`. The robot is spawned at the origin, exactly
between them. The boxes are static, so they do not move when the robot drives near
them.
 
The wall follower uses the stock `turtlebot3_world` from `turtlebot3_gazebo`. It
already provides walls to follow. Either world can be swapped for another stage.
Nothing in the nodes is tied to a specific map.


## Launch Files

Each behaviour has its own launch file. Both start the full set of nodes the
assignment asks for. These are the camera node, the status publisher, the logger
(which also hosts the service) and the matching behaviour node.
 
- **`move_between.launch`** loads an empty world with `two_objects.world`, spawns
  the `waffle_pi` at the origin and starts the four nodes ending in
  `move_between`.
- **`wall_follower.launch`** includes `turtlebot3_world.launch` with the model set
  to `waffle_pi` and starts the four nodes ending in `wall_follower`.
There are also `wall_follower_stage1.launch` and `wall_follower_stage4.launch`.
They are the same setup in two other stock worlds. We use them to test the wall
follower in different maps.



## Properties of the Architecture

### Loose Coupling

No node holds a reference to another node. They share only topic names and
message types. Because of this, the two behaviour nodes are interchangeable. Both
read `/scan` and write `/cmd_vel`. So the camera, the status publisher and the
logger do not change at all when we switch behaviours. The same property lets us
replace the simulator with a different world. In principle it would also work
with a real robot, without touching the nodes.


### Separation of Concerns

Each node does one job. The camera node only looks at images. The status
publisher only assembles the status message. The logger only writes it down. The
behaviour node only decides how to move. This keeps every node small. It also
makes it possible to test or restart one node on its own.

### Synchronous vs. Asynchronous Communication

Most of the system is asynchronous. Data flows over topics and a publisher does
not wait for its subscribers. Logging control is the exception. Turning logging
on or off is a synchronous request that expects a reply. This is why it is a
service and not a topic. Choosing the right one of the two for each case is part
of the architecture and not an afterthought.

---

The package contains the five nodes, the custom message, the service usage, the
two worlds and the two launch files. The TurtleBot3 description, the Gazebo
launch files and the simulator are used as provided and are not modified.
