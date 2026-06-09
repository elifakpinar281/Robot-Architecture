# Aufteilung & Implementierung (Lab 04b, C++)

Dieses Dokument: Architektur, gemeinsame Schnittstelle, wer macht was, und konkrete Hinweise pro Knoten. Sprache: **C++ (roscpp)**.

Voraussetzung: Die Umgebung läuft (siehe Dokument "Setup für Helena").

---

## 1. Architektur / Dateistruktur

Wir bauen **ein** gemeinsames Package `turtlebot3_control`. Idee: fünf lose gekoppelte Knoten, die nur über Topics und einen Service reden. Jeder Knoten hat genau eine Verantwortung – dadurch können wir parallel arbeiten.

```
turtlebot3_control/
├── CMakeLists.txt              # Build-Konfiguration (Abschnitt 4)
├── package.xml                 # Metadaten + Abhängigkeiten
├── include/turtlebot3_control/ # (optional) Header
├── msg/
│   └── RobotStatus.msg         # eigene Message
├── src/                        # ALLE C++-Knoten
│   ├── camera_node.cpp
│   ├── status_publisher.cpp
│   ├── logger_node.cpp
│   ├── wall_follower.cpp
│   └── move_between.cpp
├── launch/
│   ├── wall_follower.launch
│   └── move_between.launch
└── worlds/
    └── two_objects.world
```

Datenfluss:

```
Gazebo  --/scan--------------->  status_publisher  --/robot_status-->  logger_node --> CSV
Gazebo  --/camera/...image_raw-> camera_node (OpenCV-Fenster)
Verhaltensknoten --/cmd_vel---> Gazebo (Roboter fährt)
status_publisher liest /cmd_vel zusätzlich mit
logger_node bietet Service /set_logging (an/aus)
```

---

## 2. Gemeinsame Schnittstelle (Contract)

**Wichtigster Abschnitt.** Auf diese Namen einigen wir uns fest, bevor wir anfangen.

### Vorhandene Topics der Simulation

| Was | Topic | Typ |
|-----|-------|-----|
| Kamerabild | `/camera/rgb/image_raw` | `sensor_msgs/Image` |
| LiDAR | `/scan` | `sensor_msgs/LaserScan` |
| Fahrbefehl | `/cmd_vel` | `geometry_msgs/Twist` |

### Eigene Message `msg/RobotStatus.msg`

```
float64 linear_speed
float64 angular_speed
float64 distance_front
float64 distance_left
float64 distance_right
```

Topic: **`/robot_status`**. In C++: `#include "turtlebot3_control/RobotStatus.h"` (Header wird beim Bauen generiert).

### Service: Logging an/aus

- Typ: `std_srvs/SetBool`, Name: **`/set_logging`** (`data: true` = an)
- Lebt im `logger_node`
- Test: `rosservice call /set_logging "data: false"`

### Feste Knoten-/Dateinamen

| Datei (`src/`) | Executable / Knotenname | Aufgabe |
|----------------|-------------------------|---------|
| `camera_node.cpp` | `camera_node` | Kamera + OpenCV |
| `status_publisher.cpp` | `status_publisher` | published `/robot_status` |
| `logger_node.cpp` | `logger_node` | loggt + Service `/set_logging` |
| `wall_follower.cpp` | `wall_follower` | Wandverfolgung |
| `move_between.cpp` | `move_between` | Pendeln zwischen Objekten |

Diese Namen nicht ändern – CMakeLists und Launch-Files hängen daran.

---

## 3. Aufteilung (fair, zu zweit)

Jede übernimmt einen der zwei schwierigen Verhaltensknoten. Eli hat das komplette Setup durchgekämpft und richtet das Package-Gerüst ein, deshalb liegt bei Helena etwas mehr Implementierung – unterm Strich ausgeglichen.

### Eli

1. **Package-Gerüst** anlegen + `CMakeLists.txt`/`package.xml` (Abschnitt 4) + die fünf Knoten als leere Stubs + `RobotStatus.msg`, einmal bauen, **als Erstes committen und pushen**
2. **`status_publisher.cpp`** ausfüllen
3. **`wall_follower.cpp`**
4. **`wall_follower.launch`**

### Helena

1. **`camera_node.cpp`** – `/camera/rgb/image_raw` via `cv_bridge`, OpenCV (Graustufen oder Canny), Anzeige im Fenster
2. **`logger_node.cpp` + Service `/set_logging`** – `/robot_status` abonnieren, CSV mit Zeitstempel, Service schaltet an/aus
3. **`move_between.cpp`**
4. **`worlds/two_objects.world`**
5. **`move_between.launch`**

### Einzige Abhängigkeit über Kreuz

Helenas Logger braucht die `RobotStatus.msg` (und die fertige CMakeLists/package.xml). Lösung: **Eli pusht das ganze Gerüst inkl. Message zuerst**, Helena macht `git pull` und kann dann loslegen. Danach arbeiten beide unabhängig in ihren eigenen Dateien.

---

## 4. CMakeLists.txt und package.xml

Bei C++ geht hier am häufigsten etwas schief – deshalb fertig zum Übernehmen.

### `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_control)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  std_srvs
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  RobotStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_runtime cv_bridge std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(status_publisher src/status_publisher.cpp)
target_link_libraries(status_publisher ${catkin_LIBRARIES})
add_dependencies(status_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(logger_node src/logger_node.cpp)
target_link_libraries(logger_node ${catkin_LIBRARIES})
add_dependencies(logger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(wall_follower src/wall_follower.cpp)
target_link_libraries(wall_follower ${catkin_LIBRARIES})
add_dependencies(wall_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_between src/move_between.cpp)
target_link_libraries(move_between ${catkin_LIBRARIES})
add_dependencies(move_between ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

> **Wichtig für den ersten Build:** `add_executable` verlangt, dass die `.cpp`-Datei existiert. Deshalb legt Eli alle fünf Dateien gleich als leere Stubs an (siehe Grundgerüst unten). Dann baut die volle CMakeLists sofort, niemand muss sie später anfassen, und es gibt keine Merge-Konflikte.

### `package.xml`

```xml
<?xml version="1.0"?>
<package format="2">
  <name>turtlebot3_control</name>
  <version>0.1.0</version>
  <description>Lab 04b - Robotersteuerung mit Turtlebot3</description>
  <maintainer email="eli@example.com">Eli</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>

  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>std_srvs</depend>
</package>
```

---

## 5. Knoten-Grundgerüst (Stub)

Jede `.cpp` in `src/` startet mit diesem Minimum (Knotennamen anpassen):

```cpp
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "mein_knoten");
    ros::NodeHandle nh;
    ROS_INFO("mein_knoten gestartet");
    ros::spin();           // wartet auf Callbacks; bei reinen Sende-Knoten durch while-Schleife ersetzen
    return 0;
}
```

Bauen + ausführen:

```bash
cd ~/catkin_ws && catkin_make && source devel/setup.bash
# Terminal 1: roslaunch turtlebot3_gazebo turtlebot3_world.launch
# Terminal 2: rosrun turtlebot3_control status_publisher
```

> C++ muss nach **jeder** Änderung neu gebaut werden (`catkin_make`).

---

## 6. Hinweise pro Knoten (C++)

**status_publisher.cpp** – Zwei Subscriber: `/scan` (vorne/links/rechts merken) und `/cmd_vel` (linear/angular merken). Im 10-Hz-Takt ein `RobotStatus` füllen und auf `/robot_status` publishen. Aus dem `LaserScan`: vorne `msg->ranges[0]`, links `msg->ranges[90]`, rechts `msg->ranges[270]` (Index ≈ Winkel in Grad).

```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "turtlebot3_control/RobotStatus.h"
```

**camera_node.cpp** – cv_bridge in C++:

```cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat bild = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat grau;
    cv::cvtColor(bild, grau, cv::COLOR_BGR2GRAY);   // Beispiel
    cv::imshow("Kamera", grau);
    cv::waitKey(1);
}
// in main: ros::Subscriber sub = nh.subscribe("/camera/rgb/image_raw", 1, imageCallback);
```

**logger_node.cpp** – Subscriber auf `/robot_status`, ein `bool logging_enabled` (Start true), `std::ofstream` für die CSV. Im Callback nur schreiben, wenn `logging_enabled`. Zeitstempel mit `ros::Time::now()`. Service:

```cpp
#include <std_srvs/SetBool.h>

bool logging_enabled = true;
bool setLogging(std_srvs::SetBool::Request& req,
                std_srvs::SetBool::Response& res) {
    logging_enabled = req.data;
    res.success = true;
    res.message = req.data ? "Logging an" : "Logging aus";
    return true;
}
// in main: ros::ServiceServer s = nh.advertiseService("/set_logging", setLogging);
```

**wall_follower.cpp** – Einfache regelbasierte Logik reicht: vorwärts bis vorne eine Wand ist, dann drehen und den seitlichen Abstand grob konstant halten. Subscriber `/scan`, Publisher `/cmd_vel` (`geometry_msgs/Twist`). Muss nicht perfekt sein.

**move_between.cpp** – Zwei Objekte per LiDAR erkennen, auf eines zufahren, kurz davor umdrehen, zum anderen fahren, wiederholen. Roboter darf zu Beginn dazwischen stehen.

**Launch-Files** – Jedes startet alle fünf Knoten. Beispiel:

```xml
<launch>
  <node pkg="turtlebot3_control" type="status_publisher" name="status_publisher" output="screen"/>
  <node pkg="turtlebot3_control" type="logger_node"      name="logger_node"      output="screen"/>
  <node pkg="turtlebot3_control" type="camera_node"      name="camera_node"      output="screen"/>
  <node pkg="turtlebot3_control" type="wall_follower"    name="wall_follower"    output="screen"/>
</launch>
```

> `type` ist der Executable-Name **ohne** Endung.

---

## 7. Git-Workflow zu zweit

- Ein gemeinsames Repo, jede auf eigenem Branch (`eli`, `helena`), regelmäßig mergen.
- Eli legt alle fünf `.cpp` als Stubs an, schreibt die vollständige `CMakeLists.txt` und pusht zuerst. Danach fasst niemand mehr die CMakeLists an → keine Konflikte dort.
- Nach Elis erstem Push: Helena `git pull`, dann hat sie Message + CMake und baut ihre Knoten.
- Jede arbeitet nur in ihren eigenen `.cpp`/`.launch`-Dateien → kaum Konflikte.

---

## 8. Abgabe

Zip mit dem kompletten Package `turtlebot3_control` (inkl. `src/`, `msg/`, `launch/`, `worlds/`, `CMakeLists.txt`, `package.xml`) und ggf. einer Beispiel-Logdatei.
