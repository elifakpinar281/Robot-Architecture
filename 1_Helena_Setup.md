# Setup-Anleitung (für Helena)

Ziel: Du bekommst dieselbe funktionierende ROS-Umgebung wie Eli. Am Ende öffnet sich bei dir Gazebo mit dem Turtlebot. Dann bist du startklar.

Stack: **Windows 11 + WSL2 + Ubuntu 20.04 + ROS Noetic**.

> Wichtig: Es muss **Ubuntu 20.04** und **ROS Noetic** sein. Andere Versionen passen nicht zusammen. Plane ca. 30–60 Minuten ein (das meiste ist Warten beim Download).

---

## 1. WSL2 + Ubuntu 20.04

In **PowerShell als Administrator**:

```powershell
wsl --install -d Ubuntu-20.04
```

Falls `wsl` noch nicht existiert: erst `wsl --install` ohne Distro ausführen, PC neu starten, dann den Befehl oben. Beim ersten Start von Ubuntu legst du einen Benutzernamen und ein Passwort an (das Passwort merken, du brauchst es für `sudo`).

Ab jetzt arbeitest du in der **Ubuntu-Shell**. Die erkennst du am Prompt `name@rechner:...$` (PowerShell zeigt dagegen `PS C:\...>`).

Zuerst ins Linux-Home wechseln:

```bash
cd ~
```

---

## 2. ROS Noetic installieren

Paketquelle eintragen (fragt nach deinem Passwort – beim Tippen siehst du nichts, das ist normal):

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

curl installieren und den ROS-Schlüssel hinzufügen:

```bash
sudo apt update
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo apt update
```

> Eine Warnung "apt-key is deprecated" ist auf Ubuntu 20.04 egal.
> Falls beim `apt update` ein Fehler `NO_PUBKEY` kommt, hast du den `curl ... | sudo apt-key add -`-Schritt übersprungen – einfach nachholen.

Die große Installation (ca. 2–3 GB, dauert ein paar Minuten):

```bash
sudo apt install ros-noetic-desktop-full -y
```

ROS automatisch in jedem Terminal laden + Build-Tools installieren:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools -y
sudo rosdep init
rosdep update --include-eol-distros
```

> Das `--include-eol-distros` ist wichtig: Noetic ist offiziell "end of life", und ohne das Flag wird es übersprungen und du bekommst später Fehler.

---

## 3. Catkin Workspace anlegen

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. Turtlebot3 installieren (Achtung: zwei Stolpersteine!)

Repos klonen:

```bash
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

**Stolperstein 1 – falscher Branch.** `git clone` holt automatisch die ROS2-Version. Wir brauchen ROS1, also in allen drei Repos auf den `noetic`-Branch wechseln:

```bash
cd ~/catkin_ws/src/turtlebot3 && git checkout noetic
cd ~/catkin_ws/src/turtlebot3_msgs && git checkout noetic
cd ~/catkin_ws/src/turtlebot3_simulations && git checkout noetic
```

**Stolperstein 2 – Hardware-Pakete löschen.** Die brauchen wir für die Simulation nicht, und sie verhindern sonst, dass `rosdep` durchläuft:

```bash
rm -rf ~/catkin_ws/src/turtlebot3/turtlebot3_bringup
rm -rf ~/catkin_ws/src/turtlebot3/turtlebot3_navigation
rm -rf ~/catkin_ws/src/turtlebot3/turtlebot3_slam
rm -rf ~/catkin_ws/src/turtlebot3/turtlebot3_example
rm -rf ~/catkin_ws/src/turtlebot3/turtlebot3
```

Abhängigkeiten auflösen und bauen:

```bash
cd ~/catkin_ws
rosdep install --from-paths src -i -y
catkin_make
source ~/catkin_ws/devel/setup.bash
```

`rosdep` sollte "All required rosdeps installed successfully" sagen, und `catkin_make` ohne Fehler durchlaufen (dauert ein paar Minuten).

---

## 5. Modell setzen und testen

```bash
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

> Wir nehmen **waffle_pi**, weil nur dieses Modell eine Kamera hat (brauchen wir später).

**Du bist fertig, wenn:** sich ein Gazebo-Fenster mit einer Umgebung und dem Roboter öffnet. Beenden mit `Strg + C` im Terminal.

Falls das Fenster schwarz bleibt oder extrem ruckelt (kommt in WSL vor):

```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
```

Danach `roslaunch turtlebot3_gazebo turtlebot3_world.launch` nochmal starten.

---

## 6. Editor einrichten (VS Code)

Zum Programmieren:

1. VS Code unter Windows installieren.
2. In VS Code die Erweiterungen **"WSL"** und **"C/C++"** (beide von Microsoft) installieren.
3. In der Ubuntu-Shell:

```bash
cd ~/catkin_ws/src
code .
```

VS Code öffnet sich "in WSL" (unten links steht "WSL: Ubuntu-20.04"). Damit bearbeitest du die echten Linux-Dateien.

---

Wenn das Gazebo-Fenster läuft, sag Eli Bescheid und schau ins zweite Dokument (Aufteilung & Implementierung). Sobald Eli das gemeinsame Package mit der `RobotStatus.msg` gepusht hat, machst du `git pull` und kannst loslegen.
