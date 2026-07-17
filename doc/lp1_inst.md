# LP1 — Setting Up a ROS 2 Development and Simulation Environment on an Arm Server

## Introduction

This learning path builds a complete ROS 2 Jazzy development and simulation environment on a single Arm server using Docker, and runs an industrial mobile robot simulation on it: the Neobotix ROX platform in Gazebo, with the Navigation2 stack and RViz.

The middleware layer is `rmw_zenoh` — the Zenoh implementation of the ROS 2 middleware interface, developed jointly by Open Robotics and ZettaScale and distributed through the official ROS apt repository. Everything in this path runs on official arm64 binaries with no architecture-specific modifications.

Beyond bringing the stack up, two exercises examine how the system actually behaves: stopping the Zenoh router while nodes are communicating (which reveals its role as a discovery service rather than a message broker), and enabling shared memory transport to measure its effect on large-message latency.

Estimated time: 60 minutes. Only an Arm server is required — no physical robot.

## What you'll learn

* How to run a Dockerised ROS 2 Jazzy + `rmw_zenoh` environment on an Arm server
* What the Zenoh router does, verified by stopping it mid-communication
* How to launch a full robot simulation stack (Gazebo, Navigation2, RViz) and issue navigation goals
* How the `/cmd_vel` topic interface makes Nav2 one publisher among many, not a privileged component
* How to measure the CPU and internal network cost of the simulation
* How to enable Zenoh shared memory and quantify its latency benefit for large messages

## Requirements

| Item | Requirement |
|---|---|
| Host | Arm server (aarch64), 8 cores / 16 GB RAM / 30 GB free disk |
| Software | Docker and Docker Compose |

---

## Step 1 — Start the containers

Two containers are built from the same image: `robot` simulates the robot itself, and `control` acts as a remote operator station (used in LP2). Each runs a Linux desktop reachable from a browser over VNC.

Create a working directory and the Compose file:

```bash
mkdir -p ros_zenoh && cd ros_zenoh
cat > docker-compose.yaml <<'EOF'
services:
  robot:
    image: ${IMAGE_NAME:-odinlmshen/ros2-zenoh-arm:jazzy-desktop}
    hostname: robot
    stdin_open: true
    tty: true
    ports:
      - "6080:80"          # VNC desktop in the browser
      - "7447:7447/tcp"    # Zenoh router — the only inbound port of the system
      - "7447:7447/udp"
    volumes:
      - ./container_volumes/robot_container:/home/ubuntu/container_data
    working_dir: /home/ubuntu
    cap_add:
      - NET_ADMIN          # required by the network shaping used in LP3
    shm_size: '640m'       # Zenoh shared memory transport (Step 8)
    security_opt:
      - seccomp:unconfined
    networks:
      sim_network:
        ipv4_address: 172.1.0.2
    ulimits:
      memlock:             # the 8192 KiB default is not enough for Zenoh shared memory
        soft: -1
        hard: -1
      rtprio:              # lets Zenoh set thread priority for its SHM watchdog
        soft: 99
        hard: 99
      nice:
        soft: -20
        hard: -20

  control:
    image: ${IMAGE_NAME:-odinlmshen/ros2-zenoh-arm:jazzy-desktop}
    hostname: control
    stdin_open: true
    tty: true
    ports:
      - "6081:80"
    volumes:
      - ./container_volumes/control_container:/home/ubuntu/container_data
    working_dir: /home/ubuntu
    cap_add:
      - NET_ADMIN
    shm_size: '640m'
    security_opt:
      - seccomp:unconfined
    networks:
      sim_network:
        ipv4_address: 172.1.0.3
    ulimits:
      memlock:
        soft: -1
        hard: -1
      rtprio:
        soft: 99
        hard: 99
      nice:
        soft: -20
        hard: -20

networks:
  sim_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.1.0.0/16
EOF
```

The settings worth noting are `shm_size` and the `memlock` ulimit — Zenoh's shared memory transport in Step 8 depends on both — and `NET_ADMIN`, which LP3 uses to shape the network.

> This Compose file and the build recipes for the images are also available at <https://github.com/odincodeshen/ros2-zenoh-arm>.

Start them:

```bash
docker compose pull
docker compose up -d
docker compose ps
```

Docker creates the `container_volumes/` directories on first start; nothing else is needed.

**Expected result:** both containers report `running`. Compose derives their names from the working directory, so they are prefixed `ros_zenoh-`.

```text
NAME                  STATUS
ros_zenoh-control-1   Up
ros_zenoh-robot-1     Up
```

Open the container desktops in a browser (password: `ubuntu`):

* Robot container: `http://<server_ip>:6080/`
* Control container: `http://<server_ip>:6081/`

> **[Screenshot needed]** — the robot container desktop as it appears in the browser after login.

| | robot | control |
|---|---|---|
| Internal IP | `172.1.0.2` | `172.1.0.3` |
| VNC port | 6080 | 6081 |
| Zenoh port 7447 exposed to host | yes | no |

This learning path uses the **robot** container only. Every terminal prompt shown below is `ubuntu@robot` — check the prompt before running commands, as LP2 and LP3 use both containers.

## Step 2 — Copy the Zenoh configuration files

`rmw_zenoh` reads two configuration files: one for the router and one for every ROS process (session). The installed templates should never be edited directly; the convention is to copy them into `~/container_data/` and modify the working copies. `~/container_data/` is a volume shared with the host, so the files can also be edited from outside the container.

In the **robot** container terminal:

```bash
cp /opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5 ~/container_data/ROUTER_CONFIG.json5
cp /opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5 ~/container_data/SESSION_CONFIG.json5
source ~/workshop_env.bash
```

**Expected result:** the environment script detects the new files and exports their paths.

```text
  ZENOH_ROUTER_CONFIG_URI=/home/ubuntu/container_data/ROUTER_CONFIG.json5
  ZENOH_SESSION_CONFIG_URI=/home/ubuntu/container_data/SESSION_CONFIG.json5
```

Run `source ~/workshop_env.bash` in every new terminal from now on.

## Step 3 — Observe the Zenoh router's discovery behaviour

The Zenoh router's primary job is discovery: nodes connect to it on startup, it shares their locators with each other, and the nodes then establish direct peer-to-peer links. This step demonstrates the consequence of that design.

Open three terminals in the robot container and run `source ~/workshop_env.bash` in each.

Terminal 1 — start the router:

```bash
just router
```

**Expected result:**

```text
Started Zenoh router with id 84e303525488529a304c8990ad9bed73
```

Terminal 2 — start the talker:

```bash
ros2 run demo_nodes_cpp talker
```

Terminal 3 — start the listener:

```bash
ros2 run demo_nodes_cpp listener
```

**Expected result:** the listener receives every message the talker publishes.

```text
[INFO] [listener]: I heard: [Hello World: 9]
[INFO] [listener]: I heard: [Hello World: 10]
```

Now press `Ctrl+C` in Terminal 1 to stop the router, and keep watching Terminals 2 and 3.

**Expected result:** message exchange continues without interruption. The talker and listener had already established a direct peer-to-peer connection; the router is not in the data path.

The reverse also holds — nodes may be started before the router, as each node retries the connection periodically.

Restart the router in Terminal 1 before continuing:

```bash
just router
```

## Step 4 — Start the robot simulation and navigation stack

Two components make up the running robot. `rox_simu` loads the Neobotix ROX model into Gazebo and simulates its sensors and motors, publishing `/scan`, `/camera/*` and odometry. `rox_nav2` runs Navigation2 — localisation, path planning and velocity control.

Stop the talker and listener from Step 3. Open two new terminals (`source ~/workshop_env.bash` in each).

Terminal 2 — the simulation, headless:

```bash
just rox_simu no_gui
```

`no_gui` skips the Gazebo 3D viewer. The viewer is a separate process; disabling it leaves the simulation itself unchanged while saving significant CPU.

Terminal 3 — Navigation2:

```bash
just rox_nav2
```

**Expected result:** lifecycle activation completes and output stops.

```text
[lifecycle_manager-9] [INFO] [lifecycle_manager_navigation]: Managed nodes are active
```

The silence that follows is normal. Nav2 is goal-driven: it stays idle until it receives a navigation goal.

Verify that sensor data is flowing:

```bash
ros2 topic hz /scan
ros2 topic list | grep camera
```

**Expected result:** `/scan` arrives at approximately 8 Hz, and the camera topics `/camera/image_raw`, `/camera/points`, `/camera/depth/image_raw` are listed.

## Step 5 — Visualise and navigate with RViz

RViz subscribes to the robot's topics and draws them: the robot model, the map, the costmaps, laser scans and camera images. It also provides tools that publish — including the navigation goal used here.

In a new terminal:

```bash
just rviz_nav2
```

1. Wait for the map and costmap to render. The light blue area is free space; the red-to-blue gradient along the walls is the inflation layer, which raises the cost of paths that pass close to obstacles. The small window that follows the robot is the local costmap (live sensor data); the fixed background is the global costmap (the static map).
2. Select `Nav2 Goal` in the toolbar and drag an arrow on the map — the start point sets the position, the direction sets the orientation.
3. The robot plans a path and drives to it. The Navigation 2 panel on the right reports ETA and remaining distance.

**Expected result:** the Navigation 2 panel reports `Feedback: reached` with a small remaining distance and no recovery behaviours triggered.

```text
Navigation: active
Feedback: reached
Distance remaining: 0.03 m
Recoveries: 0
```

Goals must be placed inside the mapped area — the region covered by the costmap. Points outside it have no planned path, and the robot will not move.

> **[Screenshot needed]** — the RViz window showing the map, costmap and robot model, before issuing a goal.

> **[Screenshot needed]** — the Navigation 2 panel after a successful goal, showing `Feedback: reached`.

## Step 6 — Control the robot directly

The ROX base subscribes to `/cmd_vel` (`geometry_msgs/Twist`: linear and angular velocity). Any process publishing to that topic controls the robot — including Nav2, which is simply another `/cmd_vel` publisher with no special status.

Keyboard teleoperation, using the key bindings printed on screen:

```bash
just teleop
```

> **[Screenshot needed]** — the teleop key binding display, with the robot visible in RViz.

Or publish a single command to move forward at 0.2 m/s:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

Verify the movement through odometry:

```bash
ros2 topic echo /odom --once | grep -A2 position
```

**Expected result:** the `x` position has advanced. Measured on a reference system, a 3-second command at 0.2 m/s moved the robot from x ≈ 0 to x ≈ 0.39 m — less than 0.6 m because of the acceleration ramp.

```text
      x: 0.3899999883542309
```

(The UR10 arm on the ROX uses a different interface — a JointTrajectory action. Publishing `Twist` to the base and `JointTrajectory` to the arm is the standard convention across the ROS ecosystem.)

## Step 7 — Observe resource usage and internal traffic

Three tools show what the simulation costs on this machine and how much data moves between the nodes inside the container.

```bash
just top          # CPU usage per process, workshop user only, with full command lines
just rt_factor    # Gazebo real-time factor: simulated time divided by wall time
just iftop_lo     # traffic on the loopback interface, VNC excluded
```

**Expected results:**

* `just top` — `gz sim` is multi-threaded and uses roughly 2–3 cores. On a 20-core Arm server the machine stays largely idle.
* `just rt_factor` — a value near 1.0 means the simulation keeps up with wall time. Substantially lower values indicate the machine is compute-constrained, usually from other workloads competing for CPU.
* `just iftop_lo` — several hundred Mbps of internal traffic, dominated by the point cloud, all over TCP loopback.

Note the loopback figure from `iftop_lo`. The next step removes most of it.

> **[Screenshot needed]** — `just top` showing `gz sim` and the Nav2 processes.

## Step 8 — Enable shared memory

By default, processes on the same machine still exchange data over TCP loopback — the traffic seen in Step 7. Zenoh's shared memory transport places large messages directly in `/dev/shm`, avoiding the network stack entirely. It is transparent to the application: no code changes, no loaned buffers, and Zenoh falls back to TCP automatically if shared memory is unavailable.

Stop Navigation2 (Terminal 3). Latency measurement requires wall-clock timestamps, and Nav2 does not operate in that mode.

Restart the simulation with wall time and measure the baseline:

```bash
# Terminal 2
just rox_simu use_wall_time:=True no_gui
```

```bash
# new terminal
just cam_latency
```

**Expected result:** point cloud latency around 9–10 ms.

```text
Mean : 9.40 ms | Std : 0.82 ms | Min : 7.91 ms | Max : 12.15 ms
```

Record the mean, then edit both `~/container_data/ROUTER_CONFIG.json5` and `~/container_data/SESSION_CONFIG.json5`. Find the `transport/shared_memory` section and set:

```json5
enabled: true,
```

Both files contain several fields named `enabled` — change only the one inside the `shared_memory` block. Restart the router and the simulation, then run `just cam_latency` again.

**Expected result:** latency drops by roughly 30%, and jitter drops with it.

| | TCP loopback (default) | Shared memory |
|---|---|---|
| Mean latency | 9.3–10.2 ms | 6.3–7.4 ms |
| Std deviation | ~1.0 ms | ~0.65 ms |
| `/dev/shm` usage | 8 KB | 247 MB |

Confirm that shared memory is in use:

```bash
ls /dev/shm        # .zenoh files, one per Zenoh process using shared memory
just iftop_lo      # the large loopback flows are gone — the data now moves through memory
```

## Verification checklist

* [ ] Both containers running; VNC desktops accessible in a browser
* [ ] Talker and listener continue exchanging messages after the router is stopped
* [ ] `just rox_simu no_gui` and `just rox_nav2` start without errors; `Managed nodes are active`
* [ ] Nav2 goal reached in RViz (`Feedback: reached`)
* [ ] Robot moves via `just teleop` or a `/cmd_vel` publication; `/odom` confirms it
* [ ] `just top`, `just rt_factor` and `just iftop_lo` all produce output
* [ ] Shared memory reduces point cloud latency by roughly 30%; `.zenoh` files present in `/dev/shm`
