# LP2 — Distributing a ROS 2 System Across Multiple Arm Devices with Zenoh

## Introduction

The system built in LP1 runs entirely on one machine. Real robot deployments do not: the operator station sits elsewhere, sensor nodes run on separate devices, and the monitoring display is on someone's laptop. This learning path distributes that single-machine system across multiple Arm devices.

Two remote nodes are connected in turn. The `control` container joins over the Docker network and runs RViz to visualise the robot remotely. A Raspberry Pi joins over the physical network using Docker and a single environment variable — no OS reinstallation, no inbound ports, and no configuration files on the device.

The mechanism behind both is Zenoh's client mode, and the reason it is needed rather than peer mode is worth understanding before the hands-on steps — the first section covers it.

Both the server and the Pi run the same arm64 ROS 2 packages. The development machine and the deployment target share one instruction set, so there is no cross-compilation step between them.

Estimated time: 60 minutes. Requires LP1 completed plus one Raspberry Pi.

## What you'll learn

* The four roles of a Zenoh router, and why only one router is needed in this topology
* Why cross-device nodes use client mode, and what peer mode cannot do across containers and hosts
* How to connect a container as a remote client and run RViz against a robot on another machine
* How to bring a Raspberry Pi into the ROS 2 system with Docker and three environment lines
* How to verify both directions of the client tunnel — sensor data down, commands up
* How to diagnose the common cross-device connection failures
* Three ways to isolate multiple robots sharing one network

## Requirements

| Item | Requirement |
|---|---|
| Arm server | LP1 completed; `robot` container running the Zenoh router and ROX simulation |
| Edge device | Raspberry Pi (16 GB SD card or larger), aarch64, same network as the server |
| Software on the edge device | Docker |
| Optional | A second edge device, for the isolation exercise in Step 5 |

---

## Step 1 — Understand the router and client mode

**The Zenoh router has four roles:**

1. **Configuration entry point** — it reads `ROUTER_CONFIG.json5` once at startup. Any configuration change requires a router restart.
2. **Discovery service for local peers** — it introduces nodes to each other, after which they communicate directly. LP1 Step 3 demonstrated this: stopping the router did not interrupt an established conversation.
3. **Relay for client-mode nodes** — a client holds a single connection to the router, and every message it sends or receives passes through that connection.
4. **Traffic policy enforcement point** — compression, access control, downsampling and QoS rules all apply here (the subject of LP3).

**Why cross-device nodes use client mode.** Nodes inside the robot container listen on loopback only. A peer on another container or host learns their addresses through the router, tries to connect directly, and fails — producing a state where `ros2 topic list` shows every topic but no data arrives. A client makes one outbound connection to the router and lets the router relay in both directions, which also suits NAT and firewalled networks since no inbound port is needed.

**How many routers?** Router count follows subsystems, not machines. A remote side running only a few nodes connects them as clients — the approach used here. A remote side that is a multi-node subsystem of its own runs a router locally and links the two routers, so its internal traffic stays local and only cross-system traffic crosses the link.

## Step 2 — Connect the control container

The control container needs its own session configuration, set to client mode and pointed at the robot's router.

In the **control** container (`http://<server_ip>:6081/`, password `ubuntu`):

```bash
cp /opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5 ~/container_data/SESSION_CONFIG.json5
source ~/workshop_env.bash
nano ~/container_data/SESSION_CONFIG.json5
```

Change two fields:

* `mode: "peer"` → `mode: "client"`
* In the `connect/endpoints` list: `"tcp/localhost:7447"` → `"tcp/172.1.0.2:7447"` (the robot container's internal IP)

Both field names also appear in the file's comments — edit the active entries, not the commented examples.

Reset the ROS 2 daemon and verify. The daemon caches the ROS graph, so it must be restarted for the new configuration to take effect:

```bash
ros2 daemon stop
ros2 topic list
```

**Expected result:** the full topic list from the robot, roughly 80 entries including:

```text
/camera/image_raw
/camera/points
/map
/scan
```

If only `/parameter_events` and `/rosout` appear, the client configuration is not active — those two exist in every ROS 2 process and are not coming from the robot. Re-check both fields and run `ros2 daemon stop` again.

Verify that data is actually arriving, not just the graph:

```bash
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
```

**Expected result:** approximately 8 Hz and 11 Hz respectively — the same rates the robot publishes at, since the Docker network is not a bottleneck.

```text
average rate: 7.794
average rate: 11.553
```

Optional — run RViz from the control container to visualise the robot remotely:

```bash
just rviz_nav2
```

> **[Screenshot needed]** — RViz running in the control container, displaying the robot from the neighbouring container.

## Step 3 — Set up the edge device

Verified on a Raspberry Pi 5 running Raspberry Pi OS (Debian trixie). ROS 2 Jazzy has no official apt packages for Debian, so Docker is used — which also means nothing permanent is installed on the device.

Install Docker if the device does not have it:

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
# log out and back in for the group change to take effect
```

Start the edge container. The image ships with `rmw_zenoh`, the demo nodes and the standard message types already installed, and sets `RMW_IMPLEMENTATION` itself, so a single command is enough:

```bash
docker run -d --name pi_edge --net=host \
  -e ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<server_ip>:7447"]' \
  odinlmshen/ros2-zenoh-arm:jazzy-edge \
  sleep infinity
```

`--net=host` lets the Zenoh client use the host's network directly. Use the server's **LAN IP** for `<server_ip>` (run `hostname -I` on the server and take the `192.168.x.x` address — not a `172.x` Docker internal address). The `7447:7447` port mapping forwards the connection into the robot container's router.

Enter the container:

```bash
docker exec -it pi_edge bash
source /opt/ros/jazzy/setup.bash
```

**Expected result:** the environment is already configured — nothing to export.

```text
$ echo $RMW_IMPLEMENTATION
rmw_zenoh_cpp
$ echo $ZENOH_CONFIG_OVERRIDE
mode="client";connect/endpoints=["tcp/192.168.0.24:7447"]
```

<details>
<summary>Alternative: build the environment from the official ROS image</summary>

If you prefer to start from the official ROS image rather than a community one, the equivalent setup is:

```bash
docker pull ros:jazzy
docker run -it --net=host --name pi_ros ros:jazzy bash
```

Inside the container:

```bash
apt update && apt install -y ros-jazzy-rmw-zenoh-cpp ros-jazzy-demo-nodes-cpp ros-jazzy-common-interfaces
source /opt/ros/jazzy/setup.bash
```

The `source` after installation is required — it adds the Zenoh library paths to the environment. Without it, `rmw_zenoh` fails with `libzenohc.so: cannot open shared object file`.

**Run these three lines in every new shell**, including each `docker exec`:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<server_ip>:7447"]'
```

Two differences from the pre-built image: the packages live in the container's writable layer, so removing the container means repeating the installation; and `apt install` resolves to whatever version is current, so different readers may not run identical binaries.

For daily use, start the container in the background and enter it with `exec`:

```bash
docker start pi_ros
docker exec -it pi_ros bash
```

Do not use `docker start -ai`. That attaches to the container's main process; exiting the shell stops the container and every `exec` session inside it.

</details>

## Step 4 — Verify cross-device communication

Confirm network reachability first:

```bash
ping <server_ip>
```

Then, inside the Pi container with the three environment lines applied:

```bash
ros2 topic list
ros2 topic hz /scan
```

**Expected result:** the same topic list the control container sees, and `/scan` arriving at approximately 8 Hz.

```text
average rate: 7.720
```

Now verify the uplink — the client tunnel carries commands as well as sensor data.

On the **Pi**:

```bash
ros2 run demo_nodes_cpp talker
```

In the **robot** container:

```bash
ros2 run demo_nodes_cpp listener
```

**Expected result:** the robot container's listener prints the messages published on the Pi.

```text
[INFO] [listener]: I heard: [Hello World: 28]
```

This confirms both directions of the tunnel: sensor data flows from the robot to the Pi, and messages published on the Pi reach nodes inside the robot. The same path carries `/cmd_vel`, service calls and Nav2 action goals.

Connection troubleshooting:

| Symptom | Cause |
|---|---|
| `Connection refused` | Packets reach the host but nothing listens on the port — the router is not running on the server. After `docker compose up`, the router, simulation and Nav2 must all be started again |
| `Name or service not known` | The `<server_ip>` placeholder was not replaced with the actual address |
| Timeout / no route to host | Network-layer problem — check that both devices are on the same subnet and that port 7447 is not blocked |
| `ros2 topic list` shows only 2 topics | The client configuration is not in effect — re-check the three environment lines and run `ros2 daemon stop` |
| `libzenohc.so` cannot be opened | Only applies to the manual setup — the shell was opened before the packages were installed. Run `source /opt/ros/jazzy/setup.bash` again |
| `docker exec` reports the container is not running | The shell started with `docker start -ai` was closed, stopping the container. Use `docker start` followed by `docker exec` |

## Step 5 — Multi-robot isolation

When several robots share one network, identical topic names collide — a single `/cmd_vel` would drive every robot at once. There are three ways to separate them, at different layers:

| Option | Isolation layer | Applies to |
|---|---|---|
| ROS namespaces (`/robot_1/...`) | ROS graph | Fleet management: one station supervising several robots, all visible at once |
| Separate `ROS_DOMAIN_ID` | Complete separation | Independent groups sharing infrastructure without seeing each other |
| Zenoh namespace (session config) | Zenoh key expressions | Isolation without changing any ROS-side naming |

`rmw_zenoh` encodes the domain ID into its key expressions, and unlike DDS it accepts any value up to `MAX_UINT`. Zenoh namespaces are prefixed to key expressions transparently and stripped on receipt, so they never appear in the ROS graph — but router-side rules that match key expressions must be updated to account for the prefix.

`ros2 topic list` shows the graph visible to your session, filtered by domain, namespace and access control. It is not a global view of the network.

### Optional — observe and resolve a collision with a second device

With a second edge device you can see the problem rather than read about it. Start the same edge container on it, pointed at the same router.

On **both** devices, run a talker:

```bash
ros2 run demo_nodes_cpp talker
```

In the **robot** container:

```bash
ros2 node list
ros2 topic info /chatter
```

**Expected result:** two nodes share the name `/talker`, and `/chatter` reports two publishers. Duplicate node names are not valid in ROS 2 — the graph is now ambiguous, and a subscriber cannot tell the two apart.

```text
/talker
/talker
Publisher count: 2
```

Now isolate them. On the second device only, set a different domain before starting the talker:

```bash
export ROS_DOMAIN_ID=42
ros2 run demo_nodes_cpp talker
```

**Expected result:** the robot container (still on the default domain) sees a single `/talker` and one publisher — the second device's traffic is invisible to it, because `rmw_zenoh` encodes the domain ID into every key expression.

This is the `ROS_DOMAIN_ID` row of the table above, and the same mechanism that keeps several groups from interfering when they share one Zenoh router.

## Verification checklist

* [ ] Control container in client mode; full topic list; `/scan` and `/camera/image_raw` received at source rates
* [ ] Pi container created and started in background mode; three environment lines applied in each shell
* [ ] Pi receives the full topic list and `/scan` over the network
* [ ] Robot container's listener receives messages published from the Pi
