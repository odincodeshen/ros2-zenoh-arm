# ros2-zenoh-arm

ROS 2 Jazzy + Zenoh middleware images and lab environment, built and tested on arm64.

These images back the *Robotics Networking on Arm* learning path series. They are a community effort and are not affiliated with or endorsed by ZettaScale (the Zenoh authors) or Open Robotics.

## Images

| Tag | Contents | Use |
|---|---|---|
| `odinlmshen/ros2-zenoh-arm:jazzy-base` | `ros:jazzy` + `rmw_zenoh` (official apt) + networking tools | Development base — `FROM` it to build your own ROS 2 + Zenoh application |
| `odinlmshen/ros2-zenoh-arm:jazzy-edge` | base + demo nodes + standard interfaces | Edge runtime — deploy a Zenoh client node on any arm64 device |
| `odinlmshen/ros2-zenoh-arm:jazzy-desktop` | VNC desktop + Neobotix ROX simulation + Navigation2 + lab tooling | The learning path environment |

Architecture: **arm64 only**. Verified on a GB10-class Arm server and a Raspberry Pi 5.

Tags follow `jazzy-<variant>` (moving, latest verified build) and `jazzy-<variant>-YYYYMMDD` (immutable). Reference the dated tag when reproducibility matters.

## Quick start

Development base — build your own node on top:

```dockerfile
FROM odinlmshen/ros2-zenoh-arm:jazzy-base
COPY install/ /opt/my_app/
CMD ["ros2", "run", "my_app", "my_node"]
```

Edge runtime — connect a device to a Zenoh router:

```bash
docker run -d --net=host \
  -e ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<router_ip>:7447"]' \
  odinlmshen/ros2-zenoh-arm:jazzy-edge \
  ros2 run demo_nodes_cpp listener
```

Lab environment — two containers (robot + control):

```bash
docker compose up -d
# robot desktop:   http://<host>:6080/   (password: ubuntu)
# control desktop: http://<host>:6081/
```

## What is pinned, what is not

| Component | Policy |
|---|---|
| `rmw_zenoh` and all `ros-jazzy-*` packages | Installed from the official ROS apt repository at build time — release track, floats forward between builds so security fixes are picked up |
| Neobotix ROX simulation (8 repositories) | Pinned to specific commits (`docker/golden_commits.txt`), because they determine the simulation's measurable characteristics. Present in the `desktop` image only |
| Base images (`ros:jazzy` for base/edge, `ros2-desktop-vnc` for desktop) | Pinned by **index digest** — a platform-specific digest would break the build on other architectures. Update deliberately; check the current digest with `docker buildx imagetools inspect <image>:<tag>` |

## Layout

```
docker/
  install-zenoh.sh      shared package layer (rmw_zenoh + tools) used by base and desktop
  Dockerfile.base       ros:jazzy + install-zenoh.sh
  Dockerfile.edge       base + demo nodes
  Dockerfile.desktop    VNC base + install-zenoh.sh + ROX simulation + lab tooling
  files/                justfile, measurement scripts, RViz config (desktop only)
  golden_commits.txt    pinned ROX simulation commits
docker-compose.yaml     the two-container lab environment
.github/workflows/      arm64-native build, smoke tests, publish
```

`base` and `edge` form a single inheritance chain. `desktop` cannot inherit from `base` — it needs the VNC desktop image as its own foundation — so both run the same `install-zenoh.sh` to guarantee an identical Zenoh layer.

## Building locally

On an arm64 host:

```bash
docker build -f docker/Dockerfile.base -t ros2-zenoh-arm:jazzy-base docker/
docker build -f docker/Dockerfile.edge --build-arg BASE_IMAGE=ros2-zenoh-arm:jazzy-base -t ros2-zenoh-arm:jazzy-edge docker/
docker build -f docker/Dockerfile.desktop -t ros2-zenoh-arm:jazzy-desktop docker/
```

The desktop build compiles the ROX workspace from source; expect a few minutes on a multi-core Arm server.

## Acknowledgements

This repository builds on the [ROSCon 2025/2026 Zenoh workshop](https://github.com/ZettaScaleLabs/roscon2025_workshop) by the Zenoh team at [ZettaScale](https://www.zettascale.tech), licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) and the [Eclipse Public License 2.0](https://www.eclipse.org/legal/epl-2.0/). The `justfile`, the measurement scripts (`camera_latency.py`, `rt_factor_avg.py`, `network_limit.sh`), the RViz configuration and the environment script are taken from it unchanged; the Compose file and the desktop Dockerfile are derived from it.

[Zenoh](https://zenoh.io) and [`rmw_zenoh`](https://github.com/ros2/rmw_zenoh) are developed by ZettaScale and Open Robotics. This is a community project and is not affiliated with or endorsed by either.

The Dockerfile is based on [Tiryoh/docker-ros2-desktop-vnc](https://github.com/Tiryoh/docker-ros2-desktop-vnc), licensed under the [Apache License 2.0](https://github.com/Tiryoh/docker-ros2-desktop-vnc/blob/master/LICENSE).

The simulation of the [ROX robot](https://www.neobotix-robots.com/products/mobile-robots/mobile-robot-rox) is courtesy of [Neobotix](https://www.neobotix-robots.com/) and comes from [neobotix/rox](https://github.com/neobotix/rox).
