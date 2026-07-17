# LP1：在 Arm Server 上建立完整 ROS 2 開發模擬環境

> 系列：Robotics Networking on Arm: Hands-on Multi-Node ROS 2 with Zenoh —— 本篇是三部曲的第一篇（LP1 環境 → [LP2 分散部署](lp2.md) → [LP3 無線調校](lp3.md)）。全程在 Arm 平台完成，使用官方 prebuilt arm64 image，零架構特殊處理。

## 簡介

本篇在一台 Arm server 上建立 Docker 化的 ROS 2 Jazzy + `rmw_zenoh` 開發模擬環境，並依序完成以下任務：啟動 Neobotix ROX 機器人模擬（Gazebo）、Navigation2 導航堆疊與 RViz；以 talker/listener 驗證 Zenoh router 的 discovery 行為（router 停止後既有 peer-to-peer 通訊不中斷）；透過 RViz 下達導航目標並解讀 costmap；以 `teleop` 與 `/cmd_vel` topic 直接控制機器人底盤，說明 ROS 2 的 topic 介面設計；使用 `top`、`rt_factor`、`iftop` 觀測模擬系統的 CPU 與內部網路開銷；最後啟用 Zenoh shared memory 並量測其對大訊息延遲的影響（實測 point cloud latency 由 9.8ms 降至 6.7ms，約 30%）。

預計耗時約 60 分鐘。完成後具備的環境與概念（router 的 discovery 角色、config 檔案機制、量測工具）為 LP2 與 LP3 的前置基礎。

## 你將學到什麼

* 在 Arm server 上用 Docker 建立完整的 ROS 2 Jazzy + `rmw_zenoh` 開發環境
* 親手驗證 Zenoh router 的 discovery 本質：**停掉 router，通訊照常**
* 啟動工業級機器人模擬：Neobotix ROX + Gazebo + Navigation2 + RViz，下達導航目標
* 用鍵盤與一行指令直接控制機器人 —— 理解「Nav2 沒有特權」的 ROS 介面哲學
* 觀測模擬系統在 Arm server 上的資源足跡與內部流量
* 用 shared memory 優化本機通訊（實測 point cloud latency **降低 30%**）

## 環境需求

| 項目 | 需求 |
|---|---|
| 主機 | Arm server（實測平台：GB10），8 cores / 16GB RAM / 30GB disk 以上 |
| 軟體 | Docker + docker compose |

---

## Step 1 — 啟動 containers

在 Arm server 上建立工作目錄與 compose 檔（不需要 clone 任何 repo —— compose 檔加上 image 就是完整的執行條件）：

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
      - "6080:80"          # 瀏覽器 VNC 桌面
      - "7447:7447/tcp"    # Zenoh router —— 整個系統唯一的對外 port
      - "7447:7447/udp"
    volumes:
      - ./container_volumes/robot_container:/home/ubuntu/container_data
    working_dir: /home/ubuntu
    cap_add:
      - NET_ADMIN          # LP3 的網路調校需要
    shm_size: '640m'       # Zenoh shared memory（Step 8）
    security_opt:
      - seccomp:unconfined
    networks:
      sim_network:
        ipv4_address: 172.1.0.2
    ulimits:
      memlock:             # 預設 8192 KiB 不足以支撐 Zenoh shared memory
        soft: -1
        hard: -1
      rtprio:              # 讓 Zenoh 能為 SHM watchdog 設定執行緒優先權
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

值得注意的三個設定：`shm_size` 與 `memlock` ulimit（Step 8 的 shared memory 靠它們）、以及 `NET_ADMIN`（LP3 調校網路用）。

> 這份 compose 檔與 image 的 build 配方也在 <https://github.com/odincodeshen/ros2-zenoh-arm>。

啟動：

```bash
docker compose pull
docker compose up -d
docker compose ps
```

`container_volumes/` 目錄由 Docker 首次啟動時自動建立，不需要其他東西。

預期結果：`robot` 與 `control` 兩個 container 皆為 `running`（容器名以工作目錄為前綴，即 `ros_zenoh-robot-1` / `ros_zenoh-control-1`）。

瀏覽器開啟 container 桌面（鎖定密碼 `ubuntu`）：

* Robot container：`http://<server_ip>:6080/`
* Control container：`http://<server_ip>:6081/`

| | robot | control |
|---|---|---|
| 角色 | 機器人本體（模擬） | 遠端操控站（LP2 使用） |
| 內部 IP | `172.1.0.2` | `172.1.0.3` |
| VNC port | 6080 | 6081 |
| Zenoh 7447 對外 | 有 | 無 |

> 本篇只使用 robot container；control 留到 LP2。**下指令前先看 terminal 提示符**（`ubuntu@robot` vs `ubuntu@control`）確認位置。

## Step 2 — 設定 Zenoh 設定檔

在 **robot container**（VNC :6080）開 terminal：

```bash
cp /opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5 ~/container_data/ROUTER_CONFIG.json5
cp /opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5 ~/container_data/SESSION_CONFIG.json5
source ~/workshop_env.bash
```

預期結果：顯示兩行環境變數指向剛複製的檔案。

原則：install 目錄下的 DEFAULT 檔是**永不編輯的原廠模板**；`~/container_data/` 下的是工作副本，之後所有設定調整都改它。此後每開一個新 terminal 都要先 `source ~/workshop_env.bash`。

## Step 3 — Router discovery 行為驗證

先理解 `rmw_zenoh` 最核心的架構特性。開三個 terminal（都先 source）：

Terminal 1 — Zenoh router：

```bash
just router
```

預期結果：`Started Zenoh router with id ...`

Terminal 2 / 3 — 發布者與訂閱者：

```bash
ros2 run demo_nodes_cpp talker
```

```bash
ros2 run demo_nodes_cpp listener
```

listener 開始收到 `I heard: [Hello World: N]` 之後，做關鍵動作：**在 Terminal 1 按 `Ctrl+C` 停掉 router** —— 觀察 talker/listener。

結果：**通訊完全不中斷**。這證明 router 的角色是 **discovery（介紹人）**：它把節點的位址互相介紹、節點建立 peer-to-peer 直連後就功成身退，不是所有資料必經的 broker。反過來也成立 —— 先啟動 nodes 再啟動 router 也行（節點會週期性重試連接 router）。

> 這個特性是 LP2/LP3 的架構基礎：機器人內部高頻資料走直連，router 只在「跨越邊界」時承擔轉送與政策 —— 先在這裡親手驗證一次。

驗證完把 router 重新啟動（`just router`），繼續下一步。

## Step 4 — 啟動機器人 stack

Terminal 2/3 的 talker/listener 可以停掉。開兩個新 terminal：

Terminal R2 — ROX simulation（**機器人的身體**：把 ROX 載入 Gazebo，模擬感測器與馬達，發布 `/scan`、`/camera/*`。`no_gui` 關閉 3D 視窗 —— GUI 是獨立 process，關掉它模擬完全一樣、省大量 CPU/GPU）：

```bash
just rox_simu no_gui
```

Terminal R3 — Navigation2（**機器人的大腦**：定位、路徑規劃、速度控制）：

```bash
just rox_nav2
```

預期結果：出現 `Managed nodes are active` 後停止輸出 —— **這是待命不是當機**：Nav2 是目標驅動系統，等待 goal 才會動作。

確認感測資料在流動：

```bash
ros2 topic hz /scan          # 預期 ~8 Hz
ros2 topic list | grep camera
```

## Step 5 — RViz 視覺化與自主導航

新 terminal：

```bash
just rviz_nav2
```

操作：

1. 等待 map / costmap 顯示（淺藍=可通行、紅→深藍漸層=inflation 膨脹層、黑線=牆壁；會跟著機器人移動的小視窗是 local costmap —— 即時感測的「眼前路況」，固定不動的大圖是 global costmap —— 完整「城市地圖」）
2. 工具列 `Nav2 Goal`，在地圖上按住拖曳畫箭頭（位置+朝向）
3. 機器人規劃路徑並移動，右側 Nav2 面板顯示 ETA 與剩餘距離，`Feedback: reached` 即成功

> Goal 只能下在地圖範圍內（黑框內的有色區域）；框外白色是地圖外，planner 找不到路徑。

驗證點：導航成功抵達（實測 `Distance remaining: 0.03m`, `Recoveries: 0`）。

## Step 6 — 手動控制機器人：Nav2 沒有特權

模擬中的 ROX 底盤訂閱標準 topic `/cmd_vel`（線速度+角速度）。**任何往它發布訊息的 process 都是控制者**，以下方式完全等價：

鍵盤遙控（新 terminal，跟著畫面提示用 `i`/`j`/`l`/`k`/`,` 開車，在 RViz 看機器人移動）：

```bash
just teleop
```

或者一行指令讓機器人前進（0.2 m/s，單發）：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

這揭示了 ROS 的核心設計哲學：**Nav2 到頭來也只是另一個 `/cmd_vel` 發布者** —— 它跟你手打的一行 CLI 用同一個介面、沒有任何特權。這個解耦讓「換掉大腦」（自訂演算法、遠端控制、LP2 的跨裝置指令）都是即插即用。

（補充：ROX 上的 UR10 手臂走另一套介面 —— JointTrajectory action。「底盤用 Twist、手臂用 JointTrajectory」是 ROS 生態通用慣例。）

## Step 7 — 觀測資源足跡與內部流量

三個內建觀測工具，讀懂模擬系統在 Arm server 上的真實開銷：

```bash
just top          # = top -c -u ubuntu：只看 workshop 使用者的 process、顯示完整命令列
just rt_factor    # Gazebo real-time factor：模擬時間/真實時間比值，≈1.0 代表算力充足
just iftop_lo     # loopback 介面流量（排除 VNC）：看內部 nodes 互傳的資料量
```

觀察重點：

* `just top`：`gz sim` 多執行緒約吃 2–3 個 core（實測 GB10 整體 CPU 閒置 >80%）—— Arm server 跑整套 stack 綽綽有餘
* `just iftop_lo`：內部 point cloud 流量高達數百 Mbps —— 全部走 loopback TCP。**記住這個數字，下一步要打掉它。**

## Step 8 — Shared Memory：本機通訊優化

預設下同一台機器內的 process 間也走 TCP loopback（上一步看到的流量）。Zenoh 的 shared memory 讓大訊息直接走 `/dev/shm`，序列化一次、零網路棧開銷。

量 baseline（先停掉 R3 的 nav2；latency 量測需要 wall time，該模式下 Nav2 不運作）：

```bash
# R2 重啟為 wall time 模式
just rox_simu use_wall_time:=True no_gui
# 新 terminal 量 point cloud latency
just cam_latency
```

記下平均值後，編輯 `~/container_data/ROUTER_CONFIG.json5` 與 `SESSION_CONFIG.json5`，將 `transport/shared_memory/enabled` 改為 `true`（config 內有多個 `enabled`，認準 `shared_memory` 區塊）。重啟 router 與 simulation，再量一次。

實測結果（GB10）：

| | TCP loopback（預設） | Shared memory |
|---|---|---|
| Mean latency | 9.3–10.2 ms | **6.3–7.4 ms（-30%）** |
| 抖動（Std） | ~1.0 ms | ~0.65 ms |
| `/dev/shm` 用量 | 8 KB | 247 MB |

驗證：`ls /dev/shm` 可見 `.zenoh` 檔案；再跑一次 `just iftop_lo`，loopback 上的大流量消失了 —— 資料改走記憶體。

## 完成檢核

* [ ] 兩個 container running、VNC 可開
* [ ] Router discovery 實驗：停掉 router 後 talker/listener 持續通訊
* [ ] Nav2 goal 自主導航成功
* [ ] teleop / `cmd_vel` 手動控制成功
* [ ] `rt_factor` ≈ 1.0、CPU 有餘裕
* [ ] Shared memory latency -30% 對比完成、loopback 流量消失

## 下一步

[LP2：用 Zenoh 把 ROS 2 系統分散到多台 Arm 裝置](lp2.md) —— 讓 control container 與 Raspberry Pi 連進這台機器人。

> 進階讀者：想自建/客製這個 lab 環境的多架構 image（arm64 原生 CI、供應鏈自主）？見規劃中的獨立篇章「Build your own multi-arch robotics lab image」。
