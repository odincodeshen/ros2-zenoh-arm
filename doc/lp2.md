# LP2：用 Zenoh 把 ROS 2 系統分散到多台 Arm 裝置

> 系列：Robotics Networking on Arm: Hands-on Multi-Node ROS 2 with Zenoh —— 三部曲第二篇（[LP1 環境](lp1.md) → LP2 分散部署 → [LP3 無線調校](lp3.md)）。核心賣點：**開發端與部署端同一個 ISA** —— 同一套 arm64 image 從 Arm server 無縫流動到 Raspberry Pi，沒有 cross-compile、沒有架構落差。

## 簡介

本篇說明如何將 LP1 建立的單機系統擴展為跨裝置的分散式系統。內容分為概念與實作兩部分：概念部分說明 Zenoh 的 peer 與 client 模式差異（peer 依賴節點間直接可達，跨容器/跨主機時因預設 loopback listen 而不可行；client 只需一條到 router 的 outbound 連線，由 router 負責雙向轉送）以及 router 拓撲的選擇準則；實作部分依序接入兩個遠端節點 —— control container 修改 `SESSION_CONFIG.json5` 以 client mode 連入並遠端執行 RViz，Raspberry Pi 則以一行 `docker run` 啟動預建的 edge image、透過 `ZENOH_CONFIG_OVERRIDE` 環境變數設定 client 連線（不需修改系統、不需 inbound port、不需 config 檔案）。

跨主機驗證涵蓋 topic 訂閱、雙向 talker/listener 與常見連線錯誤的診斷方法。結尾說明多機器人場景的三種隔離方案（ROS namespace、`ROS_DOMAIN_ID`、Zenoh namespace）及各自的適用條件。預計耗時約 60 分鐘，需額外一台 Raspberry Pi。

## 你將學到什麼

* Zenoh router 的角色與 peer/client 兩種模式的本質差異
* 把第二個節點（container 或實體裝置）以 client mode 接進機器人系統
* 在 Raspberry Pi 上用 Docker 零安裝負擔地跑 ROS 2 Jazzy client
* Router 拓撲的設計原則與多機器人隔離方案

## 前置條件

* 完成 [LP1](lp1.md)：robot container 的 router / rox_simu / rox_nav2 運行中
* Raspberry Pi（16GB SD 卡以上），與 Arm server 同網段
* （選配）第二台邊緣裝置 —— 用於 Step 5 的隔離實作

---

## Step 1 — 核心概念：Router 與 client mode

**Zenoh router 的四個角色**：設定檔入口（啟動時載入一次 → 改 config 必重啟）、內部 peers 的 discovery 服務（介紹人，非轉運站）、client 的資料轉運站（client 的每筆訊息都經它轉送）、流量政策執行點（LP3 的主戰場）。

**為什麼跨裝置用 client mode 而不是 peer**：robot 內部 nodes 只 listen 在 loopback，peer 直連跨不出容器/主機（discovery 看得到 topic、資料卻不通）。Client mode 的語義是「**我只建一條連線到 router，請 router 替我轉送一切**」—— 這條 tunnel 雙向（下行感測資料、上行指令），對 NAT/防火牆友善（只需 outbound）。

**Router 數量跟著「子系統」走，不是跟著「機器」走**：遠端只有少量 nodes → client 直連（本篇做法）；遠端本身是多 node 子系統 → 遠端立自己的 router、兩台 router 互連。

## Step 2 — 接入第二個節點：control container

在 **control container**（VNC :6081）：

```bash
cp /opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5 ~/container_data/SESSION_CONFIG.json5
source ~/workshop_env.bash
nano ~/container_data/SESSION_CONFIG.json5
```

改兩處：`mode: "peer"` → `mode: "client"`；`connect/endpoints` 的 `"tcp/localhost:7447"` → `"tcp/172.1.0.2:7447"`（robot 的內部 IP）。

重置 daemon 並驗證：

```bash
ros2 daemon stop
ros2 topic list
```

預期結果：完整 topic 清單（`/scan`、`/camera/*`、`/map`…）。**只看到 `/parameter_events` 和 `/rosout` 兩項 = 沒連上**（那是每個 ROS 2 process 自帶的），回頭檢查 client 設定。

（選配）遠端視覺化 —— control 上跑 RViz 操控 robot：

```bash
just rviz_nav2
```

## Step 3 — Raspberry Pi 環境建立（Docker 路線）

實測平台：Raspberry Pi 5 + Raspberry Pi OS（Debian trixie）。**不需要重灌 Ubuntu** —— trixie 沒有 ROS 2 Jazzy 官方套件，用 Docker 即可，而且裝置上不留任何永久安裝。

裝 Docker（若尚未安裝）：

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
# 登出重登讓群組生效
```

啟動 edge 容器。這個 image 已內建 `rmw_zenoh`、demo nodes 與標準訊息型別，並且**內建 `RMW_IMPLEMENTATION`**，所以**一行就夠**：

```bash
docker run -d --name pi_edge --net=host \
  -e ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<server_ip>:7447"]' \
  odinlmshen/ros2-zenoh-arm:jazzy-edge \
  sleep infinity
```

`--net=host` 讓 Zenoh client 直接用主機網路。`<server_ip>` 用 Arm server 的 **LAN IP**（host 上 `hostname -I` 查，取 `192.168.x.x`，不是 `172.x` 的 Docker 內部 IP）—— Docker 的 `7447:7447` port mapping 會把連線轉進 robot container 的 router。

進入容器：

```bash
docker exec -it pi_edge bash
source /opt/ros/jazzy/setup.bash
```

預期結果：環境已經備妥，不需要 export 任何東西。

```text
$ echo $RMW_IMPLEMENTATION
rmw_zenoh_cpp
$ echo $ZENOH_CONFIG_OVERRIDE
mode="client";connect/endpoints=["tcp/192.168.0.24:7447"]
```

<details>
<summary>替代方案：從官方 ROS image 自行建立環境</summary>

若偏好從官方 image 起步而非社群 image，等價的做法是：

```bash
docker pull ros:jazzy
docker run -it --net=host --name pi_ros ros:jazzy bash
```

容器內：

```bash
apt update && apt install -y ros-jazzy-rmw-zenoh-cpp ros-jazzy-demo-nodes-cpp ros-jazzy-common-interfaces
source /opt/ros/jazzy/setup.bash   # 安裝後必須重新 source，否則出現 libzenohc.so dlopen 錯誤
```

**每個新 shell 都要執行三行環境設定**：

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<server_ip>:7447"]'
```

與預建 image 的兩個差異：套件裝在容器的 writable layer，刪掉容器就要重裝一次；`apt install` 拿到的是當下的最新版，不同讀者可能跑在不同的二進位上。

日常啟動（**背景啟動 + exec 模式** —— 用 `docker start -ai` 進入的主 shell 一退出整個容器就停了）：

```bash
docker start pi_ros
docker exec -it pi_ros bash
```

</details>

## Step 4 — 跨主機驗證

Pi 容器內：

```bash
ros2 topic list          # 應看到完整清單
ros2 topic hz /scan      # 應收到 ~8 Hz
```

雙向通訊驗證（topic 之外，指令方向也通 —— client tunnel 是雙向的）：

```bash
# Pi 發布，robot container 開 listener 接收
ros2 run demo_nodes_cpp talker
```

錯誤排查速查：

| 症狀 | 原因 |
|---|---|
| `Connection refused` | 封包到了但 port 沒人聽 —— server 端 router 沒啟動（containers 重啟後 stack 要重開） |
| `Name or service not known` | override 裡的占位符沒替換成實際 IP |
| timeout / no route | 網路層不通 —— 檢查同網段、防火牆 |
| topic list 只有 2 項 | client 設定沒生效 —— 檢查三行環境變數、`ros2 daemon stop` |

## Step 5 — 多機器人的隔離方案（概念）

多台 robot 接入同一網路時 topic 名稱會互相衝突（發一個 `/cmd_vel` 所有機器人一起動），三種隔離層次：

| 方案 | 隔離層 | 適合 |
|---|---|---|
| ROS namespace（`/robot_1/...`） | ROS graph | 艦隊管理：一個操控站同時管多台 |
| 不同 `ROS_DOMAIN_ID` | 完全隔離 | 多組實驗共用基礎設施、互不相見 |
| Zenoh namespace（session config） | Zenoh key 層 | 隔離但不動任何 ROS 端設定 |

`ros2 topic list` 顯示的是「你的 session 可見的圖」—— 受 domain、namespace、ACL 共同過濾，不是全知視角。

### 選配 —— 用第二台裝置實際看到衝突並解決

有第二台邊緣裝置的話，可以把上面的問題**做出來**而不只是讀過。在它上面啟動同樣的 edge 容器、指向同一個 router。

**兩台裝置**都跑 talker：

```bash
ros2 run demo_nodes_cpp talker
```

在 **robot** container：

```bash
ros2 node list
ros2 topic info /chatter
```

預期結果：出現**兩個同名的 `/talker`**，`/chatter` 有兩個 publisher。ROS 2 不允許重複的節點名稱 —— 此時 graph 已經有歧義，訂閱端分不出誰是誰。

```text
/talker
/talker
Publisher count: 2
```

隔離它們：只在第二台裝置上，啟動 talker 前設定不同 domain：

```bash
export ROS_DOMAIN_ID=42
ros2 run demo_nodes_cpp talker
```

預期結果：robot container（仍在預設 domain）只看到**一個** `/talker`、一個 publisher —— 第二台的流量對它完全隱形，因為 `rmw_zenoh` 把 domain ID 編進了每一個 key expression。

這就是上表 `ROS_DOMAIN_ID` 那一列的實作，也是多組人共用同一個 Zenoh router 而互不干擾的機制。

## 完成檢核

* [ ] control container client mode 連通、topic list 完整
* [ ] Pi 容器建立、三行環境設定生效
* [ ] Pi 跨主機收到 `/scan`、talker/listener 雙向通
* [ ] 理解 router 拓撲原則與多機器人隔離選項

## 下一步

[LP3：無線網路下的 Zenoh 調校](lp3.md) —— 把 Pi 換到 Wi-Fi 上，直面 810 Mbps 需求 vs 62 Mbps 現實的落差，用四種手段馴服它。
