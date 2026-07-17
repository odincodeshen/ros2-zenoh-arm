# LP3：無線網路下的 Zenoh 調校 —— 壓縮、ACL、降頻與 QoS

> 系列：Robotics Networking on Arm: Hands-on Multi-Node ROS 2 with Zenoh —— 三部曲第三篇（[LP1 環境](lp1.md) → [LP2 分散部署](lp2.md) → LP3 無線調校）。全部數據為 Arm 平台實測（GB10 + Raspberry Pi 5），完整實驗記錄見 [lp3-wifi-experiment.md](lp3-wifi-experiment.md)、逐步操作稿見 [lp3-wifi-experiment-stepbystep.md](lp3-wifi-experiment-stepbystep.md)。

## 簡介

本篇處理機器人感測資料流與無線網路頻寬之間的落差：實測系統的三個訂閱（point cloud 88 MB/s、影像 85 Mbps、雷射掃描 0.4 Mbps）合計約 810 Mbps，而實測 Wi-Fi 鏈路可用吞吐約 62 Mbps。內容為兩組實驗：實驗 A 以 `tc`/`netem` 模擬劣化網路（25 Mbps + 丟包 + 亂序），逐一套用並量測四種 Zenoh 流量政策 —— compression、access control、downsampling、priority/congestion control —— 隔離各自的機制與效果；實驗 B 在 Raspberry Pi + 真實 Wi-Fi 上重複同一組量測，驗證結論並對照模擬與真實環境的差異。所有政策僅修改 router 的設定檔，應用程式碼不變。

主要量測結果：compression 使遠端影像由 4.2 Hz 恢復至源頭速率 11.9 Hz；大訊息在 drop 壅塞策略下於三種頻寬條件（26/26/58 Mbps）均無法送達，改用 `block_first` 後以約 1 Hz 完整送達；單條 TCP 連線在 0.5% 丟包鏈路上的吞吐上限（~9 Mbps）與 Mathis 公式估算一致。文末整理兩種部署組態（影像監控場景：compression + ACL；需要大訊息場景：compression + qos/block_first）及其適用條件。前置條件為 LP1 與 LP2，預計耗時約 2 小時。

## 問題陳述

機器人的感測資料流（實測）：`/camera/points` 每幀 7.37MB @ 12Hz ≈ 700Mbps、`/camera/image_raw` ~85Mbps、`/scan` ~0.4Mbps —— **三個訂閱合計 ~810 Mbps**。而真實 2.4GHz Wi-Fi 實際可用約 50–100 Mbps：**需求是鏈路的 8–13 倍**。本篇用四種 Zenoh 手段（全部只改 router config，應用程式碼零修改）解決這個落差。

## 你將學到什麼

* 用 `ros2 topic hz/bw` + `iftop` 建立可對比的網路量測方法
* 四種調校手段的機制、實測效果與適用場景：Compression / Access Control / Downsampling / Priority+Congestion Control
* 兩個可直接抄的部署配方
* 超出教科書的實戰洞見：TCP 丟包天花板、孤兒分片白耗、tc 模擬與真實 Wi-Fi 的差異

## 前置條件

* [LP1](lp1.md)：Arm server 上 robot stack（router / rox_simu no_gui / rox_nav2）運行中
* [LP2](lp2.md)：control container（client mode）與 Raspberry Pi（Docker + 環境變數）都能連入
* Pi 用 Wi-Fi 連線（實驗 B 用）；Arm server 保持有線

---

## Part 1：量測方法

三個指標，所有情境用同一組才能對比：

```bash
# 遠端（control 或 Pi）執行 —— 碼表裝在接收端
ros2 topic hz /scan                  # 小訊息：速率與抖動（std dev）
ros2 topic hz /camera/image_raw      # 中訊息
ros2 topic bw /camera/points         # 大訊息：MB/s 與訊息大小
# robot container 執行 —— 鏈路總流量
just iftop_router                    # 讀 10 秒欄；單位是 bits；Mb 級=資料、Kb 級=控制、b 級=keepalive
```

操作紀律（實測踩坑總結）：

1. **hz/bw 統計是累計的** —— 每次切換情境，所有量測視窗 `Ctrl+C` 重啟歸零，等 60–90 秒讀收斂值
2. **改 config 必重啟 router**（設定只在啟動時載入）；壓縮等建線協商的能力，**訂閱端 process 也要重啟**
3. **改完必驗證**：`grep` 特徵字串確認區塊生效；數據跟上一情境一樣 → 先懷疑設定沒生效
4. 每個訂閱者是**獨立 TCP 連線**，iftop 上可逐一辨識（`ss -tnp` + `ps -fp` 對應 port 與 process）

## Part 2：實驗 A —— tc 模擬劣化網路（機制隔離）

用 `just network_limit` 在 robot 出口模擬中等品質 2.4GHz Wi-Fi：**25mbit + 20ms±10ms + 0.5% loss + reordering** —— 注意丟包與亂序才是逼出 TCP 壅塞行為的主角，不只是限速。

### A 系列完整數據（GB10 實測）

| 情境 | `/scan` hz | `/camera/image_raw` hz | `/camera/points` | 鏈路 RX |
|---|---|---|---|---|
| A0 正常網路 | 7.97 Hz | 11.85 Hz | 88 MB/s | 810 Mb |
| A1 劣化 25Mbps | 7.94 Hz — 無感 | 0.88 Hz 爆發式（-93%） | **餓死** + 白耗 10Mb | 23.5 Mb 貼死 |
| A2 +壓縮 | 7.95 Hz（位元組砍半） | 0.5–2.8 Hz 漂移 | 餓死 + 白耗 | 23.5 Mb 貼死 |
| A3 +ACL | 7.97 Hz | 0.84 Hz（TCP 天花板現形） | 擋下（robot 內部照常 12Hz） | **9.4 Mb 解壓** |
| A4 +降頻 3Hz | 8.0 Hz | **2.6 Hz 穩定（抖動 -24x）** | 擋下 | 6.8 Mb |
| A5 qos/block_first | 7.9 Hz | 4.4 Hz 穩定 | **復活 ~0.34 Hz** | 17.2 Mb 全有效 |

### A 系列四大洞見

1. **同一條爛鏈路，三種尺寸訊息命運分化**：小訊息無感（獨立連線+需求小）、中訊息重傷、大訊息全滅 —— 每個手段各救各的對象
2. **TCP 單連線丟包天花板 ~9Mb**（0.5% loss + reorder + 40ms RTT 的 Mathis 公式理論值，實測吻合）：shaper 給 25Mb 不代表單一流拿得到 —— Zenoh 支援 QUIC 的實證動機（LP4 主題）
3. **孤兒分片白耗**：drop 策略下 7.37MB 訊息的分片中途被丟 → 湊不齊卻已耗頻寬 —— 「連線有流量 ≠ 訂閱者有收到」，iftop 與 topic bw 必須對照
4. **調校本質 = 把供給壓到運力之下**：低於天花板後傳輸從壅塞模式（爆發+長空窗）切換到順暢模式（規律間隔）—— 體驗品質看抖動，不只看平均

各步驟的 config 區塊與詳細操作見 [stepbystep 的 Step 6–11](lp3-wifi-experiment-stepbystep.md)。

## Part 3：實驗 B —— Raspberry Pi 真實 Wi-Fi（端到端驗證）

拓撲：跟實驗 A 唯一的差別是「遠端」換成真實硬體 + 真實無線 —— Pi 扮演遠端操控站（零基礎設施），router 與所有政策仍在 Arm server 的 robot container 上。

### B 系列完整數據（GB10 + Pi 5 實測）

| 情境 | `/scan` hz | `/camera/image_raw` hz | `/camera/points`（Pi 端） | 鏈路 TX |
|---|---|---|---|---|
| B0 無調校 | 7.90 Hz — 守住 | 4.2 Hz | 零 + 白耗 28Mb | 62 Mb（近半白耗） |
| B1 +壓縮 | 7.85 Hz | **11.87 Hz 源頭滿速（std 0.024s）** | 零 + 白耗 26Mb | 55.7 Mb |
| B2 +降頻 | 7.94 Hz | 2.37 Hz 穩定 | 零 —— **白耗暴漲至 58–61Mb 依然全滅** | 64 Mb |
| B3 壓縮+ACL | 7.85 Hz | 11.89 Hz 滿速 | 零且**零白耗**（政策擋下） | 29 Mb 全有效 |
| B4 壓縮+qos/block_first | 7.89 Hz | **11.88 Hz（std 0.019s，全系列最穩）** | **復活：每秒 1.02 幀完整點雲** | 63 Mb 全有效 |

### B 系列三大洞見

1. **真實 Wi-Fi 與 tc 模擬互補**：Wi-Fi 的 MAC 層重傳（L2 ARQ）把丟包藏起來，TCP 單連線跑到 ~30Mb（tc 下只有 ~9Mb）—— tc 對 TCP 更殘酷（可重現的最壞情況），真實 Wi-Fi 更寬容但延遲更抖
2. **「餓死是政策問題非頻寬問題」三發全中**：26Mb 餓死、壓縮減半照樣餓死、頻寬翻三倍（58–61Mb）還是餓死 —— 只有換策略（block_first）能救，且頻寬越大 drop 白耗越多
3. **共享媒介效應比預測溫和**：~62Mb 負載下 scan 全程穩住，airtime 競爭只留輕微指紋（更高負載下是開放問題）

### 結論：同樣的 airtime，完全不同的價值

同樣 ~62–63Mb 的 Wi-Fi 流量 —— **B0**：4.2fps 影像 + 零點雲 + 近半白耗；**B4**：11.9fps 滿速影像 + 每秒一幀點雲 + 完整 scan + 零浪費。**調校改變的不是頻寬，是頻寬的價值。**

## Part 4：可直接抄的部署配方

**配方一：遠端只需要影像監控 → 壓縮 + ACL（= B3）**

```json5
// ROUTER_CONFIG.json5（robot 端），兩處設定：
// 1. transport/unicast/compression/enabled: true（遠端 session 也要開）
// 2. access_control 區塊 deny */camera/points/**（含 @adv key）
```

效果：影像滿速、scan 無損、大訊息零浪費。

**配方二：遠端也需要點雲 → 壓縮 + qos/block_first（= B4）**

```json5
// 1. compression 同上
// 2. qos/network 區塊：關鍵 topic 高 priority + payload_size "4096.." 改用 congestion_control: "block_first"
```

效果：三流共存 —— 影像滿速、點雲以鏈路能承受的節奏完整送達、scan 無損。

**配方三（更惡劣頻寬）：再加 downsampling** —— 犧牲影像幀率換穩定（A4 證明抖動改善 24 倍）。

完整 config 區塊見 [stepbystep](lp3-wifi-experiment-stepbystep.md) Step 7–10；Troubleshooting 速查表同文件文末。

## 延伸方向

* **LP4（規劃中）**：mTLS + QUIC —— 本篇的 TCP 天花板數據就是 QUIC 的動機；QUIC 多 stream 天然消除 priority 間的 head-of-line blocking
* 更高負載 / 更差訊號下的共享媒介效應
* 單一 session 多 topic（RViz）場景下的 priority 質性驗證
