# LP3 — Tuning Zenoh for Wireless Networks: Compression, Access Control, Downsampling and QoS

## Introduction

A robot that works on the bench often fails on Wi-Fi, and the numbers explain why. The system from LP1 publishes a point cloud of 7.37 MB per frame at 12 Hz — roughly 700 Mbps — plus camera images at about 85 Mbps and laser scans at 0.4 Mbps. Three remote subscriptions therefore ask for around **810 Mbps**. A real 2.4 GHz Wi-Fi link delivers **50–100 Mbps**. Demand exceeds the link by eight to thirteen times.

This learning path closes that gap with four Zenoh traffic policies — compression, access control, downsampling, and priority with congestion control. All four are applied by editing the router's configuration file. No application code changes.

The work is done in two series. Experiment A uses `tc`/`netem` to emulate a degraded wireless link between two containers on the server, which isolates each mechanism under repeatable conditions. Experiment B repeats the measurements with a Raspberry Pi over real Wi-Fi, confirming the conclusions outside the emulator.

Estimated time: 2 hours. Requires LP1 and LP2.

## What you'll learn

* How to build a comparable measurement set for ROS 2 traffic with `ros2 topic hz`, `ros2 topic bw` and `iftop`
* Why the same congested link affects small, medium and large messages in completely different ways
* The mechanism, effect and applicable scenario of each of the four Zenoh traffic policies
* Why a starved large message is a policy problem rather than a bandwidth problem — and what actually fixes it
* Two deployment configurations you can apply directly

## Requirements

| Item | Requirement |
|---|---|
| Arm server | LP1 completed; `robot` container running the router, simulation and Nav2 |
| Remote node | LP2 completed; the `control` container connected in client mode |
| Edge device (Experiment B) | Raspberry Pi on Wi-Fi; the server stays on wired Ethernet |

---

## Part 1 — The measurement method

Three indicators, used identically in every scenario. Without a consistent set, the scenarios cannot be compared.

Run on the **remote side** (control container or Pi) — the stopwatch belongs at the receiver:

```bash
ros2 topic hz /scan                  # small message: rate and jitter (std dev)
ros2 topic hz /camera/image_raw      # medium message
ros2 topic bw /camera/points         # large message: MB/s and message size
```

Run in the **robot** container for total link traffic:

```bash
just iftop_router
```

**Reading the output.** `ros2 topic hz` reports the mean arrival rate and the standard deviation of the interval between messages — the second number matters as much as the first, because it distinguishes a steady stream from a burst-and-stall pattern with the same average. `iftop` shows 2 s / 10 s / 40 s moving averages per connection; read the 10 s column, and note that its units are **bits** (88 MB/s ≈ 700 Mbps). Values of a few hundred **b** mean only Zenoh keepalives are flowing; **Kb** means discovery traffic; **Mb** and above is real sensor data.

Four rules that keep the measurements valid:

1. **`hz` and `bw` statistics are cumulative.** After changing anything — configuration, network shaping, router restart — stop every measurement window with `Ctrl+C` and start it again, then wait 60–90 seconds for the value to converge.
2. **Router configuration is read once at startup.** Restart the router after every configuration change. Compression is negotiated when a session is established, so the subscribing processes must restart too.
3. **Verify before measuring.** Use `grep` to confirm the block you edited is active and not inside a comment, and check the router's startup output for parse errors. If a new scenario produces data identical to the previous one, suspect the configuration first — not the mechanism.
4. **Each subscriber is a separate TCP connection.** They appear individually in `iftop`; map a port to its process with `ss -tnp | grep <port>` followed by `ps -fp <pid>`.

## Part 2 — Experiment A: emulated degradation

### Step 1 — A0: baseline

With the robot stack running (router, `rox_simu no_gui`, `rox_nav2`), measure all three indicators from the control container.

**Expected result:**

| `/scan` | `/camera/image_raw` | `/camera/points` | Link RX |
|---|---|---|---|
| 7.97 Hz (std 0.115 s) | 11.85 Hz (std 0.035 s) | ~88 MB/s (7.37 MB per frame) | ~810 Mb |

These match the rates measured at the source, so the Docker bridge is not a bottleneck — the baseline is a link that is not constraining anything. The 810 Mbps total is the problem statement for everything that follows.

### Step 2 — A1: degrade the link

In the **robot** container:

```bash
just network_limit
ping 172.1.0.3        # expect latency above 20 ms
```

**Expected result:** the script reports the emulated link parameters.

```text
WiFi medium connection simulation applied to 172.1.0.3:
 - Rate: 25mbit
 - Latency: 20ms ± 10ms
 - Packet loss: 0.5%
 - Reordering: 1% 25%
 - Duplicates: 0.1%
 - Corruptions: 0.01%
```

This is not a bandwidth cap alone. The loss and reordering are what drive TCP into congestion behaviour, and they matter more than the rate limit for what follows. 25 Mbps against the 810 Mbps baseline leaves **1/32** of the required capacity.

Restart the measurements and record:

| Scenario | `/scan` | `/camera/image_raw` | `/camera/points` | Link RX |
|---|---|---|---|---|
| A0 | 7.97 Hz | 11.85 Hz | ~88 MB/s | ~810 Mb |
| **A1** | **7.94 Hz — unaffected** | **0.88 Hz** (std 0.577 s, max gap 6.3 s) | **nothing received** | **~23.5 Mb (at the cap)** |

Three messages, three outcomes. The scan is untouched: it is small, and it has a connection of its own. The image loses 93% of its rate and arrives in bursts. The point cloud disappears entirely — each 7.37 MB message exceeds the send timeout and is dropped under the default `drop` congestion policy — yet its connection still consumes about 10 Mb of the link carrying fragments that will never be reassembled.

### Step 3 — A2: compression

Compression must be enabled at both ends of a link. Zenoh uses LZ4, chosen for speed rather than ratio — compression must not become a latency source.

In the **robot** container's `~/container_data/ROUTER_CONFIG.json5`, find `transport/unicast/compression` and set `enabled: true`. Do the same in the **control** container's `~/container_data/SESSION_CONFIG.json5`.

Restart the robot's router, then restart the measurement processes as well — compression is negotiated at session setup, so an existing connection stays uncompressed.

| Scenario | `/scan` | `/camera/image_raw` | `/camera/points` | Link RX |
|---|---|---|---|---|
| A0 | 7.97 Hz | 11.85 Hz | ~88 MB/s | ~810 Mb |
| A1 | 7.94 Hz | 0.88 Hz | nothing | ~23.5 Mb |
| **A2** | **7.95 Hz** (link bytes 380 → ~200 Kb) | **0.5–2.8 Hz, drifting** | **still nothing** (~3.7 MB compressed, still over the timeout) | **~23.5 Mb, still at the cap** |

The scan's rate is unchanged but its bytes on the link roughly halve — LZ4 achieving about 1.8× on this data. The image improves, but the reading drifts between measurements: the point cloud's doomed fragments still compete for the link, and TCP hands out the capacity differently each time. An unstable reading here is the expected result, not a mistake.

Compression widens the pipe. It does not remove a flow that cannot fit through it.

### Step 4 — A3: block the point cloud with access control

Access control is enforced at the router: messages matching a denied key expression are not forwarded. Nodes inside the robot are unaffected, because their traffic never passes through the router.

Add this block at the top level of `~/container_data/ROUTER_CONFIG.json5`:

```json5
access_control: {
  enabled: true,
  default_permission: "allow",
  rules: [
    {
      id: "deny_points_cloud",
      permission: "deny",
      messages: [
        "put", "delete", "declare_subscriber",
        "query", "reply", "declare_queryable",
        "liveliness_token", "liveliness_query", "declare_liveliness_subscriber",
      ],
      flows: ["egress", "ingress"],
      key_exprs: [
        "*/camera/points/**",
        "*/camera/points/**/@adv/**"
      ],
    },
  ],
  subjects: [
    { id: "ALL" },
  ],
  policies: [
    {
      id: "deny_points_cloud_to_all",
      rules: ["deny_points_cloud"],
      subjects: ["ALL"],
    },
  ]
},
```

The `@adv` variant covers the key expressions used by TRANSIENT_LOCAL topics. Restart the router.

| Scenario | `/scan` | `/camera/image_raw` | `/camera/points` | Link RX |
|---|---|---|---|---|
| A0 | 7.97 Hz | 11.85 Hz | ~88 MB/s | ~810 Mb |
| A1 | 7.94 Hz | 0.88 Hz | nothing | ~23.5 Mb |
| A2 | 7.95 Hz | 0.5–2.8 Hz | nothing | ~23.5 Mb |
| **A3** | **7.97 Hz** | **0.84 Hz** (still bursty, max gap 11.4 s) | **blocked — and the connection drops to zero** | **~9.4 Mb — the link is no longer saturated** |

Verify the asymmetry from both sides — this is the point of the exercise:

```bash
# control container: nothing arrives
ros2 topic bw /camera/points

# robot container: unchanged
ros2 topic hz /camera/points
```

**Expected result:** the control container receives nothing while the robot's own nodes still exchange the point cloud at about 12 Hz. The rule governs what crosses the router, not what happens inside the robot.

Note that `/camera/points` still appears in the remote `ros2 topic list` even though no data arrives — the block stops the data, not the graph entry.

With the wasted traffic gone the link finally has headroom, and yet the image is no better. It has the link to itself and still manages only 0.84 Hz in bursts, its connection stuck around 9 Mb. That ceiling is TCP's, not the shaper's: at 0.5% loss with reordering and ~40 ms RTT, a single TCP connection cannot sustain more, which the Mathis formula estimates at the same 8–9 Mbps. Removing the competition exposed the next limit rather than lifting it.

### Step 5 — A4: downsample the image

Downsampling drops publications on the egress path to a target frequency. Keep the access control block in place and add:

```json5
downsampling: [
  {
    messages: ["put", "reply"],
    flows: ["egress"],
    rules: [
      { key_expr: "*/camera/image_raw/**", freq: 3.0 },
      { key_expr: "*/camera/image_raw/**/@adv/**", freq: 3.0 },
    ],
  },
],
```

Restart the router and restart the image measurement.

| Scenario | `/scan` | `/camera/image_raw` | `/camera/points` | Link RX |
|---|---|---|---|---|
| A0 | 7.97 Hz | 11.85 Hz | ~88 MB/s | ~810 Mb |
| A1 | 7.94 Hz | 0.88 Hz | nothing | ~23.5 Mb |
| A2 | 7.95 Hz | 0.5–2.8 Hz | nothing | ~23.5 Mb |
| A3 | 7.97 Hz | 0.84 Hz (bursty) | blocked | ~9.4 Mb |
| **A4** | **8.0 Hz** | **2.6 Hz, std dev 0.06 s** — steady | **blocked** | **~6.8 Mb** |

The robot's own nodes still see the image at its original rate; only the remote side is throttled.

The headline is not the rate but the jitter: 2.6 Hz with a standard deviation of 0.06 s, against A3's 0.84 Hz with 1.47 s. Demand at 3 Hz costs about 6.5 Mb — below the ~9 Mb a single connection can actually sustain — so the transmit queue drains between frames and TCP recovers from each loss in the gap. Supply below capacity turns a congested link into a smooth one. A steady 2.6 fps is worth more than a bursty 0.84 fps average.

### Step 6 — A5: priority and congestion control

The remaining problem is the point cloud, which so far has only ever been blocked, never delivered. This step restores the congestion and changes the policy instead.

Comment out the `access_control` and `downsampling` blocks — keep them in the file, Experiment B needs them again — and add:

```json5
qos: {
  network: [
    {
      interfaces: ["eth0"],
      key_exprs: ["**/map/**", "**/scan/**"],
      messages: ["put", "query"],
      overwrite: { priority: "interactive_high" }
    },
    {
      interfaces: ["eth0"],
      key_exprs: ["**/robot_description/**"],
      messages: ["put", "query"],
      overwrite: { priority: "interactive_low" }
    },
    {
      interfaces: ["eth0"],
      key_exprs: ["**/camera/image_raw/**"],
      messages: ["put"],
      overwrite: { priority: "data_low" }
    },
    {
      interfaces: ["eth0"],
      key_exprs: ["**/camera/points/**"],
      messages: ["put"],
      overwrite: { priority: "background" }
    },
    {
      interfaces: ["eth0"],
      payload_size: "4096..",
      messages: ["put"],
      overwrite: { congestion_control: "block_first" }
    },
  ],
},
```

Verify the state of all three blocks before restarting — a scenario that silently keeps the previous configuration produces data identical to the previous scenario, which is easy to misread as "the mechanism did nothing":

```bash
grep -n "access_control\|downsampling\|qos\|block_first" ~/container_data/ROUTER_CONFIG.json5
```

Restart the router and the measurements.

| Scenario | `/scan` | `/camera/image_raw` | `/camera/points` | Link RX |
|---|---|---|---|---|
| A0 | 7.97 Hz | 11.85 Hz | ~88 MB/s | ~810 Mb |
| A1 | 7.94 Hz | 0.88 Hz | nothing | ~23.5 Mb |
| A2 | 7.95 Hz | 0.5–2.8 Hz | nothing | ~23.5 Mb |
| A3 | 7.97 Hz | 0.84 Hz | blocked | ~9.4 Mb |
| A4 | 8.0 Hz | 2.6 Hz steady | blocked | ~6.8 Mb |
| **A5** | **7.9 Hz** | **4.4 Hz, std dev 0.08 s** | **delivered: 2.0–3.0 MB/s — a complete 7.37 MB frame every ~3 s** | **~17.2 Mb, all of it useful** |

A 7.37 MB message travels as many fragments, and the receiver needs all of them. Under `drop`, congestion discards fragments mid-message: the early ones have already consumed bandwidth, the message is never reassembled, and the publisher immediately starts another one that meets the same fate. That is the "connection busy, subscriber receiving nothing" pattern from A1 and A2. `block_first` makes the head-of-queue message wait until it is fully sent and drops the ones behind it, which converts the same bandwidth into whole messages.

The image benefits without being addressed directly: once the point cloud paces itself, its fragments stop polluting the link, and the image's TCP connection competes in a cleaner environment.

One caveat on priority. Zenoh schedules priorities **within a session**. Each `ros2 topic hz` runs in its own process and therefore its own session, so the scan — already on a private connection — shows no measurable benefit from being prioritised. Seeing priority work requires one process subscribing to several topics at once, which is exactly what RViz does. That measurement is qualitative rather than numerical.

### Step 7 — Restore the network

```bash
just network_normal
```

## Part 3 — Experiment B: real Wi-Fi

The topology changes in exactly one respect: the remote side becomes a physical device on a wireless link. The router and every policy stay in the robot container on the server — the Pi is a client with no infrastructure of its own.

Before starting, return the router configuration to a clean state: comment out `access_control`, `downsampling` and `qos`, and set compression back to `false`. B0 is the untuned baseline; the policies are re-enabled one at a time.

Put the Pi on Wi-Fi (2.4 GHz makes the effects clearer) and connect it as in LP2.

| Scenario | `/scan` | `/camera/image_raw` | `/camera/points` | Link TX |
|---|---|---|---|---|
| **B0** untuned | 7.90 Hz — holds | 4.2 Hz | nothing (wasting ~28 Mb) | ~62 Mb, nearly half of it wasted |
| **B1** + compression | 7.85 Hz | **11.87 Hz — the source rate** | still nothing (wasting ~26 Mb) | ~55.7 Mb |
| **B2** + downsampling | 7.94 Hz | 2.37 Hz steady | still nothing — **its connection grew to 58–61 Mb and it still failed** | ~64 Mb |
| **B3** compression + ACL | 7.85 Hz | 11.89 Hz | blocked, and **wasting nothing** | ~29 Mb, all useful |
| **B4** compression + qos/`block_first` | 7.89 Hz | **11.88 Hz, std dev 0.019 s** | **delivered: ~1 complete frame per second** | ~63 Mb, all useful |

Three findings that the emulator could not have produced:

**Real Wi-Fi is gentler on TCP than `netem` is.** Wi-Fi retransmits at the MAC layer, so most losses never reach TCP — a single connection sustains about 30 Mb here against roughly 9 Mb under the emulator, which delivers its 0.5% loss straight to TCP. The emulator is the harsher, more reproducible case; the real link is more forgiving but jitters more. They are complementary, not redundant.

**Starvation is a policy problem, not a bandwidth problem — demonstrated three times.** The point cloud failed at 26 Mb, failed again after compression halved its size, and failed a third time in B2 when the image's downsampling handed it 58–61 Mb. Only changing the congestion policy fixed it. And the more bandwidth `drop` is given, the more it wastes.

**The shared medium was kinder than expected.** At ~62 Mb of load the scan held throughout, with only a faint trace of airtime contention (longest gap 0.62 s against 0.35 s under the emulator). Higher load or a weaker signal may well change that — an open question rather than a settled result.

**The conclusion.** B0 and B4 move almost the same number of bits — around 62 Mb. B0 delivers a stuttering 4.2 fps image, no point cloud, and wastes nearly half the airtime on fragments that never arrive. B4 delivers a full-rate image, a point cloud frame every second, and an intact scan, with nothing wasted. Tuning did not change the bandwidth. It changed what the bandwidth was worth.

## Part 4 — Deployment configurations

**Remote monitoring only — compression + access control (B3).** Two settings: enable `transport/unicast/compression` at both ends of the link, and deny `*/camera/points/**` (with the `@adv` variant) on the router. The image runs at full rate, the scan is untouched, and the large messages cost nothing. For a remote station that only needs to see, this is the whole answer.

**Large messages required — compression + qos/`block_first` (B4).** Keep compression, and add the `qos/network` block: high priority for the topics that matter, and `congestion_control: "block_first"` for payloads over 4096 bytes. All three streams coexist; the point cloud arrives at whatever pace the link supports, complete rather than shredded.

**When even a compressed image will not fit — add downsampling (A4/B2).** Trading frame rate for stability, with the jitter improvement as the payoff.

## Verification checklist

* [ ] A0 baseline recorded from the control container; the three rates match the source
* [ ] A1 reproduces the three-way split: scan unaffected, image collapsed, point cloud starved
* [ ] A2 shows the scan's link bytes roughly halving with compression
* [ ] A3 shows nothing at the control container while the robot's own nodes still see ~12 Hz
* [ ] A4 converges the remote image to ~3 Hz with an order-of-magnitude drop in jitter
* [ ] A5 delivers complete point cloud frames — the first time any arrive under congestion
* [ ] Network restored with `just network_normal`
* [ ] (Experiment B) the same sequence reproduced over real Wi-Fi from the Pi
