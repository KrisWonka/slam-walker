# SlamWalker — LiDAR SLAM and Navigation on a Walking Robot

**English** | [中文](README.zh-CN.md)

Off-the-shelf navigation stacks assume wheels. This chassis **walks** — a six-bar linkage with a large
start-up dead-zone, a lurch on every gait cycle, and feet that slip, so its odometry is unreliable.
This repository is the full ROS 2 stack that makes it map and navigate anyway: electrical integration
and Arduino firmware, a calibrated odometry bridge, slam_toolbox + AMCL + Nav2, a frontier explorer
that prices in what turning costs a walking machine, and a custom RViz operator panel.

<p align="center">
  <img src="docs/media/slam_explore.webp" width="840" alt="Left: the walking robot exploring an office on its own. Right: the occupancy map it is building in RViz, with frontiers and the planned path.">
</p>
<p align="center"><sub>Autonomous frontier exploration — nobody is driving. Left: the robot; right: the map it is building over the same 130 s (shown at 6×).</sub></p>

> **Project context** — Undergraduate thesis, Dalian University of Technology, carried out in
> Prof. McCarthy's lab at UC Irvine, Jan – May 2026. The walking platform is the lab's
> build of an existing six-bar design; **everything that makes it navigate is mine**: electrical
> integration, the 20 Hz firmware, parameter identification, the ROS 2 / Nav2 stack, the frontier
> utility and its ablation, and the physical error measurement.
> Write-up with videos: [krisxia.com/slam](https://krisxia.com/slam.html)

## Results

| | |
|---|---|
| **Localization RMSE (AMCL, on the real robot)** | **0.211 m** — mean 0.199 m, 95th percentile 0.311 m, N = 35, encoder odometry + LiDAR only, no IMU |
| **Accumulated heading change** (simulated ablation) | **−44%** — 1899° → 1056° |
| **Path length** (simulated ablation) | **−36%** — 20.97 m → 13.49 m |
| **Map coverage** (simulated ablation) | 75.8% → **82.4%** |

The three ablation rows come from a 2 × 2 study **in simulation** (turn penalty on/off × ideal vs.
walking-mechanism physics, five runs per condition); the numbers above are the walking-physics pair.

## Phase 1 — locomotion-aware frontier exploration

The classic frontier rule (Yamauchi 1997, `explore_lite`) scores a candidate on distance and
information gain. On a wheeled base that is fine — spinning in place is nearly free. On a six-bar walker
it is the most expensive thing you can ask for: it slips, which costs a great deal of extra distance,
and every slip corrupts the odometry that localization depends on. So the utility gets an explicit
turning term:

```
U = γ · information_gain − α · distance − β · |heading change|
```

With β = 0 the explorer reduces to an `explore_lite`-style baseline, which is how the ablation was run.
In the walking-physics simulation, β = 2.5 cut accumulated turning by 44% and path length by 36% while
coverage went **up** — so the robot was not simply giving up early. The weights tuned for the real robot
are in `src/slamwalker_explore/config/explore.yaml`.

Around the utility, `frontier_explorer_node.py` also handles what a real room throws at it: frontier
clustering with a minimum-information-gain cut, snapping each goal to the lowest-cost cell of its
cluster so it never lands in the inflation layer, goal timeouts and no-progress detection, hysteresis
against goal thrashing, and an automatic map save when no reachable frontier remains.

## Phase 2 — navigation on the saved map

<p align="center">
  <img src="docs/media/slam_nav.webp" width="840" alt="Left: the robot walking to a goal. Right: AMCL pose and planned path on the saved map in RViz.">
</p>
<p align="center"><sub>Goal navigation on the saved map with AMCL (sped up). Left: the robot; right: planned path and live pose.</sub></p>

slam_toolbox for graph SLAM, AMCL for localization on the saved map, and Nav2 with a DWB local
controller and a behavior-tree navigator (Smac 2D global planner during exploration, NavFn on the saved
map), re-tuned for a machine whose top speed is **0.10 m/s**.

**The hard part was the model, not the stack.** Textbook differential-drive kinematics did not describe
this machine, so instead of trusting the nominal geometry I measured the four scalars that reconcile the
model with the prototype:

| Parameter | Value | Where |
|---|---|---|
| Encoder ticks per metre | **15 623.1** (1.57 × the geometric value) | `slamwalker_nav.launch.py`, `slamwalker_explore.launch.py` |
| Effective wheelbase | **0.3913 m** | same |
| Right-encoder correction | `right_encoder_scale` | same |
| PWM dead-zone | **120 / 255** — below this the robot does not move | `arduino/test/test.ino` (`PWM_MIN_MOVE`) |

`scripts/calibrate.py` walks through the measurement (drive a known distance → ticks per metre; rotate
a known angle → effective wheelbase) and prints the parameters to paste back in.

## Verification — how wrong is it, in metres?

<img align="right" width="400" src="docs/media/amcl_error_trials.jpg" alt="Five trial trajectories overlaid on the occupancy grid, with seven hand-measured floor marks M1 to M7 shown as white crosses and the AMCL position estimate at each.">

Seven taped marks on the floor, five trials. In each trial the robot is driven through all seven; at each
mark it stops, a logging script records what AMCL believes, and the true (x, y) is measured by hand
off the tape — 5 × 7 = 35 paired samples.

| AMCL position error | |
|---|---|
| Samples | N = 35 |
| Mean | 0.199 m |
| **RMSE** | **0.211 m** |
| 95th percentile | 0.311 m |

**Why no IMU.** Fusing one through an EKF would absorb slip and sharpen heading, and it is the obvious
next step. It was left out on purpose: the point was to find out what the LiDAR scan-matcher and the
calibration are worth on their own. 0.211 m is that number.

<br clear="right">

## Hardware

<table>
  <tr>
    <td width="47%"><img src="docs/media/electronics_deck.jpg" alt="The robot's electronics deck: LiDAR on its mast at the back, the Jetson enclosure on the left, and the stacked driver boards in the middle."></td>
    <td width="53%"><img src="docs/media/wiring.jpg" alt="Wiring diagram: Jetson, USB-to-TTL adapter and LD19 LiDAR at the left, Arduino UNO in the middle, two Cytron SHIELD-MD10 R2 boards to its right, two encoder gearmotors above, a 1000 µF electrolytic on the first driver's power terminals, and the battery at bottom left."></td>
  </tr>
</table>
<p align="center"><sub>Left: The deck as built · Right: Wiring (drawn side by side; on the robot the two drivers are stacked)</sub></p>

| Item | Spec |
|---|---|
| Compute | NVIDIA Jetson Orin Nano 8 GB, JetPack 6.2, ROS 2 Humble |
| MCU | Arduino UNO — 115200 baud USB-CDC, 20 Hz encoder reports, 500 ms command watchdog |
| Motor drivers | 2 × Cytron SHIELD-MD10 R2, stacked, one channel each |
| Motors | 2 × Pololu 37D 70:1 gearmotors with 64 CPR Hall encoders |
| LiDAR | LDRobot LD19, 230400 baud |

**Two lessons from bring-up:**

- **The first driver had to go.** The chassis originally ran on an L298N: a bipolar bridge that drops
  about 2.5 V across itself and is rated 2 A per channel continuous. Two gearmotors pushing a six-bar
  linkage — whose friction throws up current spikes constantly — do not sit under that number, and it
  failed on stall current. It was replaced by the MD10 pair, which is also why the dead-zone fell to
  120/255.
- **The "serial bug" was a brown-out.** Every hard acceleration or reversal dropped the Arduino's USB and
  sent the bridge into a reconnect loop. Motor back-EMF was bouncing the Arduino's ground and pushing its
  USB 5 V out of spec. Fixed with a ground lead from the Arduino straight to pack negative (bypassing
  two stacked shield connectors) and a 1000 µF / 35 V electrolytic across driver #1's power terminals.

## Repository layout

```
src/
  slamwalker_bridge/       serial_bridge_node (cmd_vel ⇄ Arduino, odometry), teleop
  slamwalker_bringup/      launch files, URDF/Xacro, slam_toolbox / Nav2 / RViz configs
  slamwalker_explore/      frontier_explorer, session_manager, scan_resampler
  slamwalker_rviz_panel/   RViz operator panel (C++/Qt)
  ldlidar_ros2/            vendored LDRobot LD06/LD14/LD19 driver
arduino/
  test/test.ino            runtime firmware
  motor_test/, motor_diag/, …   bring-up and diagnostic sketches
scripts/                   calibrate.py (guided), auto_calibrate.py
maps/                      saved maps (my_map: teleop SLAM, auto_map: Phase 1 output)
simulation/                maps and helper scripts from the simulation runs
docs/media/                images and clips for this README
```

**Topic flow.** The LD19 driver publishes `/scan_raw` with a beam count that varies per rotation;
slam_toolbox (Karto) silently drops scans whose count changes, so `scan_resampler` republishes every
scan onto a fixed 360-beam `/scan`. `serial_bridge_node` turns `/cmd_vel` into `V<linear>,<angular>#`
frames and the Arduino's `O<dTickL>,<dTickR>,<dt_ms>` reports into `/odom` and the
`odom → base_footprint` transform.

**Operator panel.** `slamwalker_rviz_panel` adds an RViz dock with an emergency stop, *Start Frontier*,
*Done → switch to Phase 2*, saved-map loading, pose reset, and a global speed-scale slider; the buttons
call `session_manager`'s `/walker/*` services, which tear down one stack and bring up the other.

## Build

On the Jetson (Ubuntu 22.04, ROS 2 Humble):

```bash
sudo apt install -y ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-nav2-rviz-plugins \
  ros-humble-robot-state-publisher ros-humble-xacro ros-humble-rviz2 python3-serial avrdude
git clone https://github.com/KrisWonka/slam-walker.git ~/walker_ws
cd ~/walker_ws
bash src/ldlidar_ros2/scripts/create_udev_rules.sh   # LiDAR → /dev/ldlidar
colcon build --symlink-install
source install/setup.bash
```

The launch files expect the Arduino at `/dev/arduino`; a minimal udev rule:

```udev
# /etc/udev/rules.d/99-arduino.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", SYMLINK+="arduino", MODE="0666", GROUP="dialout"
```

Flash the firmware with `arduino-cli compile --fqbn arduino:avr:uno arduino/test`, then
`bash arduino/upload.sh` (press RESET when prompted).

## Run

```bash
# Phase 1 — explore autonomously; saves maps/auto_map.yaml when no reachable frontier is left
ros2 launch slamwalker_explore slamwalker_explore.launch.py

# Phase 2 — navigate on a saved map (2D Pose Estimate to seed AMCL, then Nav2 Goal)
ros2 launch slamwalker_bringup slamwalker_nav.launch.py map:=$HOME/walker_ws/maps/auto_map.yaml

# Operator panel: run the session manager, then open RViz with the panel config
ros2 run slamwalker_explore session_manager
rviz2 -d src/slamwalker_explore/config/explore.rviz

# Manual mapping instead of Phase 1
ros2 launch slamwalker_bringup slamwalker_slam.launch.py
ros2 run slamwalker_bridge teleop
ros2 run nav2_map_server map_saver_cli -f ~/walker_ws/maps/my_map
```

Useful launch arguments: `serial_port` (default `/dev/arduino`), `lidar_port` (`/dev/ldlidar`),
`lidar_model` (`LDLiDAR_LD19`); for exploration also `start_explorer`.

## Troubleshooting

- **Robot does not move at low speed** — the walking mechanism's dead-zone; confirm with
  `arduino/motor_test/`. Nav2's `min_speed_*` settings keep DWB samples above it.
- **USB drops when the motors reverse** — see the brown-out note above: grounding and bulk capacitance,
  not the serial code.
- **slam_toolbox ignores most scans** — make sure `scan_resampler` is running and slam_toolbox listens
  on `/scan`, not `/scan_raw`.
- **TF has two unconnected trees** — the bridge must publish `odom → base_footprint`, not
  `odom → base_link`.
- **`map_server` complains `yaml_filename` is not initialized** — Nav2's `RewrittenYaml` only fills keys
  that already exist, so the params file needs `map_server` and `amcl` sections.
- **Intermittent LiDAR time-outs** — almost always USB power; plug the LD19 into the Jetson directly or
  use a powered hub.

## Acknowledgements

- LDRobot LDLiDAR ROS 2 driver — <https://github.com/ldrobotSensorTeam/ldlidar_ros2>
- slam_toolbox, Nav2, and the TurtleBot 4 simulation used as a reference while tuning
