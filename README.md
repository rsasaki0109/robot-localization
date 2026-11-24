# robo-localization

GNSS / LiDAR / odometry localization methods, compared in a simulated street
canyon, in deterministic Rust with no required dependencies. Companion of
[robo-planning](https://github.com/rsasaki0109/robot-planning) and
[robo-control](https://github.com/rsasaki0109/robot-control); the GNSS, LiDAR
SLAM, LiDAR localization and visual localization stacks this builds on are
included as submodules ([below](#components-submodules)).

GNSS と LiDAR 地図照合をどう組み合わせるか（排他的に切り替える・LiDAR を主に
RTK で補正する・EKF でまとめて融合する・ファクターグラフで重み付けする）を、
同じビル街・同じセンサノイズで比べるリポジトリです。GNSS（libgnss++）、
LiDAR SLAM、LiDAR 地図照合、Visual localization の各スタックはサブモジュールで
入っています。

![Six localizers driving the same route through a crowded street canyon](docs/media/methods_crowd.gif)

*Six localizers on the same drive through a crowded district (600 people).
Each panel follows the robot: buildings shaded by height, people (brown), the
LiDAR scan (cyan), the latest GNSS fix (green RTK fixed, yellow float, red
single), the true track (black) and the estimate's, coloured by what it rests
on. The strip under each panel is the run's source timeline: every colour
change is a change of source. The switching arbiter trades GNSS and LiDAR back
and forth; the LiDAR-primary arbiter takes LiDAR once and keeps it.*

## The district

![The district from above: building heights and the GNSS solution along the route](docs/media/overview.gif)

A 400 m x 300 m business district in miniature (`city`): low-rise blocks in
the west, 90–220 m towers in the middle with a 12 m alley between them,
mid-rise blocks in the east and an open park. The 844 m route starts in the
park, runs down the tower canyon, up the alley, along the low-rise street and
back. Every satellite of a 31-satellite sky (GPS, Galileo, BeiDou and a QZSS
near the zenith) is traced against the buildings each epoch (`gnss`):

* **line of sight**, or **reflected** off a facade behind the antenna (the
  extra path, 2 r cos(el), is a pseudorange bias; only reflections from
  facades within 50 m and above the receiver's C/N0 mask are tracked), or
  **blocked**;
* the single-point error is the weighted least-squares solution's response to
  the biases and the code noise, after the receiver's fault exclusion (the
  worst normalized residual dropped while above 4, up to four times);
* **RTK fixed** with seven or more line-of-sight satellites and at most two
  reflected ones in the solution — and then, with reflected satellites,
  sometimes **wrong**: 0.4–2.5 m off for 3–14 epochs while still reporting
  2 cm; **float** with five or more line-of-sight satellites (decimetres to
  metres); **single** otherwise.

The open park and the south street fix; the tower canyon and the alley give
single-point solutions 15–30 m off. The LiDAR (`lidar`, 180 beams, 40 m) is
ray traced against the footprints and the people, and matched to the map's
facade segments by Huber-weighted point-to-line Gauss–Newton whose search
narrows from 1.5 m to 0.3 m as it converges (so people a metre from a wall
stop pulling it), with the covariance from the inverse information: in a
street between two straight walls it is loose along the street and tight
across it. Wheel odometry has about 1 % scale error and a calibrated MEMS
gyro's bias (about 100 deg/h). The robot drives the route by pure pursuit at
1.2 m/s; every method sees the same drive and the same noise.

## The methods

| Method | GNSS | LiDAR map | How they combine (`methods`) |
|---|:---:|:---:|---|
| GNSS EKF | ✓ | | EKF on odometry; fixes gated at 99 %; after 20 rejected RTK fixed fixes that agree with each other relative to the odometry, the estimate moves onto them |
| LiDAR EKF | | ✓ | EKF on odometry; matches gated against their covariance; a wider gate after 2 s without one |
| switching | ✓ | ✓ | one source in charge: GNSS hands over after 3 unusable epochs, LiDAR takes over when a scan matches, GNSS takes back over after 5 good RTK fixed fixes in a row |
| LiDAR primary | ✓ | ✓ | LiDAR takes charge wherever a scan matches and keeps it; RTK fixed fixes that pass the gate are fused as corrections; GNSS is in charge only where the map cannot place the robot |
| loose EKF | ✓ | ✓ | every gated fix and match into one EKF, nothing switched |
| factor graph | ✓ | ✓ | 8 s sliding window, a node per scan: odometry between nodes, Huber-weighted LiDAR matches with their covariance, Cauchy-weighted GNSS fixes; float and single fixes join only within 3 sigma; for 3 s after an RTK fixed fix carried weight, a match more than 0.3 m from the graph is left out |

## Results (`cargo run --release --example compare -- 5`)

Median over 5 seeds (worst seed in brackets). *Hand-overs* counts changes of
the source in charge, for the two arbiters; the fusing methods do not hand
over (their source split only says which measurements carried weight).

| Method | Condition | RMSE | 95 % | Max | Hand-overs | GNSS / LiDAR / both / dead reckoning |
|---|---|---|---|---|---|---|
| GNSS EKF | clear | 2.95 m (17.20) | 7.00 m | 7.23 m | – | 43 % / 0 % / 0 % / 57 % |
| LiDAR EKF | clear | 0.10 m (0.21) | 0.23 m | 0.56 m | – | 0 % / 92 % / 0 % / 8 % |
| LiDAR EKF | crowd | 0.12 m (0.26) | 0.26 m | 0.66 m | – | 0 % / 90 % / 0 % / 10 % |
| LiDAR EKF | outdated map | 0.17 m (0.27) | 0.35 m | 0.95 m | – | 0 % / 92 % / 0 % / 8 % |
| switching | clear | 0.05 m (0.16) | 0.07 m | 0.11 m | 15 | 44 % / 55 % / 0 % / 1 % |
| switching | crowd | 0.05 m (0.19) | 0.07 m | 0.13 m | 15 | 42 % / 56 % / 0 % / 1 % |
| switching | outdated map | 0.08 m (0.25) | 0.20 m | 0.29 m | 13 | 37 % / 62 % / 0 % / 1 % |
| LiDAR primary | clear | 0.04 m (0.10) | 0.06 m | 0.11 m | 3 | 7 % / 92 % / 0 % / 1 % |
| LiDAR primary | crowd | 0.04 m (0.16) | 0.07 m | 0.12 m | 3 | 8 % / 91 % / 0 % / 1 % |
| LiDAR primary | outdated map | 0.09 m (0.25) | 0.24 m | 0.42 m | 3 | 6 % / 92 % / 0 % / 2 % |
| loose EKF | clear | 0.04 m (0.12) | 0.06 m | 0.11 m | – | 7 % / 38 % / 54 % / 1 % |
| loose EKF | crowd | 0.04 m (0.15) | 0.06 m | 0.12 m | – | 9 % / 38 % / 52 % / 1 % |
| loose EKF | outdated map | 0.09 m (0.23) | 0.23 m | 0.36 m | – | 7 % / 44 % / 48 % / 1 % |
| factor graph | clear | 0.02 m (0.03) | 0.03 m | 0.07 m | – | 8 % / 37 % / 55 % / 0 % |
| factor graph | crowd | 0.02 m (0.04) | 0.03 m | 0.22 m | – | 10 % / 37 % / 53 % / 0 % |
| factor graph | outdated map | 0.08 m (0.08) | 0.18 m | 0.29 m | – | 8 % / 44 % / 48 % / 0 % |

(GNSS EKF is the same in every condition: the crowd and the map do not
affect it.) **Conditions:** *clear*, an empty street and an exact map;
*crowd*, 600 people on the sidewalks, a quarter of them walking within 2.5 m
of the route; *outdated map*, an empty street but a map surveyed a year ago:
every footprint off by about 12 cm, two buildings built since missing from it
and a kiosk and a fence removed since still in it.

What the table says:

* **GNSS alone** is not enough here: 57 % of the drive dead reckons through
  canyons where the fixes are 15–30 m off; its worst seed's RMSE is 17 m.
* **The LiDAR map alone** holds the robot to decimetres, but dead reckons
  across the park (no facade within 40 m) and leans on the map: an outdated
  map nearly doubles its error.
* **Switching** between them is accurate, but hands over 13–15 times a lap:
  every canyon exit sends it back to GNSS, every canyon entry to LiDAR. Each
  hand-over moves the estimate onto the other source's frame, which is what
  looks unsettling in a replay.
* **LiDAR primary** is as accurate with 3 hand-overs a lap (GNSS → LiDAR at
  the park's edge, and back in the park), and RTK fixed fixes still correct
  it wherever they arrive.
* **Fusing** (loose EKF, factor graph) does not hand over at all; the factor
  graph is the most accurate, or tied, everywhere. With the outdated map every LiDAR
  method loses a few centimetres; RTK keeps the fusing methods and LiDAR
  primary within 0.3–0.4 m where the LiDAR-only filter reaches 0.95 m.
* The factor graph's two safeguards came from failures found here: without
  the 3 sigma admission for float fixes, a canyon's float fixes (biased the
  same way for many epochs) dragged the window 14 m; without the RTK hold,
  scan matches sliding along a crowded street pushed the graph 2–3 m away
  from correct RTK fixes, which the Cauchy kernel then discounted.

These numbers are from a simulation in which the LiDAR's map is the world
itself (or a known distortion of it); a real map adds its survey error, and
real reflections, multipath and RTK failures are richer than this model.
The arbiters and the factor graph have full-size counterparts in
[RobotNativeEngine](https://github.com/rsasaki0109/RobotNativeEngine), run
against PLATEAU's 3D city models of Shibuya and Nishi-Shinjuku with
ray-traced GNSS and a 3D LiDAR (example 152): `rne_slam::HybridLocalizer`
switches, with `HybridLocalizerConfig::lidar_keeps_charge` it is LiDAR
primary, and `FactorGraphLocalizer` is the factor graph.

| | |
|:---:|:---:|
| ![clear](docs/media/methods_clear.gif) | ![outdated map](docs/media/methods_outdated_map.gif) |
| clear | outdated map (purple: the map's footprints) |

## Components (submodules)

The methods here take position fixes and scan matches; producing them on a
real robot is the job of these stacks, included under `third_party/`:

| Submodule | What it is | Its role here |
|---|---|---|
| [`gnssplusplus-library`](https://github.com/rsasaki0109/gnssplusplus-library) (libgnss++) | C++20 GNSS toolkit: SPP, RTK, PPP, CLAS/MADOCA, RTCM/UBX, QZSS L6 | the receiver: RTK fixed / float / single solutions (`gnss::Fix`) |
| [`lidar_slam`](https://github.com/rsasaki0109/lidar_slam) | ROS 2 LiDAR SLAM writing Autoware-compatible maps | builds the LiDAR map |
| [`lidar_localization_ros2`](https://github.com/rsasaki0109/lidar_localization_ros2) | map-based 3D LiDAR localization for ROS 2 and Nav2 (NDT, global localization) | the scan-to-map match (`lidar::ScanMatch`) |
| [`visloc-rs`](https://github.com/rsasaki0109/visloc-rs) | SfM, visual-inertial SLAM, 3D Gaussian Splatting and map-based localization in Rust | the camera's answer where LiDAR is not available |

```bash
git clone --recurse-submodules --shallow-submodules https://github.com/rsasaki0109/robot-localization
# or, in an existing clone:
git submodule update --init --depth 1
```

The crate does not build or link them: they are large stacks with their own
build systems (CMake, colcon, Cargo workspaces). Clone without
`--recurse-submodules` to get the methods alone.

## Usage

```bash
cargo run --release --example compare                     # the table, 3 seeds
cargo run --release --example compare -- 10               # 10 seeds
cargo run --release --features render --example gallery   # GIFs -> out/
```

```rust
use robo_localization::{city::City, methods::{Arbiter, Policy}, sim};

let city = City::district(1);
let mut localizer = Arbiter::new(sim::start_pose(&city), Policy::LidarPrimary);
let report = sim::run(&mut localizer, &city, sim::Condition::CROWD, 0, false);
println!("RMSE {:.2} m, {} hand-overs", report.rmse(), report.switches);
```

A new method implements `methods::Localizer` (`predict`, `gnss`, `lidar`,
`pose`, `source`) and joins `sim::all_methods`.

## Features

- default: the methods and the simulation, no dependencies.
- `render`: GIF output through robo-planning's renderer (`tiny-skia`, `gif`).

## Development

```bash
cargo fmt --all
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-features
```

All randomness comes from a seeded `rng::Rng`, so every table and GIF is
reproducible.
