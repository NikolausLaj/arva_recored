# ARVA Real-World Test Protocol

`arva_test_protocol` prepares reproducible flight-test directories and records
existing ROS 2 topics. It does not arm a vehicle, select a PX4 mode, publish
flight setpoints, or bypass PX4 and RC safety mechanisms.

See [the operator protocol](docs/real_world_test_protocol.md) for the ordered
field procedure for all seven tests.

Each prepared run has this layout:

```text
tests/real_world/<test_type>/run_<id>/
  bag/
  config.yaml
  metadata.yaml
  parameters/
  qos_overrides.yaml
  recording_topics.txt
```

The package currently provides templates for ARVA characterization, heading
controller, terrain following, waypoint following, fluxline following, fine
search, and complete-search experiments. Fill in the geometry and operational
values in a copy of the relevant template before preparing a flight.

## Workflow

Build and source the ROS workspace in the project's ROS/Docker environment:

```sh
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
colcon build --packages-up-to arva_test_protocol
source install/setup.bash
```

Source the PX4 dependency workspace before rebuilding so the generated workshop
setup retains access to `px4_ros2_cpp` and the other preinstalled dependencies.

## SITL Data Collection

For a single heading-controller response, use the shortcut after building and
sourcing the package (MCAP storage support must be installed; see below):

```sh
ros2 run arva_test_protocol arva_test_heading
```

Start the normal SITL stack separately. The shortcut prepares the next numbered
run under `tests/sitl/heading_controller/`, snapshots the configured parameter
files, and starts MCAP recording. Perform one heading response after recording
begins, then press `Ctrl-C`. The command prints the bag location and the analysis
command for that run. No YAML edits, manual run numbers, or event markers are
needed. Stopping the recorder leaves the outcome undeclared; use
`arva_test_finish` if you want to record `completed`, `failed`, or `aborted`.

Analysis uses the existing `fluxline_follow` mode interval. Use one response per
quick run; for multiple heading steps in one recording, use the explicit segment
and event-marker workflow below. The quick run does not declare planned heading
offsets; document the actual offset in the run configuration if needed. The copied
parameter files contain saved settings; runtime parameter overrides still need
to be captured separately.

The repository is detected from the current directory or the standard workshop
path. For another checkout, pass `--workspace-root /path/to/roscon-25-workshop`.
Optional flags include `--operator "Your Name"`, `--run-id 003`, and `--dry-run`
(prepares a run without recording).

For a terrain-following trial at a constant target, use:

```sh
ros2 run arva_test_protocol arva_test_terrain
```

The shortcut prepares the next numbered run under `tests/sitl/terrain_follow/`
(`run_001`, `run_002`, ...), copies
`px4_roscon_25/arva_master/cfg/terrain_follow.yaml` into
`parameters/terrain_follow.yaml`, and starts MCAP recording in `bag/`.
The analysis target defaults to `target_distance` in that saved controller YAML;
no live parameter query is required. Start the normal stack separately. Perform the terrain
trial, then press `Ctrl-C`; the command prints the bag location and analysis
command. No YAML edits or event markers are needed. The outcome remains
undeclared until you use `arva_test_finish`.

Analysis covers the entire recording using the target captured at startup.
Start recording near the beginning of the intended trial and keep the target
constant during each recording. The shortcut records an operator-performed
trial; it does not execute the template's step sequence or change a flight mode.
For multiple target steps in one bag, use the explicit segment and marker
workflow below. Roll/pitch range compensation is not applied by this analyzer;
its measurement reference remains PX4 `dist_bottom`.

Use `--live-parameters` to read the target from the running
`/terrain_follow_controller` and also save `parameters/terrain_follow_runtime.yaml`.
Alternatively, use `--target-agl 2.0` to supply the analysis reference explicitly.
These options do not change the controller target; ensure the analysis reference
matches the controller. For an offline preview, use `--dry-run` (creates a run
and snapshots the YAML without recording). The terrain shortcut also supports
`--workspace-root`, `--operator`, and `--run-id`.

For a waypoint-following trial, use:

```sh
ros2 run arva_test_protocol arva_test_waypoint
```

The shortcut creates the next numbered folder under `tests/sitl/waypoint_follow/`
and records MCAP in `bag/`. In the standard workshop checkout this is
`/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop/tests/sitl/waypoint_follow/run_001/`.
Each run snapshots the waypoint, terrain, and mode-executor parameter YAML files.
The planned analysis route comes from `waypoints_ned` in `mode_executor.yaml`,
or from the fallback route in `WaypointFollowMode.cpp` when that parameter is empty.
Rebuild `arva_master` after changing its saved configuration or fallback route so
the running stack matches the captured route.

Start the normal SITL stack separately, start recording, perform one waypoint
trial, then press `Ctrl-C`. Analysis uses the recorded `waypoint_follow` mode
interval; no event markers are required. The command prints the bag location
and analysis command. Use `arva_test_finish` to declare the outcome separately.
The shortcut also supports `--workspace-root`, `--operator`, `--run-id`, and
`--dry-run`. If you set a runtime route, pass the same flattened north/east pairs
with `--waypoints-ned 0 0 0 20 10 20`; this supplies the analysis reference only.

Keep simulated runs separate from future physical-flight data. In the workshop
container, save them under the mounted repository so they are also available on
the host. The mount configuration does not need to change. Use the repository
root to resolve parameter snapshot paths. Increment `--run-id` for each repeat:

```sh
export REPO_ROOT="$HOME/roscon-25-workshop_ws/src/roscon-25-workshop"

ros2 run arva_test_protocol arva_test_prepare \
  --config "$REPO_ROOT/px4_roscon_25/arva_test_protocol/config/terrain_follow.yaml" \
  --workspace-root "$REPO_ROOT" \
  --output-root "$REPO_ROOT/tests/sitl" \
  --run-id 001 --operator "operator name" \
  --software-revision "sitl-build-or-source-hash"
```

Set `REPO_ROOT` in each terminal that uses these commands. These absolute paths
work regardless of the terminal's current directory. On the host, the runs
appear under `roscon-25-workshop/tests/sitl/`.

The supplied revision is saved as `software_commit` in `metadata.yaml`; if
omitted, the tool tries to read the Git commit. Fill in the copied
`config.yaml` before recording. The stock templates deliberately leave
experiment-specific geometry and intervals empty, and the recorder refuses to
start until required analysis inputs are present:

- ARVA characterization: surveyed samples, expected distance/angle, and sample
  time boundaries.
- Heading and terrain: step/segment definitions with event names or time offsets;
  terrain segments also need commanded AGL targets.
- Waypoint following: at least two planned local-NED XY waypoint pairs.
- Fluxline and fine search: surveyed transmitter XY position in the run config or
  metadata.
- Full search: surveyed transmitter position and at least two coverage waypoints.

Start the normal SITL stack and production ROS nodes separately, then record and
analyze the prepared run:

```sh
ros2 run arva_test_protocol arva_test_record \
  --run-dir "$REPO_ROOT/tests/sitl/terrain_follow/run_001" --confirm
```

After the SITL trial ends, stop the recorder with `Ctrl-C`, then store the
operator-declared outcome and analyze the run:

```sh
ros2 run arva_test_protocol arva_test_finish \
  --run-dir "$REPO_ROOT/tests/sitl/terrain_follow/run_001" --outcome completed

ros2 run arva_test_protocol arva_test_analyze \
  --run-dir "$REPO_ROOT/tests/sitl/terrain_follow/run_001"
```

After analyzing the repeated runs for a test type, aggregate their metrics:

```sh
ros2 run arva_test_protocol arva_test_aggregate \
  --test-dir "$REPO_ROOT/tests/sitl/terrain_follow"
```

Preserve the simulator/PX4 version, world, seed, coordinate origin, and actual
runtime parameter values with each run. The prepared directory snapshots the
parameter YAML files listed by the test configuration; if SITL applies runtime
overrides, also save `ros2 param dump` output in that run's `parameters/`
directory. The test tools do not start SITL or activate a flight mode.

Prepare a run. This copies the configuration and current controller parameter
files into an immutable run directory, writes metadata, and does not record or
command the vehicle.

```sh
ros2 run arva_test_protocol arva_test_prepare \
  --config "$PWD/src/roscon-25-workshop/px4_roscon_25/arva_test_protocol/config/terrain_follow.yaml" \
  --workspace-root "$PWD/src/roscon-25-workshop" \
  --operator "operator name"
```

Supply per-run surveyed information with a metadata file when needed:

```yaml
UAV_configuration: x500 with ARVA receiver revision 1
flight_height: 1.25
flight_velocity: 0.75
transmitter_position:
  frame: PX4 local NED
  meters: [12.4, -3.1, 0.0]
transmitter_heading: 0.0
starting_position:
  frame: PX4 local NED
  meters: [0.0, 0.0, -1.25]
starting_heading: 1.57
search_area:
  frame: PX4 local NED
  polygon_meters: []
```

Pass it through `--metadata-file <path>`. Add only the fields that apply to the
run.

Start the production ROS nodes and perform normal pre-flight checks separately.
Then deliberately start recording for the prepared directory. The first command
only displays its waiting state; the second requires an explicit confirmation.

```sh
ros2 run arva_test_protocol arva_test_record \
  --run-dir tests/real_world/terrain_follow/run_001

ros2 run arva_test_protocol arva_test_record \
  --run-dir tests/real_world/terrain_follow/run_001 --confirm
```

The recorder prints `recording started` and `test active`. End the recording
with `Ctrl-C` after the operator has completed or aborted the physical test.
Then record the operator-declared result:

```sh
ros2 run arva_test_protocol arva_test_finish \
  --run-dir tests/real_world/terrain_follow/run_001 \
  --outcome completed
```

Valid outcomes are `completed`, `failed`, and `aborted`. A recorded bag does not
itself imply a successful flight or localization; acceptance is determined by
the later analysis and the test-specific criteria.

The recorder subscribes to configured topics even when a test-only publisher is
not active yet, so event markers and mode-specific telemetry are captured once
they appear.

For configured PX4 `/fmu/out/` topics, recording explicitly uses best-effort,
volatile QoS. The recorder saves these profiles in `qos_overrides.yaml` and
passes them to rosbag2, avoiding reliable subscriptions that cannot receive
PX4 sensor data when publishers appear after recording starts. Other topics
keep rosbag2's automatic QoS selection. Bottom distance is the `dist_bottom`
field of `/fmu/out/vehicle_local_position`, with validity in `dist_bottom_valid`;
it is not a separate topic. Recordings with zero messages for that topic must
be repeated to capture the missing data.

Use explicit event markers to timestamp test phases or manual parameter changes
while recording. For example, mark a terrain step immediately after the operator
changes the target AGL:

```sh
ros2 run arva_test_protocol arva_test_mark \
  --run-dir tests/real_world/terrain_follow/run_001 \
  --event terrain_step_1 --confirm
```

Markers publish only `/arva_test_protocol/event`; they do not set a parameter,
change a mode, or publish a flight setpoint. Each marker is republished briefly
to allow ROS discovery to connect it to the active recorder.

## Direct MCAP Recording

For Foxglove playback with custom PIEPS and PX4 message definitions, record
directly to MCAP. Install the storage plugin once in the container:

```sh
sudo apt-get install ros-humble-rosbag2-storage-mcap
```

If the container user has no sudo password, run these commands from a terminal
on the Docker host, replacing `<container>` with the name or ID from `docker ps`:

```sh
docker exec --user root <container> apt-get update
docker exec --user root <container> apt-get install -y ros-humble-rosbag2-storage-mcap
```

The repository's Docker dependency script includes this plugin for future image
builds. Installing it in an existing container takes effect immediately.

Then select MCAP when recording a new prepared run (with populated analysis
intervals):

```sh
ros2 run arva_test_protocol arva_test_record \
  --run-dir "$HOME/roscon-25-workshop_ws/src/roscon-25-workshop/tests/sitl/heading_controller/run_002" \
  --storage-id mcap --confirm
```

Stop recording with `Ctrl-C` and wait for `recording stopped`. Open the `.mcap`
file in the run's `bag/` directory from Foxglove on the host. No conversion is
needed. Finishing, analysis, and aggregation use the same commands for both
formats; analysis detects the format from the bag metadata. The default storage
format remains SQLite (`sqlite3`) when `--storage-id` is omitted.

## Analysis

Run analysis after recording has stopped. Analysis only reads the bag and writes
outputs beneath the existing run directory.

```sh
ros2 run arva_test_protocol arva_test_analyze \
  --run-dir tests/real_world/terrain_follow/run_001
```

The analyzers are:

- `arva_sensor_characterization`: per-sample distance and heading errors, valid
  measurement rates, CSV output, YAML summary, and an optional plot.
- `heading_controller`: ARVA heading offset, measured UAV heading, commanded yaw
  rate, settling time, overshoot, oscillation crossings, and steady-state error.
- `terrain_follow`: per-segment AGL tracking RMSE, maximum and mean absolute
  errors, steady-state error, optional step-response metrics, CSV output, YAML
  summary, and an optional plot.
- `waypoint_follow`: planned-versus-flown XY trajectory, cross-track RMSE,
  maximum and mean position error, path length, and turn-neighborhood error.
- `fluxline_follow`: transmitter-relative trajectory, convergence time, path
  length, transition distance, success, and ARVA loss/recovery counts.
- `fine_search`: XY estimate, XY localization error, search duration, success,
  valid-distance rate, and a transmitter/trajectory plot.
- `full_search`: coverage, fluxline, and fine-search durations; ARVA detection
  timing; Mode Executor transition sequence; final localization; success; and
  phase-colored trajectory and timeline plots.

Before preparing a run, populate the relevant template fields:

- ARVA: `analysis.samples` with surveyed geometry and expected distance and
  angle values. Use either `start_offset_s`/`end_offset_s` or
  `start_event`/`end_event` marker names.
- Terrain: `analysis.segments` with AGL targets and optional initial AGL for
  step metrics. Segments accept the same offset or event-marker boundaries.
- Waypoint/full search: planned local-NED waypoint pairs.
- Fluxline/fine/full search: a surveyed transmitter local-NED position and
  representative start conditions.

The analysis configuration is copied into the run directory during preparation,
so each run remains reproducible.

Aggregate repeated runs after analysis. The result includes a machine-readable
summary, a run-level CSV, success rate when a test defines `metrics.success`, and
mean, median, maximum, and standard deviation for every recorded numeric metric.

```sh
ros2 run arva_test_protocol arva_test_aggregate \
  --test-dir tests/real_world/fine_search
```

Compare aggregates from equivalent simulation and real-world runs without
inventing a simulation value:

```sh
ros2 run arva_test_protocol arva_test_compare \
  --simulation-summary simulation/fine_search/aggregate/summary.yaml \
  --real-summary tests/real_world/fine_search/aggregate/summary.yaml \
  --output tests/real_world/fine_search/simulation_comparison.yaml
```

For a terrain step response, an operator may deliberately change the existing
controller parameter after recording begins, then publish an event marker:

```sh
ros2 param set /terrain_follow_controller target_distance 1.50
ros2 run arva_test_protocol arva_test_mark \
  --run-dir tests/real_world/terrain_follow/run_001 \
  --event terrain_step_to_1_50m --confirm
```

The controller rejects non-positive or non-finite targets and resets PID history
when it accepts a target change. This command changes the desired AGL only; it
does not arm, take off, or select a PX4 mode.

For waypoint and full-search tests, load the flattened NED waypoint pairs before
selecting the custom mode. The `waypoints_ned` parameter is rejected while
WaypointFollow is active, preventing a path change during a flight:

```sh
ros2 param set /arva_master waypoints_ned "[0.0, 0.0, 0.0, 20.0, 10.0, 20.0]"
```

Copy the same pairs into `planned_waypoints_ned_m` or
`planned_coverage_waypoints_ned_m` in the run configuration so analysis uses the
actual commanded path. The parameter update is captured on `/parameter_events`.

For a complete autonomous-search validation only, automatic completion after a
valid fine-search result can be enabled before selecting the custom mode:

```sh
ros2 param set /arva_master fine_search.auto_complete_on_search_result true
```

It is disabled by default and is rejected while FineSearch is active. When
enabled, a `true` `/finesearch_controller/completed` message reports mode success
to the existing executor, which follows its existing `land` transition. Record
the deliberate parameter change with the full-search run and set
`auto_complete_on_fine_search: true` in that run configuration. Restart the
fine-search state before an individual fine-search or full-search run:

```sh
ros2 service call /finesearch_controller/reset std_srvs/srv/Trigger "{}"
```

The reset service is rejected while FineSearch is active. The controller also
resets itself when the Mode Executor enters `fine_search`, so normal autonomous
missions start from a fresh search state.

## Configuration Notes

The recorder only uses topics currently provided by the production system:

- `/arva_master/mode_state`
- `/arva_test_protocol/event`
- `/pieps/measurement`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_attitude`
- `/terrain_controller/cmd_vel`
- `/fluxline_controller/cmd_vel`
- `/finesearch_controller/cmd_vel`
- `/finesearch_controller/state`
- `/finesearch_controller/estimated_position`
- `/finesearch_controller/completed`

Controller-command and fine-search topics are selected per test. Fine-search
estimated positions are XY-only; their Z value is `NaN` because the current
algorithm estimates a horizontal minimum, not transmitter elevation.

The project uses PX4 local NED coordinates. Surveyed transmitter positions and
the ARVA receiver mounting offset must be expressed against a documented local
origin before comparing real data with simulation.
