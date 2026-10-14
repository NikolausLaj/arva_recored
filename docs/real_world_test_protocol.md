# Real-World ARVA Flight-Test Protocol

This protocol validates the physical system after simulation tuning. It is not a
parameter-sweep procedure. Keep the simulation-selected controller, velocity,
and acceleration configuration unless a measured real-world defect requires a
documented change.

## Safety And Ground Truth

1. Perform normal PX4, RC, battery, GNSS/estimator, propeller, receiver, and
   range-sensor checks before every flight.
2. Start the production nodes separately. The test tools never arm, take off,
   select a PX4 mode, or publish a flight setpoint.
3. Keep manual/RC abort authority available throughout every airborne test.
4. Survey the transmitter and receiver antenna phase-center positions against
   the PX4 local NED origin. Record receiver mounting offset and orientation.
5. Populate the chosen test configuration and run-specific metadata before
   preparation. Record the actual parameters used, not planned values.
6. Prepare the run, start recording with explicit confirmation, then perform
   the operator-supervised test. Mark step changes and test phases with
   `/arva_test_protocol/event`.
7. Stop recording with `Ctrl-C`, declare `completed`, `failed`, or `aborted`,
   then run offline analysis. A bag alone does not establish a successful test.

## Common Commands

```sh
ros2 run arva_test_protocol arva_test_prepare \
   --config "$PWD/src/roscon-25-workshop/px4_roscon_25/arva_test_protocol/config/<test>.yaml" \
   --workspace-root "$PWD/src/roscon-25-workshop" \
  --metadata-file <run-metadata>.yaml --operator "name"

ros2 run arva_test_protocol arva_test_record \
  --run-dir tests/real_world/<test>/run_001 --confirm

ros2 run arva_test_protocol arva_test_mark \
  --run-dir tests/real_world/<test>/run_001 --event <event-name> --confirm

ros2 run arva_test_protocol arva_test_finish \
  --run-dir tests/real_world/<test>/run_001 --outcome completed

ros2 run arva_test_protocol arva_test_analyze \
  --run-dir tests/real_world/<test>/run_001
```

Use names such as `geometry_01_start`, `geometry_01_end`, `heading_step_20_deg`,
or `terrain_step_to_1_50m`. Repeated marker publications are intentional and
are treated as a single named boundary by analysis.

## 1. ARVA Characterization

1. Use a static receiver or manually position the UAV while disarmed when
   practical.
2. For each surveyed geometry, mark a start and end event after the geometry is
   stable.
3. Vary distance, receiver position around the transmitter, and relative
   transmitter/receiver heading.
4. Add one `analysis.samples` entry per geometry with its marker names,
   `expected_distance_m`, and `expected_angle_rad`.
5. Analyze `arva_sensor_characterization`. Record distance and angular error,
   mean, maximum, standard deviation, and validity rates.

## 2. Heading Controller

1. Hover at a safe, constant altitude with zero commanded linear velocity.
2. Activate the existing fluxline-follow mode only after recording begins.
3. Introduce the planned heading offsets, for example $\pm10^\circ$,
   $\pm20^\circ$, and $\pm30^\circ$.
4. Mark each step start and end. Add matching `analysis.segments` entries to
   `heading_controller.yaml`.
5. Check convergence toward zero ARVA offset, settling time, overshoot,
   oscillation crossings, steady-state error, measured heading, and commanded
   yaw rate.

## 3. Terrain Following

### Step Response

1. Hover over level terrain and begin recording.
2. Deliberately change only the existing terrain target parameter:

   ```sh
   ros2 param set /terrain_follow_controller target_distance 1.50
   ```

3. Immediately mark the step. Use a second marker once the response window
   ends. Repeat representative up/down steps.
4. Add target AGL and marker boundaries to `analysis.segments`.
5. Analyze rise time, settling time, overshoot, steady-state error, RMSE, and
   maximum AGL error. The controller publishes zero terrain vertical command
   when PX4 reports invalid range data.

### Terrain Tracking

1. Keep a constant target AGL.
2. Fly the representative slope/terrain profile using the selected flight
   velocity.
3. Mark the beginning and end of each terrain segment.
4. Analyze AGL RMSE, mean absolute error, maximum error, and the trace during
   rapid elevation changes. Retune only after evidence shows the simulation
   values are inadequate.

## 4. Waypoint Following

1. Populate `planned_waypoints_ned_m` in the run configuration with the exact
   route to be flown.
2. Before selecting the custom mode, load the same flattened pairs into
   `/arva_master` `waypoints_ned`; it is rejected while WaypointFollow is active.
3. Include straight legs, $90^\circ$ turns, direction changes, and a representative
   coverage/lawnmower section.
4. Optionally repeat once at a slower reference velocity.
5. Analyze the `waypoint_follow` run for flown-versus-planned XY trajectory,
   cross-track RMSE, maximum error, path length, heading/velocity traces, and
   turn-neighborhood error.

## 5. Fluxline Following

1. Survey and record the transmitter position and heading.
2. Define several materially different start positions and aircraft headings.
3. For each run, position the UAV, start recording, activate fluxline following,
   and terminate only through the normal fine-search transition or a safe abort.
4. Analyze convergence time, flown path length, path efficiency, minimum and
   transition distance to transmitter, signal loss/recovery, and success.
5. Repeat each representative configuration; do not count manual recovery as an
   autonomous success.

## 6. Fine Search

1. Survey the transmitter position and define several initial positions inside
   the fine-search region.
2. For standalone trials, reset stale internal search state before the run:

   ```sh
   ros2 service call /finesearch_controller/reset std_srvs/srv/Trigger "{}"
   ```

3. Record the full trajectory, ARVA data, estimate, controller state, and
   completion signal.
4. Analyze XY localization error between estimated and surveyed transmitter
   position, duration, success, trajectory, and valid-distance rate.
5. Aggregate approximately 5--10 representative completed runs to report mean,
   median, maximum, standard deviation, duration, and success rate.

## 7. Complete Autonomous Search

1. Fix a representative search polygon and coverage path. Change the surveyed
   transmitter position between runs while retaining the polygon where possible.
2. Set `planned_coverage_waypoints_ned_m` and load the matching
   `waypoints_ned` parameter before mode activation.
3. Reset fine-search state before the run. The controller also resets when the
   executor enters FineSearch.
4. Leave automatic completion disabled for component tests. For a deliberate
   complete-mission test only, set both the recorded configuration field
   `auto_complete_on_fine_search: true` and the existing executor parameter:

   ```sh
   ros2 param set /arva_master fine_search.auto_complete_on_search_result true
   ```

5. Start recording before selecting the custom mode. Supervise the expected
   sequence: coverage/waypoint, ARVA detection, fluxline, fine search,
   localization, executor `land` transition.
6. A successful run must contain the ordered autonomous mode progression,
   a valid fine-search completion, a final XY localization estimate, and the
   executor landing transition. Analyze phase durations, detection and
   transition times, total path length, localization error, mode timeline, and
   success rate across transmitter locations.

## Aggregation And Simulation Comparison

After analyzing all runs for one test type:

```sh
ros2 run arva_test_protocol arva_test_aggregate \
  --test-dir tests/real_world/fine_search
```

Use `arva_test_compare` only with aggregate summaries generated from equivalent
recorded simulation and real-world experiments. It reports shared metrics such
as real-minus-simulation RMSE, convergence time, localization error, or search
time. It does not create missing simulation results.