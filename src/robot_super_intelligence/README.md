# robot_super_intelligence

## Architecture

This package replaces `robot_state_controller` and owns robot modes, command
routing, and action supervision. Startup uses the manager and standard muxes:

- `manager.py` exposes `/manage_robot` (`autonomy_msgs/ManageRobot`). An operation
  enum selects navigation, dig, dump, or `RUN_MACHINE` (3). A `machine_name`
  string selects a registered machine. It owns parent goals, machine
  execution, deadlines, feedback, and results.
- `arena_markers.py` subscribes to `/arena/markers` and derives the construction
  approach from the published polygon. Enable the marker publisher before starting
  DigTraverseDump (for example, `arena_name:=ARENA_NASA` in startup).
- `child_operation.py` manages one child goal at a time: routing, acceptance,
  cancellation, and terminal results. Ownership is retained through late acceptance.
- `control_routing.py` owns mux selection, locks, stop heartbeat, mode publishing,
  and E-stop/teleop services. All three classes share the existing manager node.
- `operations.py` owns the typed navigation, dig, and dump ROS action clients,
  request validation, and goal construction. Machine states dispatch these operations.
- Standard `twist_mux` selects drive commands by priority and lock settings.
  Standard `topic_tools mux` selects the arm/drum input through its service.

`config/control.yaml` holds the standard mux configuration, priorities, shared
lock topic, timeouts, and manager settings. The manager keeps both outputs stopped
while changing sources, waits for previous velocity inputs to expire, then starts
the next operation. Keep `source_transition_sec` at least as large as the longest
normal `twist_mux` input timeout. Navigation selects the zero arm/drum input.
Full-auto dispatches its current step through the same manager path.

`machine.py` executes a graph, and `machine_runner.py` advances it through
state-completion callbacks. Each class under `machines/` defines its own states,
explicit transitions, initial/end states, and successful ending outcomes:

- `FullAutoMachine` (`machine_name: full_auto`): RunStart → construction traversal → dump →
  excavation traversal → excavate → back to construction, until the deadline.
- `RunStartMachine` (`machine_name: run_start`): RunStart → construction traversal → RunEnd.
  It finishes successfully with `CONSTRUCTION_REACHED`; its deadline is a failure.

- `DigTraverseDumpMachine` (`machine_name: dig_traverse_dump`): Excavate (Dig) →
  TraverseToConstruction → DumpMaterial (Dump) → RunEnd, once. Uses the goal's
  `routine_index` for dig/dump. Construction comes from `/arena/markers`: a
  `LINE_STRIP` polygon with namespace `Construction zone`. Traversal reads the
  latest polygon, approaches the midpoint of its highest-local-Y edge, and faces
  inward. Marker pose and frame are respected; visual line height is ignored.
  Goals are rejected until valid geometry is received. Deletion, expiry, or invalid
  geometry before traversal fails the run. A navigation goal already underway
  keeps its target. Finishes with `DUMP_SUCCESS`; failures and deadlines abort.

These reuse the classes under `states/` and run in the robot's FULL_AUTO control
mode. No extra node or action server is needed for a new machine. RunStart and
DigTraverseDump receive their construction destination from arena markers.
FullAuto goals are rejected before execution because the current interface has no
excavation-destination source. Its graph remains available for development and tests.

## Ownership and stopping

The robot starts in ESTOP. `/robot_state/estop` (`std_srvs/SetBool`) engages the
stop with `true`; `false` clears it to IDLE without enabling teleop or resuming work.
`/robot_state/teleop` enables teleop with `true`, or selects idle with `false`.
Autonomous goals are explicit and rejected while stopped or busy.

E-stop publishes zero drive commands, selects the zero arm/drum input, and cancels
the child action. Parent goals remain reserved until the child reports a terminal result, including goals still
awaiting acceptance. Completed operations restore TELEOP after the child is
terminal; an explicitly activated E-stop stays latched. The full-auto deadline
succeeds; RunStart and standalone operation deadlines abort. Action cancellation
returns CANCELED; E-stop and mode exit abort the parent.

`twist_mux` input timeouts and the wheel controller's `cmd_vel_timeout` handle
stale drive commands. The manager refreshes the drive lock and publishes zeros
while stopped. The arm/drum controller has no command timeout: lost arm/drum
commands do not automatically stop those joints. No extra watchdog node is used.
Routine servers retain their own control-loop timers and do not change modes.
The legacy mode topics accept stop/teleop/idle requests but cannot start autonomy;
repeated ESTOP requests never toggle it off. The old `/full_auto` action endpoint
is replaced by `/manage_robot` with `operation: 3, machine_name: full_auto`.

## Extending

Each class in `states/` declares allowed `Outcome` values. `enter()` resets data
for each visit; `start(dispatch, on_complete)` calls `on_complete(outcome)` when
finished and returns immediately. States are reused across cycles. Use
`dispatch(ManageRobot.Goal(...), on_result)` for child operations and map the
result string to the state's outcome in `on_result`. Do not block, poll, or call `/manage_robot` recursively. `cancel()` cleans
up local work; the manager cancels and waits for the dispatched child.

For a new state, subclass `State` in `states/state.py`, export it from
`states/__init__.py`, and add it to the machine's `state_types` and explicit
`transitions`. `start()` is abstract: incomplete states cannot be instantiated.
Require configuration in the constructor rather than accepting missing arguments.
Traversal states inherit `Traverse`, provide a destination callable, and declare a
`success_outcome`. If a previously valid destination disappears during a run,
they report `FAILED`.

Standalone ROS operations are separate adapters in `operations.py`. They own
`client`, `valid(request)`, `send_goal(request, feedback, on_response)`, `source`,
and `mode`; they are not machine states. Register new adapters in the manager's
`operations` mapping and extend `ManageRobot.action` and mux configuration as
needed. Machine `valid()` must reject missing or invalid configuration before a
goal is accepted; `from_goal()` constructs the configured state instances.
Tests cover the graph separately from ROS delegation, cancellation races, and
command routing.

`MachineRunner.start(on_complete)` reports one final outcome. State transitions
are queued through the executor; duplicate and late state callbacks are ignored.
The manager completes action goals directly from callbacks. ROS request Futures
are handled with `add_done_callback`; only the existing service-response boundary
uses a small async adapter so acknowledgements retain their original meaning.

For a new machine, subclass `Machine` under `machines/`, defining `state_types`,
`initial_state`, `end_state`, `transitions`, and `success_outcomes`. Export the
class and register its string name in `MACHINES` in `machines/__init__.py`.
No action-interface change is needed. Empty or unknown machine names are rejected.

## Run

`one_cycle` follows DigTraverseDump, then navigates to the `Excavation approach`
arena marker and finishes with `EXCAVATION_REACHED`. NASA's approach is at
`(2.5, 3.875)` in `map`, on the excavation/obstacle border halfway between the
top wall and the pillar's upper face, facing into excavation (-X). Both arena
destinations must be available before acceptance. The return uses the latest
approach marker; a missing, invalid, deleted, or expired marker fails traversal.
`completed_cycles` increments after dumping, as in DigTraverseDump; return
failure still aborts the goal. Enable the arena publisher with
`arena_name:=ARENA_NASA` and restart it after rebuilding `robot_navigation`.

```bash
ros2 action send_goal /manage_robot autonomy_msgs/action/ManageRobot '{operation: 3, machine_name: one_cycle, routine_index: 0, time_limit_sec: 180.0}' --feedback
```

```bash
ros2 service call /robot_state/estop std_srvs/srv/SetBool '{data: false}'
# Full excavation loop (currently rejected: excavation destination is not configured)
ros2 action send_goal /manage_robot autonomy_msgs/action/ManageRobot '{operation: 3, machine_name: full_auto, time_limit_sec: 600.0}' --feedback
# One dig / traverse / dump, using the published arena markers.
ros2 action send_goal /manage_robot autonomy_msgs/action/ManageRobot '{operation: 3, machine_name: dig_traverse_dump, routine_index: 0, time_limit_sec: 120.0}' --feedback
# Short startup sequence instead
ros2 action send_goal /manage_robot autonomy_msgs/action/ManageRobot '{operation: 3, machine_name: run_start, time_limit_sec: 30.0}' --feedback
```
