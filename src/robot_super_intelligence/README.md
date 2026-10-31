# robot_super_intelligence

## Architecture

This package replaces `robot_state_controller` and owns robot modes, command
routing, and action supervision. Startup launches two nodes:

- `manager.py` exposes `/manage_robot` (`autonomy_msgs/ManageRobot`). An operation
  enum selects navigation, dig, dump, or full-auto. `action_clients.py` adapts the
  request to Nav2 `NavigateToPose`, `Dig`, or `Dump`, forwarding feedback and results.
- `command_mux.py` arbitrates both drive and arm/drum commands. `arbitration.py`
  applies eligible-source grants, configurable priorities, a shared stop lock, and
  command timeouts. The manager atomically grants both channels through the
  internal `/control/select` service before dispatching a child action.

`config/control.yaml` defines source priorities, the shared lock topic and initial
state, command/manager timeouts, and the manager heartbeat period. Startup loads this
file for both nodes; restart them after editing it.

The manager grants one source at a time. Priorities compare only explicitly
eligible sources; a higher priority never bypasses ownership or the shared stop lock.
Both outputs always use the same winning source, with zero for missing channels.
The arbiter also supports multiple eligible sources for future policies. Every
operation owns wheels and arms/drums together. Navigation keeps arm/drum outputs
at zero under that same grant. Full-auto has no motion channel;
its current step receives the appropriate routine's grant.

`full_auto.py` runs the existing `machine.py` transition graph internally, using
state-completion futures rather than calling the public manager action again:

```text
Client -> manage_robot -> navigation / dig / dump action
                      -> full-auto states -> same internal dispatcher
```

The sequence remains RunStart, construction traversal, dump, excavation traversal,
excavate, then back to construction. Operational states are still placeholders
returning `NOT_IMPLEMENTED`; a full-auto goal currently aborts without motion.

## Ownership and stopping

The robot starts in ESTOP. `/robot_state/estop` (`std_srvs/SetBool`) engages the
stop with `true`; `false` clears it to IDLE without enabling teleop or resuming work.
`/robot_state/teleop` enables teleop with `true`, or selects idle with `false`.
Autonomous goals are explicit and rejected while stopped or busy.

E-stop locks both outputs immediately and cancels the child action. Parent goals
remain reserved until the child reports a terminal result, including goals still
awaiting acceptance. Completed operations restore TELEOP after the child is
terminal; an explicitly activated E-stop stays latched. The full-auto deadline
succeeds; standalone operation deadlines abort. Action cancellation
returns CANCELED; E-stop and mode exit abort the parent.

Command watchdogs output zero for stale inputs. Loss of the manager heartbeat
locks the mux until a new grant; restoring the heartbeat alone does not resume
motion. These watchdogs and the one-shot run deadline use steady time. Routine
servers retain their own control-loop timers and no longer change global modes.
The legacy mode topics accept stop/teleop/idle requests but cannot start autonomy;
repeated ESTOP requests never toggle it off. The old `/full_auto` action endpoint
is replaced by `/manage_robot` with `operation: 3`.

## Extending

Each class in `states/` declares allowed `Outcome` values. `enter()` resets data
for each visit; `start(dispatch)` returns a future resolving to an outcome. States
are reused across cycles. Use the supplied `dispatch(ManageRobot.Goal(...))` for
child operations; map its result string to the state's outcome in a completion
callback. Do not block, poll, or call `/manage_robot` recursively. `cancel()` cleans
up local work; the manager cancels and waits for the dispatched child.

For a new state, export it from `states/__init__.py`, add it to `STATE_TYPES`, and
add its success and failure entries to `TRANSITIONS`. For a new operation, extend
`ManageRobot.action`, the typed adapters in `action_clients.py`, manager mode
mapping, and mux source configuration. Tests cover the graph separately from
ROS delegation, cancellation races, and command arbitration.

## Run

```bash
ros2 service call /robot_state/estop std_srvs/srv/SetBool '{data: false}'
ros2 action send_goal /manage_robot autonomy_msgs/action/ManageRobot '{operation: 3, time_limit_sec: 600.0}' --feedback
```
