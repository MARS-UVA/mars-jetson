# robot_super_intelligence

## Architecture

- `server.py` owns the `full_auto` ROS 2 action, mode monitoring, feedback, and
  stop requests. Its steady-clock timer advances the machine at 20 Hz without
  blocking subscription or cancellation callbacks. Only one goal runs at a time;
  goals require `FULL_AUTO` mode and a positive, finite `time_limit_sec`.
- `machine.py` is independent of ROS. `STATE_TYPES` registers the state classes,
  and `TRANSITIONS` maps `(state class, Outcome)` to the next class. The machine
  tracks elapsed wall-clock time, completed excavation cycles, and the latest
  outcome. Feedback reports these alongside the current state; the action result
  reports why the entire run ended.
- `states/` contains one class per state. `outcomes.py` defines the shared
  `Outcome` constants; each state declares its allowed return values in `outcomes`.

The normal path is:

```text
RunStart -> TraverseToConstruction -> DumpMaterial -> TraverseToExcavation -> Excavate
                    ^                                                          |
                    +----------------------------------------------------------+
```

Successful excavation increments the cycle count. The deadline interrupts the
active state and enters `RunEnd`, completing the action successfully. Failures,
cancellation, and leaving `FULL_AUTO` also end the run. Explicit action cancellation
returns CANCELED; leaving the mode returns ABORTED with `MODE_EXITED`. Every finished
run stops its command channels and requests ESTOP through `robot_state/set`, which
sets the mode directly rather than toggling it.

The controller selects `cmd_vel/full_auto` and `arm_drum_control/full_auto` in
`FULL_AUTO`. Operational states currently return `NOT_IMPLEMENTED`, so real goals
stop at the first traversal without commanding motion.

## Extending the machine

1. Implement the relevant class in `states/`, replacing its `Placeholder` base
   with `State`. Use `enter()` to reset per-invocation data and start asynchronous
   work. State objects are reused on subsequent cycles.
2. Implement `tick()` to return `None` while work is pending, then a declared
   `Outcome` when complete. Keep it nonblocking: future ROS action clients should
   send goals asynchronously and inspect their futures on later ticks. Complex
   routines can delegate to helper modules beneath `states/`.
3. Implement `cancel()` to stop owned work, including child actions and commands.
   The machine calls it when ending a run. Normal successful transitions do not
   call it, so finish normal cleanup before returning a completion outcome.
4. For a new state, export it from `states/__init__.py`, register it in
   `STATE_TYPES`, and add its transitions. Add new return conditions to `Outcome`
   and the state's `outcomes`, with a transition for each nonterminal outcome.
   Update the failure-transition registration if the new state needs it.
5. Supply ROS dependencies from `server.py`: construct state instances there and
   pass a class-to-instance mapping as `Machine(..., states=...)`. Keep ROS clients
   out of the transition engine. Before adding motion, replace the server's current
   unconditional neutral-command publishing with appropriate command ownership.

`test/test_machine.py` uses injected states and a clock to test transitions and
deadlines. `test/test_action.py` exercises the ROS action lifecycle. Extend these
when adding new transitions or child-action behavior.

## Run

The startup robot launch includes the server. To select full-auto and send a goal:

```bash
ros2 topic pub --once /robot_state/set std_msgs/msg/UInt8 '{data: 5}'
ros2 action send_goal /full_auto autonomy_msgs/action/FullAuto '{time_limit_sec: 600.0}' --feedback
```
