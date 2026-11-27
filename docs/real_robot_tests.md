# Real robot tests

These tests can only be run by hand on real Dynamixel motors. No mock and no CI job covers them.
Run them whenever the driver, the controller manager (`hector_controller_manager`) or the robot
configuration changes in a way the test names. Fill in the result fields, and commit the filled
copy next to the test log.

## General setup

- **Motors.**
  - Bench: one X series motor (it must support the bus watchdog) on a U2D2, powered from a bench
    supply whose output can be switched off by hand.
  - Robot: athena with its normal configuration (`athena_driver_launch`).
- **People.** Every test needs two people: the operator, who runs the commands, and the e-stop
  holder, who does nothing else. Each test's **Safety** line says what the e-stop holder holds.
- **Arm.** The arm has no brake (PH54 and PH42 motors). The arm stays in position mode until W3 has
  passed. The only exception is the sag measurement in B4, which has its own safety rules.
- **Software.** `hector_controller_manager` at 50 Hz like athena, a sync component unless the test
  says async. Record the commit of every repository in the result fields.
- **Logging.** Record all of these for every test:
  - `ros2 bag record /joint_states` and the Dynamixel debug topics (`~/goal_joint_states`,
    `~/read_joint_states`, `~/write_joint_states`);
  - the controller manager log, including the Dynamixel `[timing]` lines;
  - the pal_statistics topics.

  Position, velocity and current values in the pass criteria are read from `~/read_joint_states`,
  goals from `~/goal_joint_states`.
- **Bench URDF.** Some tests need command interfaces athena does not export: velocity on the arm,
  or a pure current joint. Use a bench URDF and note it in the result.
- **E-stop topics.** The bench has no e-stop board. There, an e-stop is a message on the topic
  configured in the controller manager, published like this:

  ```bash
  ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable \
    /estop_board/hard_estop std_msgs/msg/Bool "{data: true}"
  ```

  Publish `false` to release it. The soft e-stop uses `e_stop_manager/aggregated_state/emergency_stop_software`
  in the same way.

Each test has these fields:
- **Result:** pass / fail, the measured values;
- **Date / tester / commits:**;
- **Notes:**.

## B1 Switch and lifecycle timing

- **Safety:** e-stop holder at the bench supply switch. The bench motor turns freely with a light
  load and nothing within its reach.
- **Setup:** bench motor. Run once with a sync component and once with `is_async`.
- **Steps:**
  1. Configure and activate the component through `set_hardware_component_state`.
  2. Switch: start only (position controller), stop only, a combined switch with a mode change
     (position to velocity, bench URDF), and a same-interface handover (two position controllers).
  3. Activate and deactivate the torque-off controller.
  4. Trigger the startup re-apply: publish `true` and then `false` on the hard e-stop topic (see
     General setup).
- **Observe:**
  - the `[timing]` lines for `on_configure`, `on_activate` and `perform_command_mode_switch`;
  - the duration of the whole switch service call.
- **Pass:**
  - every perform takes 20 ms or less;
  - `on_configure` plus `on_activate` stays below the controller manager's hardware state timeout;
  - all numbers are recorded.
- **Result:** __________ **Date / tester / commits:** __________

## B2 Bus watchdog while writes stop

- **Safety:** e-stop holder at the bench supply switch. Free load, soft limits set, nothing within
  reach of the load.
- **Setup:** bench motor, `bus_watchdog_cycles` 4, velocity 0.3 rad/s. A debug build that skips
  `write()` for 300 ms while `read()` continues.
- **Steps:** command the velocity, trigger the write gap, watch the motor.
- **Observe:**
  - whether the motor stops within 4 cycles (80 ms at 50 Hz);
  - whether the watchdog error is flagged (the Bus Watchdog register reads -1).
- **Pass:** the answer to "do reads feed the bus watchdog" is recorded. If they do, note that a
  read-only gap does not stop a velocity joint.
- **Result:** __________ **Date / tester / commits:** __________

## B3 Handover during motion

- **Safety:** e-stop holder at the bench supply switch. Free load, soft limits set.
- **Setup:** bench motor, position and velocity controllers (bench URDF), sync and async.
- **Steps:**
  1. Move at 0.5 rad/s, or along a position trajectory.
  2. Switch to the other controller in one request.
- **Observe:**
  - the travel between the old controller's last command and the new one's first;
  - the number of mode changes (torque blips) per switch.
- **Pass:**
  - the travel during the handover is at most 0.02 rad beyond the position held when NaN arrived;
  - at most one mode change per switch.

  A handover across a slow `on_deactivate` is covered on the robot by the flipper SyncGroup test
  (R1), because SyncGroup joins its action threads there.
- **Result:** __________ **Date / tester / commits:** __________

## B4 NaN commands and the arm's velocity-0 sag

- **Safety:**
  - Part 1: e-stop holder at the bench supply switch.
  - Part 2 is the one documented exception to "no arm velocity without a working watchdog".
    - It runs on a bench URDF only, for this measurement only.
    - The arm hangs over a catch with a few cm of clearance (a sling or a stand), not on a
      support, so it can sag but not fall far.
    - The e-stop holder holds the robot's hard e-stop and presses it at once if the sag exceeds
      0.1 rad on any joint, or if anything else moves.
    - Nobody stands under the arm.
- **Setup:** bench motor (sync and async) for part 1; the athena arm with a bench URDF that exports
  velocity for part 2.
- **Steps:**
  1. Publish NaN to each controller type during motion.
  2. Sag: put the arm in a horizontal pose, switch joints 2 to 4 to velocity mode one at a time,
     command NaN (meaning velocity 0) and wait 10 s.
- **Observe:** the position after NaN; the sag per joint over 10 s.
- **Pass:**
  - position: the joint stays within 0.01 rad of where it was when NaN arrived, over 10 s;
  - velocity: the joint stops, below 0.01 rad/s within 100 ms;
  - sag: the value per joint is recorded and stayed below the 0.1 rad abort threshold. It feeds
    the decision whether velocity mode may run on the arm at all.
- **Result:** __________ **Date / tester / commits:** __________

## B5 Controller manager loss

- **Safety:** e-stop holder on the robot's hard e-stop. Arm supported, flippers lifted off the
  ground.
- **Setup:** bench motor in velocity mode with the watchdog on, then athena: the flippers have the
  watchdog armed, the arm has it off until W3 passes.
- **Steps:**
  1. During motion at a known velocity v, SIGKILL the controller manager process.
  2. Restart it. During motion, stop the whole process with `kill -STOP <pid>` for 500 ms, then
     `kill -CONT <pid>`. This freezes the hang check too, so this step tests the motors' own
     reaction to a silent bus, not the controller manager's hang detection. That one is covered by
     the controller manager's own test suite.
- **Observe:** the travel after the kill or the stop, per joint.
- **Pass:**
  - bench motor and flippers: they stop within the watchdog window, so the travel stays below
    v × (window + one cycle);
  - arm (watchdog off): it holds its last position goal, within 0.02 rad;
  - after `kill -CONT`, nothing jumps: the travel in the next 500 ms stays within 0.02 rad of the
    commanded path.
- **Result:** __________ **Date / tester / commits:** __________

## B6 Power cut during motion

- **Safety:** e-stop holder on the robot's hard e-stop, operator at the motor power switch. Arm on
  a catch with clearance: it drops when power goes. Flippers lifted.
- **Setup:** bench motor, then athena via the hard e-stop.
- **Steps:**
  1. Cut motor power during motion and keep it off for 5 s.
  2. Restore it.
- **Observe:**
  - read errors leading to ERROR, `on_error`, and the component ending unconfigured;
  - the claimers stopped (`ros2 control list_controllers`);
  - **the duration of `on_error` on the dead bus**;
  - the motion when power returns.
- **Pass:**
  - the component reaches Unconfigured and its claimers are inactive;
  - at power return no joint moves more than 0.02 rad;
  - the `on_error` time is recorded and stays within the controller manager's fault-path budget.
- **Result:** __________ **Date / tester / commits:** __________

## B6b Power blip shorter than the read error threshold

- **Safety:** as B6.
- **Setup:** bench motor.
- **Steps:** cut power for about 100 ms during motion. The threshold is 25 failed reads, which is
  500 ms at 50 Hz.
- **Observe:** whether the driver notices the motor reboot (torque off while desired on).
- **Pass:** the component returns ERROR and goes through the fault path, and the controller manager
  recovers it. It must never continue silently with a rebooted motor.
- **Result:** __________ **Date / tester / commits:** __________

## B7 Hard e-stop and recovery

- **Safety:** e-stop holder on a second hard e-stop or at the main power. Arm supported or on a
  catch; nobody within its reach while it re-energises.
- **Setup:** athena.
- **Steps:**
  1. Press the hard e-stop during motion.
  2. Release it.
  3. Wait for the controller manager's cooldown and recovery.
  4. Repeat with the gripper grasping an object at the press. While the power is off, take the
     object out and open the fingers by hand. After recovery, start a gripper controller that sends
     no command yet.
- **Observe:**
  - the recovery time;
  - the controller set after recovery;
  - the positions in the first 500 ms after re-energising.
- **Pass:**
  - the startup set is active again within the recovery timeout;
  - in the first 500 ms after torque on, no joint moves more than 0.02 rad or faster than
    0.1 rad/s;
  - a torque-off controller that was active before the press is not restored, and the joints come
    back powered;
  - in step 4 the gripper stays where the hand left it and does not close onto the old grasp.
- **Result:** __________ **Date / tester / commits:** __________

## B8 Soft e-stop

- **Safety:** e-stop holder on the robot's hard e-stop. Arm supported during step 3 (torque off).
- **Setup:** athena with the startup set active, `arm_torque_off` inactive.
- **Steps:**
  1. Engage the soft e-stop during motion.
  2. Try to activate a normal controller.
  3. Activate `arm_torque_off`, then deactivate it alone (the arm stays limp).
  4. Hands off. Release the e-stop.
- **Observe:** which controllers stop and how fast; whether the hardware holds; which requests are
  rejected.
- **Pass:**
  - every non-protected command claimer stops within one cycle, and every joint stays within
    0.02 rad of its position at the e-stop;
  - the normal activation is rejected while the e-stop is latched, and `arm_torque_off` is
    accepted;
  - the arm stays limp after step 3, and after the release the controller manager restores the
    startup set: the arm's torque comes back with `arm_trajectory_controller`, holding within
    0.02 rad.
- **Result:** __________ **Date / tester / commits:** __________

## B8b Fail-safe e-stop topics

The expected behaviour follows the controller manager's e-stop design. Check it against the final
design before running.

- **Safety:** e-stop holder on the robot's hard e-stop. Arm supported.
- **Setup:** athena, with the e-stop publishers (e-stop board driver and `e_stop_manager`) under
  the operator's control.
- **Steps:**
  1. Start the controller manager while no publisher exists on the soft and the hard e-stop topic.
  2. Start the publishers with `false`.
  3. With the startup set running, engage the soft e-stop, then kill its publisher without
     releasing it.
  4. Try to activate a normal controller.
  5. Start the publisher again and publish `false`.
- **Observe:** the controller states after each step; the controller manager log.
- **Pass:**
  - after step 1 no command claimer is active: a topic without a message counts as engaged;
  - after step 2 the startup set comes up;
  - after step 3 the e-stop stays engaged, and the joints hold within 0.02 rad;
  - step 4 is rejected with the e-stop named as the reason;
  - only step 5, a release from a live publisher, releases the latch.
- **Result:** __________ **Date / tester / commits:** __________

## B9 Torque-off controller

- **Safety:** e-stop holder on the robot's hard e-stop. Arm supported while limp. **Hands off the
  arm before step 3**: the operator says "torque on" and waits until everybody has stepped back.
  After step 3 the limp arm shows only in the LEDs and the diagnostics, not in the controller list.
- **Setup:** athena.
- **Steps:**
  1. Activate `arm_torque_off`.
  2. Move the arm by hand.
  3. Deactivate it alone. The arm stays limp, move it by hand again.
  4. Activate `arm_torque_off` again.
  5. Hands off. Deactivate it, combined in one AUTO switch with activating
     `arm_trajectory_controller` (which brings `arm_safety_position_controller`).
  6. Repeat steps 4 and 5, then press the hard e-stop while the arm is limp, and release it.
- **Observe:**
  - the gripper stays powered while the arm is limp;
  - the motion at torque on;
  - the number of performs per switch.
- **Pass:**
  - limp, still limp after step 3, then at torque on no joint moves more than 0.02 rad;
  - one perform for the combined switch;
  - after the hard e-stop release the arm comes back powered, holding within 0.02 rad.
- **Result:** __________ **Date / tester / commits:** __________

## B10 Offset calibration

- **Safety:** e-stop holder on the robot's hard e-stop. Flippers lifted.
- **Setup:** athena flipper with an adjustable offset transmission.
- **Steps:**
  1. Calibrate a joint that no controller claims and that holds position.
  2. Calibrate a claimed joint.
- **Observe:** the motor's motion; the service responses; `overlap_skipped_cycles` in the health
  diagnostics before and after each request.
- **Pass:** the held joint moves less than 0.005 rad, the claimed joint's request is refused, and
  `overlap_skipped_cycles` grows by at most 1 per request.
- **Result:** __________ **Date / tester / commits:** __________

## B11 Switch on a shared bus

- **Safety:** e-stop holder on the robot's hard e-stop. Nobody within the arm's reach.
- **Setup:** athena arm bus (arm and gripper).
- **Steps:** while `arm_safety_position_controller` moves the arm along a trajectory, switch the
  gripper between its two controllers ten times.
- **Observe:** the arm's velocity and tracking error during the gripper switches.
- **Pass:** during each switch, no arm joint's velocity drops more than 0.05 rad/s below its value
  in the cycles before, and the tracking error grows by less than 0.01 rad.
- **Result:** __________ **Date / tester / commits:** __________

## B12 A cap without a main interface is refused

- **Safety:** e-stop holder at the bench supply switch. Load on a catch with clearance.
- **Setup:** bench motor with a max_current and a position command interface (bench URDF), held by
  a position controller.
- **Steps:** activate a controller that claims only `max_current`.
- **Observe:** the switch response and the controller manager log; the motor.
- **Pass:** the switch is refused with a message naming max_current without position or velocity,
  the position controller stays active and the motor holds within 0.02 rad.
- **Result:** __________ **Date / tester / commits:** __________

## R1 Flipper SyncGroup handover

- **Safety:** e-stop holder on the robot's hard e-stop. Robot lifted, so the flippers turn freely.
- **Setup:** athena with the startup set (flipper_velocity_controller → SyncGroup → flipper SPC).
- **Steps:**
  1. Drive the flippers at 0.3 rad/s through `flipper_velocity_controller`.
  2. Switch to `flipper_trajectory_controller` in one FORCE_AUTO request.
  3. Switch back.
- **Observe:** the travel during each handover; the mode changes; the faults in the log.
- **Pass:** each handover moves a flipper at most 0.02 rad beyond the position held when NaN
  arrived; no mode change (both chains end in position); no fault.
- **Result:** __________ **Date / tester / commits:** __________

## R2 Gripper grasp across a controller change

- **Safety:** e-stop holder on the robot's hard e-stop. Nothing but the test object between the
  fingers.
- **Setup:** athena gripper, `do_not_reset_on_ctrl_change` set, an object in the gripper.
- **Steps:** grasp with `gripper_position_controller`, then switch to `gripper_trajectory_controller`
  and back.
- **Observe:** the present current, the goal position and the object during the switches.
- **Pass:**
  - the present current stays at or above 90 % of the configured current limit throughout;
  - the goal position stays within 0.001 rad of its value before the switch. This separates the
    keep-goal policy from a latch at the present position;
  - the object stays gripped.
- **Result:** __________ **Date / tester / commits:** __________

## R3 Arm current limits through SimpleTransmission

The `max_current` that `arm_safety_position_controller` claims passes the SimpleTransmission of
arm_joint_1 and arm_joint_7 unchanged, so they run at the compliant 3 A and 0.5 A.

- **Safety:** e-stop holder on the robot's hard e-stop. Arm workspace clear.
- **Setup:** athena with the startup set, `arm_safety_position_controller` in its compliant mode.
- **Steps:** move arm_joint_1 and arm_joint_7 through their usual motions, including poses where
  gravity or the payload loads them.
- **Observe:** the goal current and the present current of both joints in `~/write_joint_states`
  and `~/read_joint_states`; the tracking of each motion.
- **Pass:**
  - the goal current reads 3 A on arm_joint_1 and 0.5 A on arm_joint_7;
  - both joints complete every motion, against gravity too, without stalling at the limit.
- **Result:** __________ **Date / tester / commits:** __________

## W3 Bus watchdog reliability on the arm

- **Safety:** e-stop holder on the robot's hard e-stop. Arm supported.
- **Setup:** athena arm with `bus_watchdog_cycles` 4 at 50 Hz.
- **Steps:**
  1. Run 30 min of random switches, torque-off cycles and a soft e-stop.
  2. Kill the controller manager.
- **Observe:** the watchdog trip warnings in the log; the stop after the kill.
- **Pass:**
  - no spurious trip in 30 min;
  - the arm stops within the window after the kill.

  Only after this passes may the arm run in velocity mode.
- **Result:** __________ **Date / tester / commits:** __________

## R4 Robot soak

- **Safety:** e-stop holder on the robot's hard e-stop for the whole run. Robot lifted.
- **Setup:** athena, full launch.
- **Steps:** 30 min of chained switches through the orchestrator (flipper velocity to trajectory,
  arm trajectory to twist), torque-off cycles and one soft e-stop. The arm twist step needs
  `moveit_twist_controller` built from source against the controller manager's ros2_control. Without
  it, skip that step and note that in the result.
- **Observe:** the mode changes per switch, the faults, the warnings, and the cycle time statistics.
- **Pass:** no fault; no joint more than 0.02 rad away from its commanded path; the mode changes per
  switch match the plan.
- **Result:** __________ **Date / tester / commits:** __________

## Asynchronous components (A1 to A4)

Run these after B1, B3, B4 and B5 have passed in their async variant. **Setup** for all of them:
athena.xacro with `dynamixel_async:=true`, and the controller manager at 100 Hz (`update_rate: 100`)
instead of the 50 Hz of the general setup, so a `bus_watchdog_cycles` of 4 is a 40 ms window. In
the bus simulation that is `dynamixel_update_rate:=100`. A1 to A3 run with the
arm's watchdog disarmed, as athena has `bus_watchdog_cycles` 0 on the arm today. Record the owner scheduling from the "Realtime
Scheduling" diagnostic and the skipped cycles per component from "Controller Manager Activity".
Run each test once sync and once async at the same rate and compare.

## A1 E-stop latency during a fast move

- **Safety:** e-stop holder on the robot's hard e-stop. Arm workspace clear, nobody within reach.
- **Steps:**
  1. Move the arm along a trajectory near its velocity limit.
  2. Engage the soft e-stop in the middle of the move. Repeat 10 times.
  3. Repeat with the hard e-stop 3 times.
- **Observe:** the time from the e-stop message to the first `~/read_joint_states` sample in which
  every arm joint is below 0.01 rad/s, and the travel in that time.
- **Pass:** the soft e-stop stops the arm within 3 cycles plus the motor's deceleration, async no
  more than one cycle later than sync; every value is recorded.
- **Result:** __________ **Date / tester / commits:** __________

## A2 30 min stress

- **Safety:** e-stop holder on the robot's hard e-stop for the whole run. Robot lifted.
- **Steps:** R4's 30 min sequence, with `stress-ng --cpu <cores> --io 2` on the PC throughout.
- **Observe:** the skipped cycles per component, the bus silence warnings, the watchdog trip
  warnings, faults, and the cycle time statistics.
- **Pass:** no fault and no watchdog trip; no joint more than 0.02 rad from its commanded path;
  the skipped cycles are recorded, and no stretch of them reaches the watchdog window.
- **Result:** __________ **Date / tester / commits:** __________

## A3 Bus pulled during motion

- **Safety:** e-stop holder on the robot's hard e-stop. Arm on a catch with clearance, flippers
  lifted.
- **Steps:** while the arm and the flippers move, pull the USB adapter of one bus. Plug it back and
  request the component active with `set_hardware_component_state`. Once per bus.
- **Observe:** the time to the fault, the claimers stopped, the motion after the pull, the recovery.
- **Pass:** the component faults and its claimers stop; a position joint holds its last goal within
  0.02 rad (a joint with an armed watchdog would stop within the window, which applies only once
  the arm runs in velocity mode); the other bus keeps running
  untouched; the recovery works without a restart and nothing moves at torque on.
- **Result:** __________ **Date / tester / commits:** __________

## A4 Watchdog trip in velocity mode, stop distance

W3 with the async variant. It decides whether the arm may run in velocity mode asynchronously.

- **Safety:** e-stop holder on the robot's hard e-stop. Bench URDF that exports velocity on the arm,
  watchdog on. Arm supported, soft limits set.
- **Steps:**
  1. Move one arm joint at v = 0.3 rad/s, then at 1 rad/s.
  2. Stop the whole controller manager for 200 ms as one command:
     `kill -STOP $p; sleep 0.2; kill -CONT $p`, with `p` the pid of `ros2_control_node`.
  3. Repeat with a stop of 20 ms, which stays inside the window:
     `kill -STOP $p; sleep 0.02; kill -CONT $p`.
- **Observe:** the stop distance per joint after the stop, and what happens after `kill -CONT`.
- **Pass:**
  - 200 ms: the joint stops, travel at most v × (window + one cycle) plus the measured
    deceleration distance;
  - 20 ms: no trip and no visible hitch;
  - after the 200 ms stop the component either resumes, the plugin re-arming the watchdog and the
    joint taking up the command it still has, or faults. Record which, and whether a resume is
    acceptable on the arm or a trip must hold the joint until a new command.
- **Result:** __________ **Date / tester / commits:** __________
