# Development Log

Running record of nontrivial problems on Glitch MK2, written up STAR-style
(Situation, Task, Action, Result) so they're reusable later — both for the
IEEE-style project workbook and for interview prep.

---

## 2026-08-29 — `ros2 launch` tab completion silently broken

**Situation:** `ros2 launch <package> <TAB>` did nothing in this workspace —
no package or launch-file autocompletion, no error either.

**Task:** Figure out why completion wasn't registering and fix it without
breaking the other workspace (`homer_ws`) that shares this machine.

**Action:** Traced how `ros2`'s argcomplete hook actually gets registered:
it lives in `/opt/ros/jazzy/share/ros2cli/environment/ros2-argcomplete.bash`,
an environment hook that only runs when `/opt/ros/jazzy/setup.bash` (the
underlay) is sourced. `~/.bashrc` was sourcing `~/homer_ws/install/local_setup.bash`
directly and never sourced the underlay at all — confirmed with a fresh shell:
`ROS_DISTRO` was empty and `ros2` wasn't even on `PATH`. Added
`source /opt/ros/jazzy/setup.bash` before the workspace line in `.bashrc`.

**Result:** Fresh shells now have `ROS_DISTRO=jazzy`, `ros2` resolves, and
tab completion registers (`complete -p ros2` shows the argcomplete hook).
Per-terminal, this workspace's overlay (`install/setup.bash`) still needs to
be sourced manually for *this package's* launch files to show up in
completion — only the underlay got promoted to `.bashrc`, on purpose, so
switching between workspaces on this machine still works.

---

## 2026-08-29 — Xbox controller teleop: sluggish, then choppy, then coasting

**Situation:** Driving with the Xbox controller, there was a noticeable lag
between stick input and the robot actually moving.

**Task:** Find where in the stick → wheel pipeline the delay was coming
from, and fix it — through several rounds of "fixed one thing, revealed the
next" as the real behavior showed up on hardware.

**Action (iteration 1 — find the bottleneck):** Traced the full command path:
`joy_node` → `teleop_twist_joy` → `cmd_vel` → `pico_interface.py` (Pi, over
USB serial) → `main.py` on the Pico → `regulated_wheel.py`'s velocity PID →
PWM. Every stage was sub-20ms except one: `regulate_velocity()` clamped the
PID's output *increment* to `±0.03` duty per 20ms tick, regardless of error
size — an accidental slew-rate limiter capping ramp-up at 1.5 duty/sec, i.e.
hundreds of ms to visibly respond to any real stick input.

**Result (1):** Loosened the clamp to `±0.15`. Confirmed responsiveness
improved for a large step command, but normal driving (0.15–0.35 m/s per
`gamepad.yaml`) still felt sluggish.

**Action (iteration 2 — the clamp wasn't actually binding at normal speeds):**
Did the math: at the gamepad's real operating speeds, `k_p * error` never
got close to the new clamp — `k_p` itself was too low, and the controller's
structure (`duty += increment`, an incremental/integral form) meant duty had
to "wind up" from zero on every new command with no term that let it jump
immediately. Added a feedforward term (`duty ≈ velocity / MAX_VEL`) applied
the instant a new target arrives, with the PID reduced to a trim on top.

**Result (2):** Much more responsive — but now choppy when smoothly
sweeping the stick in a circle (combined linear + angular), and it visibly
paused at the crossover points during turns.

**Action (iteration 3 — feedforward snap was fighting continuous input):**
The feedforward jump only fired on an exact value change, which happens on
almost every sample during smooth analog input — so `duty` kept getting
stomped back to a raw estimate ~75x/second instead of blending smoothly.
Separately, a special-cased hard `stop()` on `ref_lin_vel == 0.0` caused an
abrupt cutoff whenever a wheel's target crossed zero mid-turn. Restructured:
feedforward is now recomputed fresh every tick from the live target (so it
tracks continuously), the PID became a persistent `trim` term added on top
instead of something reset every sample, and the hard zero-stop was removed
in favor of letting feedforward + trim settle to zero naturally.

**Result (3):** Circle-drag smoothness fixed — but a new symptom appeared:
wheels kept coasting briefly after releasing the controller, and direction
reversals during turns were still slow.

**Action (iteration 4 — stale trim crossing a direction change):** Root
cause was the same in both cases: `trim` is a *persistent* correction, and
it doesn't know when the wheel's required direction has flipped. On release,
feedforward snaps to 0 instantly but the leftover `trim` from driving kept
`duty` nonzero until it slowly decayed. On a reversal, the stale trim from
the old direction was partially canceling the new feedforward. Fixed by
resetting `trim` (and the PID error terms) whenever `sign(ref_lin_vel)`
actually changes — covering release (sign goes to 0) and reversal (sign
flips) with the same check.

**Result (4):** Release now stops the wheel within one control tick
(≤20ms), and direction reversals are immediate since feedforward alone
drives the new direction while trim rebuilds cleanly from zero. Called
"good enough to debug on" as of this writing — still open follow-ups below.

**Open follow-ups:**
- `FF_GAIN = 1/MAX_LIN_VEL` is a linear guess, not measured. `sentient_wheel.py`
  already has a duty-sweep test in its `__main__` block — running that and
  fitting the real duty→velocity curve (especially the stiction deadband at
  low duty) would make the feedforward jump land much closer to correct.
- `k_p`/`k_d` and the trim cap (`±0.3`) haven't been retuned since the
  controller was restructured around feedforward + trim.
