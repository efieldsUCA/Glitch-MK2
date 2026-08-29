import sys
from utime import ticks_us, ticks_diff
import select
from machine import freq
from diff_drive_controller import DiffDriveController

freq(240_000_000)

WATCHDOG_US = 500_000   # 500 ms

mobile_base = DiffDriveController(
    left_ids=((16, 18, 17), (26, 27)),
    right_ids=((21, 19, 20), (6, 7))
)
pico_messenger = select.poll()
pico_messenger.register(sys.stdin, select.POLLIN)

TX_PERIOD_US = 16_667   # 60 Hz odometry
targ_lin_vel, targ_ang_vel = 0.0, 0.0
last_us = ticks_us()
last_cmd_us = ticks_us()

while True:
    now_us = ticks_us()

    if ticks_diff(now_us, last_cmd_us) > WATCHDOG_US:
        if targ_lin_vel != 0.0 or targ_ang_vel != 0.0:
            targ_lin_vel = 0.0
            targ_ang_vel = 0.0
            mobile_base.set_vels(0.0, 0.0)

    if ticks_diff(now_us, last_us) >= TX_PERIOD_US:
        meas_lin_vel, meas_ang_vel = mobile_base.get_vels()
        print(f"{meas_lin_vel:.3f},{meas_ang_vel:.3f}")
        last_us = now_us

    if pico_messenger.poll(0):
        in_msg = sys.stdin.readline().strip()
        if not in_msg:
            continue
        last_cmd_us = ticks_us()
        try:
            parts = in_msg.split(",")
            if len(parts) == 2:
                targ_lin_vel = float(parts[0])
                targ_ang_vel = float(parts[1])
                mobile_base.set_vels(targ_lin_vel, targ_ang_vel)
        except Exception as e:
            print(f"ERR: {e}")
