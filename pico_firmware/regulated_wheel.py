from sentient_wheel import SentientWheel
from machine import Timer


def clamp(x, min_val, max_val):
    return max(min_val, min(x, max_val))


def sign(x):
    return (x > 0) - (x < 0)


class RegulatedWheel(SentientWheel):
    def __init__(self, driver_ids: list | tuple, encoder_ids: list | tuple) -> None:
        super().__init__(driver_ids, encoder_ids)
        # Constants
        self.k_p = 0.25
        self.k_i = 0.0
        self.k_d = 0.05
        self.reg_freq = 50  # Hz
        self.MAX_LIN_VEL = 0.8  # m/s, matches set_velocity's clamp
        self.FF_GAIN = 1.0 / self.MAX_LIN_VEL  # rough duty-per-(m/s) estimate, retune from measured duty/velocity curve
        # Variables
        self.reg_vel_counter = 0
        self.duty = 0.0
        self.trim = 0.0  # PID correction on top of feedforward
        self.error = 0.0
        self.prev_error = 0.0
        self.error_inte = 0.0  # integral
        self.error_diff = 0.0  # differentiation
        self.ref_lin_vel = 0.0
        # PID controller config
        self.vel_reg_timer = Timer(
            freq=self.reg_freq,
            mode=Timer.PERIODIC,
            callback=self.regulate_velocity,
        )

    def regulate_velocity(self, timer):
        if self.reg_vel_counter > self.reg_freq:  # stale command watchdog
            self.stop()
            self.trim = 0.0
            self.prev_error = 0.0
            self.error_inte = 0.0
        else:
            self.error = self.ref_lin_vel - self.meas_lin_vel
            self.error_inte += self.error
            self.error_diff = self.error - self.prev_error
            self.prev_error = self.error
            inc_trim = (
                self.k_p * self.error
                + self.k_i * self.error_inte
                + self.k_d * self.error_diff
            )
            inc_trim = clamp(inc_trim, -0.15, 0.15)  # ~130ms full-authority trim ramp at reg_freq
            self.trim = clamp(self.trim + inc_trim, -0.3, 0.3)  # cap trim's authority over feedforward
            ff_duty = clamp(self.ref_lin_vel * self.FF_GAIN, -1.0, 1.0)
            self.duty = clamp(ff_duty + self.trim, -1.0, 1.0)
            if self.duty > 0:
                self.forward(self.duty)
            elif self.duty < 0:
                self.backward(-self.duty)
            else:
                self.stop()
            self.reg_vel_counter += 1

    def set_velocity(self, ref_lin_vel):
        ref_lin_vel = clamp(ref_lin_vel, -self.MAX_LIN_VEL, self.MAX_LIN_VEL)
        if sign(ref_lin_vel) != sign(self.ref_lin_vel):
            self.trim = 0.0
            self.prev_error = 0.0
            self.error_inte = 0.0
        self.ref_lin_vel = ref_lin_vel
        self.reg_vel_counter = 0


if __name__ == "__main__":
    """ Set target velocity to different values to tune PID"""
    from utime import sleep
    from machine import Pin

    # regw = RegulatedWheel(
    #     driver_ids=(16, 18, 17),
    #     encoder_ids=(26, 27),
    # )  # left
    regw = RegulatedWheel(
        driver_ids=(21, 19, 20),
        encoder_ids=(6, 7),
    )  # right

    # LOOP
    for i in range(100):
        if i == 25:  # step up @ t=0.5s
            regw.set_velocity(0.6)
        elif i == 80:  # step down @ t=1.6s
            regw.set_velocity(0.0)
        print(
            f"Reference velocity={regw.ref_lin_vel} m/s, Measured velocity={regw.meas_lin_vel} m/s"
        )
        sleep(0.02)
    # Terminate
    regw.disable()
    print("Wheel disabled")

