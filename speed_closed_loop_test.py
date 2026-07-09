# ESP32 MicroPython speed closed-loop test for the current line follower.
# Put the car on a stand so both wheels are off the ground before running.

from machine import Pin, PWM
from time import sleep_ms, ticks_ms, ticks_diff

try:
    from machine import Encoder as HardwareEncoder
except ImportError:
    HardwareEncoder = None


# ============================================================
# Motor pins and PWM: same as main.py
# ============================================================

LEFT_IN1_PIN = 15
LEFT_IN2_PIN = 13
RIGHT_IN1_PIN = 25
RIGHT_IN2_PIN = 14

LEFT_INVERT = False
RIGHT_INVERT = True

PWM_FREQ = 20000
PWM_MAX = 1000

MAX_FORWARD_LEFT = 990
MAX_FORWARD_RIGHT = 990
MAX_REVERSE_LEFT = 750
MAX_REVERSE_RIGHT = 750


# ============================================================
# Encoder and speed closed-loop parameters: same as main.py
# ============================================================

LEFT_ENCODER_A_PIN = 16
LEFT_ENCODER_B_PIN = 17
RIGHT_ENCODER_A_PIN = 18
RIGHT_ENCODER_B_PIN = 19

LEFT_COUNTS_PER_REV = 10203.03
RIGHT_COUNTS_PER_REV = 10200.10
LEFT_ENCODER_SIGN = -1
RIGHT_ENCODER_SIGN = 1
ENCODER_PHASES = 1
ENCODER_FILTER_NS = 1000

SPEED_PERIOD_MS = 10

CLOSED_LOOP_REFERENCE_COMMAND = 900.0
CLOSED_LOOP_REFERENCE_RPM = 286.0
CLOSED_LOOP_MAX_RPM = 426.0
CLOSED_LOOP_MIN_ACTIVE_RPM = 18.0
CLOSED_LOOP_TARGET_STEP_RPM = 84.0

LEFT_FF_OFFSET = 575.08
LEFT_FF_SLOPE = 0.8504
LEFT_LOAD_BIAS = 18.0
RIGHT_FF_OFFSET = 562.07
RIGHT_FF_SLOPE = 0.8466
RIGHT_LOAD_BIAS = 16.0

LEFT_SPEED_KP = 0.40
LEFT_SPEED_KI = 1.00
RIGHT_SPEED_KP = 0.40
RIGHT_SPEED_KI = 1.00

SPEED_INTEGRAL_LIMIT = 260.0
SPEED_FILTER_ALPHA = 0.28
SPEED_TARGET_DROP_RESET_RPM = 24.0
SPEED_OVERSPEED_DECAY_RPM = 25.0
SPEED_OVERSPEED_INTEGRAL_DECAY = 0.60

LEFT_START_BOOST_PWM = 660
RIGHT_START_BOOST_PWM = 650
START_BOOST_MAX_MS = 55
START_BOOST_EXIT_RPM = 18.0
RESTART_ZERO_TIME_MS = 120
TARGET_STOP_RPM = 1.0


# ============================================================
# Test settings
# ============================================================

LOG_PERIOD_MS = 200
STEP_DURATION_MS = 3000
STOP_BETWEEN_STEPS_MS = 600

# name, left_command, right_command, duration_ms
TEST_STEPS = (
    ("both_600", 600, 600, STEP_DURATION_MS),
    ("both_750", 750, 750, STEP_DURATION_MS),
    ("both_900", 900, 900, STEP_DURATION_MS),
    ("both_985", 985, 985, STEP_DURATION_MS),
    ("left_750", 750, 0, 2200),
    ("right_750", 0, 750, 2200),
)


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def move_towards(current, target, maximum_change):
    difference = target - current
    if difference > maximum_change:
        return current + maximum_change
    if difference < -maximum_change:
        return current - maximum_change
    return target


class Motor:

    def __init__(self, in1_pin, in2_pin, invert, max_forward, max_reverse):
        self.pin1 = Pin(in1_pin, Pin.OUT, value=0)
        self.pin2 = Pin(in2_pin, Pin.OUT, value=0)
        self.pwm1 = PWM(self.pin1, freq=PWM_FREQ)
        self.pwm2 = PWM(self.pin2, freq=PWM_FREQ)
        self.invert = invert
        self.max_forward = max_forward
        self.max_reverse = max_reverse
        self.has_duty_u16 = hasattr(self.pwm1, "duty_u16")
        self.stop()

    def _write_pwm(self, pwm, command):
        command = int(clamp(command, 0, PWM_MAX))
        if self.has_duty_u16:
            pwm.duty_u16(command * 65535 // PWM_MAX)
        else:
            pwm.duty(command * 1023 // PWM_MAX)

    def set(self, command, bypass_limit=False):
        command = int(command)
        if bypass_limit:
            command = clamp(command, -PWM_MAX, PWM_MAX)
        else:
            command = clamp(command, -self.max_reverse, self.max_forward)

        if self.invert:
            command = -command

        if command > 0:
            self._write_pwm(self.pwm1, command)
            self._write_pwm(self.pwm2, 0)
        elif command < 0:
            self._write_pwm(self.pwm1, 0)
            self._write_pwm(self.pwm2, -command)
        else:
            self.stop()

    def stop(self):
        self._write_pwm(self.pwm1, 0)
        self._write_pwm(self.pwm2, 0)


class EncoderCounter:

    def __init__(self, encoder_id, pin_a_number, pin_b_number):
        self.hardware = None
        self.using_hardware = False
        self.software_count = 0

        if HardwareEncoder is not None:
            pin_a = Pin(pin_a_number, Pin.IN, Pin.PULL_UP)
            pin_b = Pin(pin_b_number, Pin.IN, Pin.PULL_UP)
            try:
                self.hardware = HardwareEncoder(
                    encoder_id,
                    pin_a,
                    pin_b,
                    phases=ENCODER_PHASES,
                    filter_ns=ENCODER_FILTER_NS
                )
                self.using_hardware = True
                return
            except TypeError:
                pass
            except Exception:
                pass

            try:
                self.hardware = HardwareEncoder(
                    pin_a,
                    pin_b,
                    phases=ENCODER_PHASES,
                    filter_ns=ENCODER_FILTER_NS
                )
                self.using_hardware = True
                return
            except Exception:
                pass

        self.pin_a = Pin(pin_a_number, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b_number, Pin.IN, Pin.PULL_UP)
        self.last_state = (self.pin_a.value() << 1) | self.pin_b.value()
        trigger = Pin.IRQ_RISING | Pin.IRQ_FALLING
        self.pin_a.irq(trigger=trigger, handler=self._update_software_count)
        self.pin_b.irq(trigger=trigger, handler=self._update_software_count)

    def _update_software_count(self, pin):
        state = (self.pin_a.value() << 1) | self.pin_b.value()
        transition = (self.last_state << 2) | state
        if transition in (1, 7, 14, 8):
            self.software_count += 1
        elif transition in (2, 11, 13, 4):
            self.software_count -= 1
        self.last_state = state

    def value(self, new_value=None):
        if self.using_hardware:
            if new_value is None:
                return self.hardware.value()
            return self.hardware.value(new_value)

        if new_value is None:
            return self.software_count

        self.software_count = int(new_value)
        self.last_state = (self.pin_a.value() << 1) | self.pin_b.value()
        return self.software_count


class SignedSpeedController:

    def __init__(self, kp, ki, ff_offset, ff_slope, load_bias, start_boost_pwm):
        self.kp = kp
        self.ki = ki
        self.ff_offset = ff_offset
        self.ff_slope = ff_slope
        self.load_bias = load_bias
        self.start_boost_pwm = start_boost_pwm
        self.integral_pwm = 0.0
        self.last_abs_target = 0.0
        self.last_sign = 0
        self.zero_since_ms = ticks_ms()
        self.boost_active = False
        self.boost_start_ms = ticks_ms()

    def reset(self, now=None):
        if now is None:
            now = ticks_ms()
        self.integral_pwm = 0.0
        self.last_abs_target = 0.0
        self.last_sign = 0
        self.zero_since_ms = now - RESTART_ZERO_TIME_MS
        self.boost_active = False
        self.boost_start_ms = now

    def feedforward(self, abs_target_rpm):
        if abs_target_rpm <= TARGET_STOP_RPM:
            return 0.0
        value = self.ff_offset + self.ff_slope * abs_target_rpm + self.load_bias
        return clamp(value, 0.0, float(PWM_MAX))

    def update(self, target_rpm, measured_rpm, dt_seconds, now):
        if target_rpm > TARGET_STOP_RPM:
            target_sign = 1
        elif target_rpm < -TARGET_STOP_RPM:
            target_sign = -1
        else:
            if self.last_abs_target > TARGET_STOP_RPM:
                self.zero_since_ms = now
            self.last_abs_target = 0.0
            self.last_sign = 0
            self.integral_pwm = 0.0
            self.boost_active = False
            return 0

        abs_target = abs(target_rpm)
        if self.last_abs_target <= TARGET_STOP_RPM or self.last_sign != target_sign:
            zero_duration = ticks_diff(now, self.zero_since_ms)
            if zero_duration >= RESTART_ZERO_TIME_MS and abs(measured_rpm) < 5.0:
                self.boost_active = True
                self.boost_start_ms = now
            self.integral_pwm = 0.0

        previous_abs_target = self.last_abs_target
        self.last_abs_target = abs_target
        self.last_sign = target_sign
        measured_aligned = measured_rpm * target_sign

        if self.boost_active:
            boost_elapsed = ticks_diff(now, self.boost_start_ms)
            reached_speed = measured_aligned >= START_BOOST_EXIT_RPM
            if boost_elapsed >= START_BOOST_MAX_MS or reached_speed:
                self.boost_active = False
                self.integral_pwm = 0.0
            else:
                output = max(self.start_boost_pwm, self.feedforward(abs_target))
                return target_sign * int(clamp(output, 0, PWM_MAX))

        if (
            previous_abs_target > TARGET_STOP_RPM and
            previous_abs_target - abs_target >= SPEED_TARGET_DROP_RESET_RPM and
            self.integral_pwm > 0.0
        ):
            self.integral_pwm = 0.0

        if (
            measured_aligned - abs_target >= SPEED_OVERSPEED_DECAY_RPM and
            self.integral_pwm > 0.0
        ):
            self.integral_pwm *= SPEED_OVERSPEED_INTEGRAL_DECAY

        error = abs_target - measured_aligned
        proportional_pwm = self.kp * error
        candidate_integral = self.integral_pwm + self.ki * error * dt_seconds
        candidate_integral = clamp(
            candidate_integral,
            -SPEED_INTEGRAL_LIMIT,
            SPEED_INTEGRAL_LIMIT
        )
        ff = self.feedforward(abs_target)
        candidate_output = ff + proportional_pwm + candidate_integral

        allow_integral = False
        if 0.0 < candidate_output < PWM_MAX:
            allow_integral = True
        elif candidate_output >= PWM_MAX and error < 0:
            allow_integral = True
        elif candidate_output <= 0 and error > 0:
            allow_integral = True

        if allow_integral:
            self.integral_pwm = candidate_integral

        output = ff + proportional_pwm + self.integral_pwm
        return target_sign * int(clamp(output, 0, PWM_MAX))


left_motor = Motor(
    LEFT_IN1_PIN,
    LEFT_IN2_PIN,
    LEFT_INVERT,
    MAX_FORWARD_LEFT,
    MAX_REVERSE_LEFT
)
right_motor = Motor(
    RIGHT_IN1_PIN,
    RIGHT_IN2_PIN,
    RIGHT_INVERT,
    MAX_FORWARD_RIGHT,
    MAX_REVERSE_RIGHT
)

left_encoder = EncoderCounter(0, LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN)
right_encoder = EncoderCounter(1, RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN)

left_speed_controller = SignedSpeedController(
    LEFT_SPEED_KP,
    LEFT_SPEED_KI,
    LEFT_FF_OFFSET,
    LEFT_FF_SLOPE,
    LEFT_LOAD_BIAS,
    LEFT_START_BOOST_PWM
)
right_speed_controller = SignedSpeedController(
    RIGHT_SPEED_KP,
    RIGHT_SPEED_KI,
    RIGHT_FF_OFFSET,
    RIGHT_FF_SLOPE,
    RIGHT_LOAD_BIAS,
    RIGHT_START_BOOST_PWM
)

left_desired_rpm = 0.0
right_desired_rpm = 0.0
left_ramped_rpm = 0.0
right_ramped_rpm = 0.0
left_measured_rpm = 0.0
right_measured_rpm = 0.0
left_closed_loop_pwm = 0
right_closed_loop_pwm = 0
last_speed_update_ms = ticks_ms() - SPEED_PERIOD_MS
last_left_count = 0
last_right_count = 0


def calculate_rpm(delta_count, dt_ms, counts_per_rev):
    if dt_ms <= 0:
        return 0.0
    return delta_count * 60000.0 / (counts_per_rev * dt_ms)


def command_to_target_rpm(command):
    command = float(command)
    if command > 0:
        sign = 1.0
    elif command < 0:
        sign = -1.0
    else:
        return 0.0

    target = abs(command) * CLOSED_LOOP_REFERENCE_RPM / CLOSED_LOOP_REFERENCE_COMMAND
    if target < CLOSED_LOOP_MIN_ACTIVE_RPM:
        return 0.0
    return sign * clamp(target, 0.0, CLOSED_LOOP_MAX_RPM)


def read_left_count():
    return left_encoder.value() * LEFT_ENCODER_SIGN


def read_right_count():
    return right_encoder.value() * RIGHT_ENCODER_SIGN


def reset_speed_loop(now=None):
    global left_desired_rpm
    global right_desired_rpm
    global left_ramped_rpm
    global right_ramped_rpm
    global left_measured_rpm
    global right_measured_rpm
    global left_closed_loop_pwm
    global right_closed_loop_pwm
    global last_speed_update_ms
    global last_left_count
    global last_right_count

    if now is None:
        now = ticks_ms()

    left_desired_rpm = 0.0
    right_desired_rpm = 0.0
    left_ramped_rpm = 0.0
    right_ramped_rpm = 0.0
    left_measured_rpm = 0.0
    right_measured_rpm = 0.0
    left_closed_loop_pwm = 0
    right_closed_loop_pwm = 0

    left_speed_controller.reset(now)
    right_speed_controller.reset(now)

    try:
        left_encoder.value(0)
        right_encoder.value(0)
    except Exception:
        pass

    last_left_count = read_left_count()
    last_right_count = read_right_count()
    last_speed_update_ms = now - SPEED_PERIOD_MS


def update_speed_closed_loop(now=None):
    global left_ramped_rpm
    global right_ramped_rpm
    global left_measured_rpm
    global right_measured_rpm
    global left_closed_loop_pwm
    global right_closed_loop_pwm
    global last_speed_update_ms
    global last_left_count
    global last_right_count

    if now is None:
        now = ticks_ms()

    if ticks_diff(now, last_speed_update_ms) < SPEED_PERIOD_MS:
        return

    current_left_count = read_left_count()
    current_right_count = read_right_count()
    dt_ms = ticks_diff(now, last_speed_update_ms)
    if dt_ms <= 0:
        dt_ms = SPEED_PERIOD_MS

    left_raw_rpm = calculate_rpm(
        current_left_count - last_left_count,
        dt_ms,
        LEFT_COUNTS_PER_REV
    )
    right_raw_rpm = calculate_rpm(
        current_right_count - last_right_count,
        dt_ms,
        RIGHT_COUNTS_PER_REV
    )

    left_measured_rpm += SPEED_FILTER_ALPHA * (left_raw_rpm - left_measured_rpm)
    right_measured_rpm += SPEED_FILTER_ALPHA * (right_raw_rpm - right_measured_rpm)

    left_ramped_rpm = move_towards(
        left_ramped_rpm,
        left_desired_rpm,
        CLOSED_LOOP_TARGET_STEP_RPM
    )
    right_ramped_rpm = move_towards(
        right_ramped_rpm,
        right_desired_rpm,
        CLOSED_LOOP_TARGET_STEP_RPM
    )

    dt_seconds = dt_ms / 1000.0
    left_closed_loop_pwm = left_speed_controller.update(
        left_ramped_rpm,
        left_measured_rpm,
        dt_seconds,
        now
    )
    right_closed_loop_pwm = right_speed_controller.update(
        right_ramped_rpm,
        right_measured_rpm,
        dt_seconds,
        now
    )

    left_motor.set(left_closed_loop_pwm)
    right_motor.set(right_closed_loop_pwm)
    last_left_count = current_left_count
    last_right_count = current_right_count
    last_speed_update_ms = now


def set_wheel_commands(left_command, right_command):
    global left_desired_rpm
    global right_desired_rpm

    left_desired_rpm = command_to_target_rpm(left_command)
    right_desired_rpm = command_to_target_rpm(right_command)
    update_speed_closed_loop()


def stop_all():
    reset_speed_loop()
    left_motor.stop()
    right_motor.stop()


def print_header():
    print("=" * 88)
    print("speed closed-loop test")
    print("Put the car on a stand: wheels off the ground.")
    print("encoder mode: left=%s right=%s" % (
        "hardware" if left_encoder.using_hardware else "irq",
        "hardware" if right_encoder.using_hardware else "irq"
    ))
    print("cmd -> target: 900 command = %.1f rpm" % CLOSED_LOOP_REFERENCE_RPM)
    print("=" * 88)
    print("step,t_ms,L_tgt,L_meas,L_pwm,L_err,R_tgt,R_meas,R_pwm,R_err")


def log_status(step_name, start_ms):
    left_error = left_ramped_rpm - left_measured_rpm
    right_error = right_ramped_rpm - right_measured_rpm
    print("%s,%d,%.1f,%.1f,%d,%.1f,%.1f,%.1f,%d,%.1f" % (
        step_name,
        ticks_diff(ticks_ms(), start_ms),
        left_ramped_rpm,
        left_measured_rpm,
        left_closed_loop_pwm,
        left_error,
        right_ramped_rpm,
        right_measured_rpm,
        right_closed_loop_pwm,
        right_error
    ))


def run_step(step_name, left_command, right_command, duration_ms):
    start_ms = ticks_ms()
    last_log_ms = start_ms - LOG_PERIOD_MS
    while ticks_diff(ticks_ms(), start_ms) < duration_ms:
        set_wheel_commands(left_command, right_command)
        now = ticks_ms()
        if ticks_diff(now, last_log_ms) >= LOG_PERIOD_MS:
            log_status(step_name, start_ms)
            last_log_ms = now
        sleep_ms(2)


def settle_stop():
    stop_all()
    sleep_ms(STOP_BETWEEN_STEPS_MS)
    reset_speed_loop()


def run_test():
    print_header()
    reset_speed_loop()
    sleep_ms(300)
    for step in TEST_STEPS:
        name, left_command, right_command, duration_ms = step
        print("")
        print("STEP %s left_cmd=%d right_cmd=%d" % (
            name,
            left_command,
            right_command
        ))
        run_step(name, left_command, right_command, duration_ms)
        settle_stop()

    print("")
    print("done. Check that measured rpm follows target rpm with similar left/right error.")


try:
    run_test()
except KeyboardInterrupt:
    print("")
    print("stopped by user")
finally:
    stop_all()
