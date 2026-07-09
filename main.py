# ESP32 MicroPython five-sensor line follower.
# Core tracking logic is ported from the Arduino five-photoelectric-sensor
# project: thresholded sensors, weighted five-in-one position PID, and
# Last_Flag lost-line recovery.

from machine import Pin, PWM, ADC
from time import sleep_ms, ticks_ms, ticks_diff
import gc

try:
    from machine import Encoder as HardwareEncoder
except ImportError:
    HardwareEncoder = None

try:
    import ujson as json
except ImportError:
    import json


# ============================================================
# Runtime settings
# ============================================================

START_DELAY_MS = 2000
START_NO_LINE_FORWARD_MS = 500
START_NO_LINE_COMMAND = 520
STARTUP_SOFT_START_MS = 800
STARTUP_SOFT_START_COMMAND = 620
CONTROL_PERIOD_MS = 5
DEBUG_PRINT = False
DEBUG_PERIOD_MS = 100


# ============================================================
# Motor pins and PWM
# ============================================================

LEFT_IN1_PIN = 15
LEFT_IN2_PIN = 13
RIGHT_IN1_PIN = 25
RIGHT_IN2_PIN = 14

LEFT_INVERT = False
RIGHT_INVERT = True

PWM_FREQ = 20000
PWM_MAX = 1000

BASE_COMMAND = 969
MAX_FORWARD_LEFT = 971
MAX_FORWARD_RIGHT = 971
MAX_REVERSE_LEFT = 750
MAX_REVERSE_RIGHT = 750


# ============================================================
# Encoder and speed closed loop
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

USE_SPEED_CLOSED_LOOP = True
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
# Sensor calibration
# ============================================================

SENSOR_PINS = (27, 33, 32, 35, 34)
SENSOR_POSITION_MM = (-35.0, -15.0, 0.0, 15.0, 35.0)
POSITION_WEIGHTS = (0, 1000, 2000, 3000, 4000)
LINE_CENTER = 2000

CONFIG_FILE = "line_track_config.json"

WHITE_VALUES = (104.914, 59.683, 57.817, 66.747, 139.306)
BLACK_VALUES = (1967.5, 815.5, 239.5, 1295.5, 2735.5)
SENSOR_THRESHOLDS = (943.078, 399.801, 139.574, 619.686, 1307.594)
LINE_IS_HIGH = True
CALIBRATION_FROM_CONFIG = False

SENSOR_AVERAGE_SAMPLES = 4
BINARY_ACTIVE_NORM = 0.42
LINE_SUM_MIN = 180
POSITION_SUM_MIN = 80


# ============================================================
# Arduino-style PID tracking
# ============================================================

PID_KP = 0.227
PID_KI = 0.0
PID_KD = 0.24
PID_INTEGRAL_LIMIT = 10000.0
COMMAND_FILTER_ALPHA = 0.55
DERIVATIVE_FILTER_ALPHA = 0.80
CURVE_SLOWDOWN_START_ERROR = 180.0
CURVE_SLOWDOWN_FULL_ERROR = 1750.0
CURVE_SLOWDOWN_MAX_COMMAND = 120

# 出弯防抖窗口：检测到刚从较大弯道回中后，短时间降速并压小转向。
# 只作用于普通循迹 follow_line_step，不改变十字、直角强转和丢线逻辑。
POST_CURVE_DAMP_ENABLE = True
POST_CURVE_ENTER_ERROR = 520.0
POST_CURVE_EXIT_ERROR = 180.0
POST_CURVE_DAMP_MS = 110
POST_CURVE_BASE_COMMAND = 850
POST_CURVE_TURN_SCALE = 0.90
POST_CURVE_DERIVATIVE_SCALE = 0.7
POST_CURVE_MAX_TURN = 200.0


# ============================================================
# Cross and right-angle handling
# ============================================================

CROSS_HOLD_MS = 100
CROSS_HOLD_COMMAND = 760
CROSS_HOLD_CORRECTION_KP = 0.070
CROSS_HOLD_MAX_CORRECTION = 80
CROSS_HOLD_ERROR_LIMIT = 900.0
FIRST_CROSS_RIGHT_BIAS = 400
CROSS_COOLDOWN_MS = 70
CROSS_SUM_MIN = 1850
CROSS_SIDE_SUM_MIN = 520
CROSS_SKEW_OPPOSITE_MIN = 210
CROSS_SKEW_OPPOSITE_SUM_MIN = 260
CROSS_APPROACH_GUARD_MS = 28
CROSS_APPROACH_COMMAND = 930
CROSS_APPROACH_SUM_MIN = 1300
CROSS_APPROACH_CENTER_MIN = 360
CROSS_APPROACH_TAIL_SUM_MIN = 1180

CORNER_FULL_POWER_MS = 118
CORNER_CONFIRM_FRAMES = 2
CORNER_MIN_HOLD_MS = 82
CORNER_MAX_HOLD_MS = 340
CORNER_REACQUIRE_FRAMES = 1
CORNER_EXIT_ERROR = 650
CORNER_OUTER_COMMAND = 1000
CORNER_REVERSE_COMMAND = 960
CORNER_SEARCH_OUTER_COMMAND = 865
CORNER_SEARCH_INNER_COMMAND = 0
CORNER_CENTER_MIN = 170
CORNER_EDGE_MIN = 290
CORNER_EDGE_DELTA = 40

# 直角弯急停：识别到直角后先短暂停车，再进入原强转逻辑。
# 只改变直角入口瞬间，不改直角强转功率、搜索功率和其他循迹逻辑。
CORNER_BRAKE_ENABLE = True
CORNER_BRAKE_MS = 10

LOST_OUTER_COMMAND = 880
LOST_REVERSE_COMMAND = 360
LOST_MIDDLE_COMMAND = 560


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


def curve_base_command(base_command, error):
    abs_error = abs(error)
    if abs_error <= CURVE_SLOWDOWN_START_ERROR:
        return base_command

    error_span = CURVE_SLOWDOWN_FULL_ERROR - CURVE_SLOWDOWN_START_ERROR
    if error_span <= 0:
        return base_command - CURVE_SLOWDOWN_MAX_COMMAND

    ratio = (abs_error - CURVE_SLOWDOWN_START_ERROR) / error_span
    ratio = clamp(ratio, 0.0, 1.0)
    slowdown = int(CURVE_SLOWDOWN_MAX_COMMAND * ratio)
    return base_command - slowdown


def cross_straight_commands(base_command, entry_error, right_bias=0.0):
    correction = entry_error * CROSS_HOLD_CORRECTION_KP + right_bias
    correction = clamp(
        correction,
        -CROSS_HOLD_MAX_CORRECTION,
        CROSS_HOLD_MAX_CORRECTION
    )

    left = base_command + correction
    right = base_command - correction
    return (
        int(clamp(left, 0, MAX_FORWARD_LEFT)),
        int(clamp(right, 0, MAX_FORWARD_RIGHT))
    )


def cross_hold_commands(entry_error, right_bias=0.0):
    return cross_straight_commands(CROSS_HOLD_COMMAND, entry_error, right_bias)


def cross_approach_commands(entry_error, right_bias=0.0):
    return cross_straight_commands(
        CROSS_APPROACH_COMMAND,
        entry_error,
        right_bias
    )


def is_circle_entry_cross(cross_count):
    # 两圈运行且每圈两个十字：
    # 0 = 第1圈进入大圆圈十字
    # 1 = 第1圈出大圆圈十字
    # 2 = 第2圈进入大圆圈十字
    # 3 = 第2圈出大圆圈十字
    return cross_count == 0 or cross_count == 2


def first_cross_bias(cross_count):
    if is_circle_entry_cross(cross_count):
        return FIRST_CROSS_RIGHT_BIAS
    return 0.0


def cross_entry_error(position):
    if abs(last_error) >= 50.0:
        entry_error = last_error
    elif position is not None:
        entry_error = position - LINE_CENTER
    else:
        entry_error = 0.0

    return clamp(
        entry_error,
        -CROSS_HOLD_ERROR_LIMIT,
        CROSS_HOLD_ERROR_LIMIT
    )


def startup_base_command(now, start_ms):
    elapsed = ticks_diff(now, start_ms)
    if elapsed <= 0:
        return STARTUP_SOFT_START_COMMAND
    if elapsed >= STARTUP_SOFT_START_MS:
        return BASE_COMMAND

    span = BASE_COMMAND - STARTUP_SOFT_START_COMMAND
    return int(STARTUP_SOFT_START_COMMAND + span * elapsed / STARTUP_SOFT_START_MS)


def load_tracking_config():
    global WHITE_VALUES
    global BLACK_VALUES
    global SENSOR_THRESHOLDS
    global LINE_IS_HIGH
    global CALIBRATION_FROM_CONFIG

    try:
        with open(CONFIG_FILE, "r") as file:
            config = json.load(file)

        if tuple(config.get("pins", ())) != SENSOR_PINS:
            return
        if tuple(float(v) for v in config.get("positions_mm", ())) != SENSOR_POSITION_MM:
            return

        channels = config.get("channels", ())
        if len(channels) != 5:
            return

        WHITE_VALUES = tuple(float(ch["white"]) for ch in channels)
        BLACK_VALUES = tuple(float(ch["black"]) for ch in channels)
        SENSOR_THRESHOLDS = tuple(float(ch["threshold"]) for ch in channels)
        LINE_IS_HIGH = bool(config.get("line_is_high", LINE_IS_HIGH))
        CALIBRATION_FROM_CONFIG = True
    except Exception:
        CALIBRATION_FROM_CONFIG = False


load_tracking_config()


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


def set_wheel_commands(left_command, right_command, bypass_limit=False):
    global left_desired_rpm
    global right_desired_rpm

    if not USE_SPEED_CLOSED_LOOP or bypass_limit:
        reset_speed_loop()
        left_motor.set(left_command, bypass_limit=bypass_limit)
        right_motor.set(right_command, bypass_limit=bypass_limit)
        return

    left_desired_rpm = command_to_target_rpm(left_command)
    right_desired_rpm = command_to_target_rpm(right_command)
    update_speed_closed_loop()


def stop_all():
    reset_speed_loop()
    left_motor.stop()
    right_motor.stop()


reset_speed_loop()


sensor_adcs = []
for pin_number in SENSOR_PINS:
    adc = ADC(Pin(pin_number))
    adc.atten(ADC.ATTN_11DB)
    try:
        adc.width(ADC.WIDTH_12BIT)
    except Exception:
        pass
    sensor_adcs.append(adc)

raw_values = [0, 0, 0, 0, 0]
p_values = [0, 0, 0, 0, 0]
normalized_values = [0.0, 0.0, 0.0, 0.0, 0.0]
s_values = [0, 0, 0, 0, 0]

last_flag = 0
pid_integral = 0.0
last_error = 0.0
last_derivative = 0.0
last_left_command = 0.0
last_right_command = 0.0
post_curve_armed = False
post_curve_damp_start_ms = ticks_ms() - POST_CURVE_DAMP_MS


def read_adc_average(adc):
    total = 0
    for _ in range(SENSOR_AVERAGE_SAMPLES):
        total += adc.read()
    return total / SENSOR_AVERAGE_SAMPLES


def normalize_sensor(raw, index):
    white = WHITE_VALUES[index]
    black = BLACK_VALUES[index]
    span = black - white
    if abs(span) < 1.0:
        return 0.0

    if LINE_IS_HIGH:
        value = (raw - white) / span
    else:
        value = (white - raw) / (white - black)

    return clamp(value, 0.0, 1.0)


def light_judge():
    for index in range(5):
        raw = read_adc_average(sensor_adcs[index])
        norm = normalize_sensor(raw, index)
        raw_values[index] = int(raw)
        normalized_values[index] = norm
        p_values[index] = int(norm * 1000)

        if LINE_IS_HIGH:
            active_raw = raw >= SENSOR_THRESHOLDS[index]
        else:
            active_raw = raw <= SENSOR_THRESHOLDS[index]

        active_norm = norm >= BINARY_ACTIVE_NORM
        s_values[index] = 1 if (active_raw or active_norm) else 0


def flag_judge():
    global last_flag

    if s_values[0]:
        last_flag = 1
    elif s_values[1]:
        last_flag = 2
    elif s_values[2]:
        last_flag = 3
    elif s_values[3]:
        last_flag = 4
    elif s_values[4]:
        last_flag = 5


def line_detected():
    return sum(s_values) > 0 or sum(p_values) >= LINE_SUM_MIN


def compute_position():
    total = sum(p_values)
    values = p_values

    if total < POSITION_SUM_MIN:
        values = [value * 1000 for value in s_values]
        total = sum(values)

    if total <= 0:
        return None

    weighted = 0
    for index in range(5):
        weighted += POSITION_WEIGHTS[index] * values[index]

    return weighted / total


def reset_post_curve_damp():
    global post_curve_armed
    global post_curve_damp_start_ms

    post_curve_armed = False
    post_curve_damp_start_ms = ticks_ms() - POST_CURVE_DAMP_MS


def update_post_curve_damp(error, now):
    global post_curve_armed
    global post_curve_damp_start_ms

    if not POST_CURVE_DAMP_ENABLE:
        return False

    abs_error = abs(error)
    if abs_error >= POST_CURVE_ENTER_ERROR:
        post_curve_armed = True
    elif post_curve_armed and abs_error <= POST_CURVE_EXIT_ERROR:
        post_curve_damp_start_ms = now
        post_curve_armed = False

    return ticks_diff(now, post_curve_damp_start_ms) < POST_CURVE_DAMP_MS


def reset_pid_history():
    global pid_integral
    global last_error
    global last_derivative
    global last_left_command
    global last_right_command

    pid_integral = 0.0
    last_error = 0.0
    last_derivative = 0.0
    last_left_command = 0.0
    last_right_command = 0.0
    reset_post_curve_damp()


def update_filtered_derivative(raw_derivative):
    global last_derivative

    last_derivative = (
        DERIVATIVE_FILTER_ALPHA * raw_derivative +
        (1.0 - DERIVATIVE_FILTER_ALPHA) * last_derivative
    )
    return last_derivative


def follow_line_step(base_command):
    global pid_integral
    global last_error
    global last_left_command
    global last_right_command

    position = compute_position()
    if position is None:
        return calculate_lost_commands(base_command) + (None,)

    error = position - LINE_CENTER

    if error * last_error < 0:
        pid_integral = 0.0
    else:
        pid_integral += error
        pid_integral = clamp(
            pid_integral,
            -PID_INTEGRAL_LIMIT,
            PID_INTEGRAL_LIMIT
        )

    now = ticks_ms()
    post_curve_damp_active = update_post_curve_damp(error, now)

    raw_derivative = error - last_error
    derivative = update_filtered_derivative(raw_derivative)

    if post_curve_damp_active:
        derivative *= POST_CURVE_DERIVATIVE_SCALE

    turn = PID_KP * error + PID_KI * pid_integral + PID_KD * derivative
    tracking_base_command = curve_base_command(base_command, error)

    if post_curve_damp_active:
        tracking_base_command = min(tracking_base_command, POST_CURVE_BASE_COMMAND)
        turn *= POST_CURVE_TURN_SCALE
        turn = clamp(turn, -POST_CURVE_MAX_TURN, POST_CURVE_MAX_TURN)

    left = tracking_base_command + turn
    right = tracking_base_command - turn

    left = (
        COMMAND_FILTER_ALPHA * left +
        (1.0 - COMMAND_FILTER_ALPHA) * last_left_command
    )
    right = (
        COMMAND_FILTER_ALPHA * right +
        (1.0 - COMMAND_FILTER_ALPHA) * last_right_command
    )

    left = clamp(left, -MAX_REVERSE_LEFT, MAX_FORWARD_LEFT)
    right = clamp(right, -MAX_REVERSE_RIGHT, MAX_FORWARD_RIGHT)

    last_error = error
    last_left_command = left
    last_right_command = right

    return int(left), int(right), error


def calculate_lost_commands(base_command):
    if last_flag == 1:
        return -LOST_REVERSE_COMMAND, LOST_OUTER_COMMAND
    if last_flag == 2:
        return int(base_command * 0.55), base_command
    if last_flag == 3:
        return LOST_MIDDLE_COMMAND, LOST_MIDDLE_COMMAND
    if last_flag == 4:
        return base_command, int(base_command * 0.55)
    if last_flag == 5:
        return LOST_OUTER_COMMAND, -LOST_REVERSE_COMMAND
    return 0, 0


def is_cross_detected():
    black_count = sum(s_values)
    outer_pair = s_values[0] and s_values[4]
    enough_total = sum(p_values) >= CROSS_SUM_MIN
    center_seen = s_values[1] or s_values[2] or s_values[3]
    left_side = (
        s_values[0] or
        s_values[1] or
        (p_values[0] + p_values[1] >= CROSS_SIDE_SUM_MIN)
    )
    right_side = (
        s_values[3] or
        s_values[4] or
        (p_values[3] + p_values[4] >= CROSS_SIDE_SUM_MIN)
    )
    side_bridge = center_seen and left_side and right_side
    left_skew = (
        s_values[0] and
        (s_values[1] or p_values[1] >= CROSS_SIDE_SUM_MIN // 2) and
        center_seen and
        (
            p_values[3] >= CROSS_SKEW_OPPOSITE_MIN or
            p_values[3] + p_values[4] >= CROSS_SKEW_OPPOSITE_SUM_MIN
        )
    )
    right_skew = (
        s_values[4] and
        (s_values[3] or p_values[3] >= CROSS_SIDE_SUM_MIN // 2) and
        center_seen and
        (
            p_values[1] >= CROSS_SKEW_OPPOSITE_MIN or
            p_values[0] + p_values[1] >= CROSS_SKEW_OPPOSITE_SUM_MIN
        )
    )

    return (
        black_count >= 4 or
        (outer_pair and (center_seen or enough_total)) or
        (side_bridge and (black_count >= 3 or enough_total)) or
        (enough_total and (left_skew or right_skew))
    )


def is_cross_approach_detected():
    if is_cross_detected():
        return False

    center_seen = s_values[2] or p_values[2] >= CROSS_APPROACH_CENTER_MIN
    left_pair = (
        (s_values[0] and s_values[1]) or
        (s_values[0] and p_values[1] >= CROSS_SIDE_SUM_MIN // 2) or
        (p_values[0] + p_values[1] >= CROSS_SIDE_SUM_MIN)
    )
    right_pair = (
        (s_values[3] and s_values[4]) or
        (s_values[4] and p_values[3] >= CROSS_SIDE_SUM_MIN // 2) or
        (p_values[3] + p_values[4] >= CROSS_SIDE_SUM_MIN)
    )
    enough_total = sum(p_values) >= CROSS_APPROACH_SUM_MIN

    return center_seen and (left_pair or right_pair) and enough_total


def is_cross_approach_tail_detected():
    if is_cross_detected():
        return False

    left_tail = s_values[0] and s_values[1] and not s_values[3] and not s_values[4]
    right_tail = s_values[3] and s_values[4] and not s_values[0] and not s_values[1]
    strong_left = p_values[0] + p_values[1] >= CROSS_APPROACH_TAIL_SUM_MIN
    strong_right = p_values[3] + p_values[4] >= CROSS_APPROACH_TAIL_SUM_MIN

    return (left_tail and strong_left) or (right_tail and strong_right)


def get_corner_direction(block_cross_approach=True):
    if is_cross_detected():
        return 0
    if block_cross_approach and is_cross_approach_detected():
        return 0

    left_pair = s_values[0] and s_values[1]
    right_pair = s_values[3] and s_values[4]
    left_edge = p_values[0]
    right_edge = p_values[4]
    center_seen = s_values[2] or p_values[2] >= CORNER_CENTER_MIN

    if left_pair and not s_values[4]:
        return -1
    if right_pair and not s_values[0]:
        return 1
    if (
        center_seen and
        left_edge >= CORNER_EDGE_MIN and
        left_edge > right_edge + CORNER_EDGE_DELTA
    ):
        return -1
    if (
        center_seen and
        right_edge >= CORNER_EDGE_MIN and
        right_edge > left_edge + CORNER_EDGE_DELTA
    ):
        return 1
    return 0


def corner_full_commands(direction):
    if direction < 0:
        return -CORNER_REVERSE_COMMAND, CORNER_OUTER_COMMAND
    return CORNER_OUTER_COMMAND, -CORNER_REVERSE_COMMAND


def corner_search_commands(direction):
    if direction < 0:
        return CORNER_SEARCH_INNER_COMMAND, CORNER_SEARCH_OUTER_COMMAND
    return CORNER_SEARCH_OUTER_COMMAND, CORNER_SEARCH_INNER_COMMAND


def track_loop():
    global last_flag

    print("=" * 70)
    print("ESP32 five-sensor line follower, Arduino PID core")
    print("Added: calibration, speed closed loop, corner, cross")
    print("base command:", BASE_COMMAND)
    print(
        "closed loop: %.0f command -> %.1f rpm, max %.1f rpm" %
        (
            CLOSED_LOOP_REFERENCE_COMMAND,
            CLOSED_LOOP_REFERENCE_RPM,
            CLOSED_LOOP_MAX_RPM
        )
    )
    print(
        "encoder mode: left=%s right=%s" %
        (
            "hardware" if left_encoder.using_hardware else "irq",
            "hardware" if right_encoder.using_hardware else "irq"
        )
    )
    print("calibration from config:", CALIBRATION_FROM_CONFIG)
    print("start in %d ms" % START_DELAY_MS)
    print("=" * 70)

    stop_all()
    for _ in range(20):
        light_judge()
        sleep_ms(4)

    sleep_ms(START_DELAY_MS)

    start_search_start = ticks_ms()
    last_loop_time = ticks_ms()
    last_debug_time = ticks_ms()
    last_seen_time = ticks_ms()
    seen_line_once = False

    cross_hold_active = False
    cross_hold_start = ticks_ms()
    cross_hold_error = 0.0
    cross_hold_bias = 0.0
    cross_hold_bypass = False
    cross_count = 0
    cross_cooldown_start = ticks_ms() - CROSS_COOLDOWN_MS
    cross_approach_active = False
    cross_approach_start = ticks_ms() - CROSS_APPROACH_GUARD_MS
    cross_approach_error = 0.0
    cross_approach_bypass = False

    corner_active = False
    corner_direction = 0
    corner_start = ticks_ms()
    corner_reacquire_frames = 0
    corner_candidate_direction = 0
    corner_candidate_frames = 0

    corner_brake_active = False
    corner_brake_direction = 0
    corner_brake_start = ticks_ms()

    while True:
        now = ticks_ms()
        elapsed = ticks_diff(now, last_loop_time)
        if elapsed < CONTROL_PERIOD_MS:
            sleep_ms(CONTROL_PERIOD_MS - elapsed)
            continue
        last_loop_time = now

        light_judge()
        detected = line_detected()
        state = "NORMAL"
        left_command = 0
        right_command = 0
        position = compute_position()
        error = None
        base_command = startup_base_command(now, start_search_start)

        if cross_hold_active:
            if ticks_diff(now, cross_hold_start) < CROSS_HOLD_MS:
                state = "CROSS_HOLD"
                left_command, right_command = cross_hold_commands(cross_hold_error, cross_hold_bias)
                set_wheel_commands(
                    left_command,
                    right_command,
                    bypass_limit=cross_hold_bypass
                )
            else:
                cross_hold_active = False
                cross_hold_error = 0.0
                cross_hold_bias = 0.0
                cross_hold_bypass = False
                cross_cooldown_start = now
                reset_pid_history()
            continue

        cross_cooldown_active = (
            ticks_diff(now, cross_cooldown_start) < CROSS_COOLDOWN_MS
        )

        if corner_brake_active:
            if ticks_diff(now, corner_brake_start) < CORNER_BRAKE_MS:
                state = "CORNER_BRAKE"
                left_command = 0
                right_command = 0
                set_wheel_commands(
                    left_command,
                    right_command,
                    bypass_limit=True
                )
                continue

            corner_brake_active = False
            corner_active = True
            corner_direction = corner_brake_direction
            corner_start = now
            corner_reacquire_frames = 0
            reset_pid_history()

        if corner_active:
            corner_elapsed = ticks_diff(now, corner_start)

            if corner_elapsed <= CORNER_FULL_POWER_MS:
                state = "CORNER_FULL"
                left_command, right_command = corner_full_commands(corner_direction)
                set_wheel_commands(
                    left_command,
                    right_command,
                    bypass_limit=True
                )
            elif corner_elapsed <= CORNER_MAX_HOLD_MS:
                state = "CORNER_SEARCH"
                left_command, right_command = corner_search_commands(corner_direction)
                set_wheel_commands(left_command, right_command)

                if (
                    corner_elapsed >= CORNER_MIN_HOLD_MS and
                    detected and
                    position is not None and
                    abs(position - LINE_CENTER) <= CORNER_EXIT_ERROR
                ):
                    corner_reacquire_frames += 1
                else:
                    corner_reacquire_frames = 0

                if corner_reacquire_frames >= CORNER_REACQUIRE_FRAMES:
                    corner_active = False
                    corner_direction = 0
                    corner_reacquire_frames = 0
                    reset_pid_history()
            else:
                corner_active = False
                corner_direction = 0
                corner_reacquire_frames = 0
                reset_pid_history()
                stop_all()
            continue

        if detected:
            seen_line_once = True
            last_seen_time = now
            flag_judge()
            block_cross_approach_corner = True

            if not cross_cooldown_active and is_cross_detected():
                state = "CROSS_HOLD"
                cross_hold_active = True
                cross_hold_start = now
                circle_entry_now = is_circle_entry_cross(cross_count)
                if cross_approach_active:
                    cross_hold_error = cross_approach_error
                else:
                    cross_hold_error = cross_entry_error(position)
                cross_hold_bias = first_cross_bias(cross_count)
                cross_hold_bypass = circle_entry_now
                cross_count += 1
                cross_approach_active = False
                cross_approach_error = 0.0
                cross_approach_bypass = False
                corner_candidate_direction = 0
                corner_candidate_frames = 0
                reset_pid_history()
                left_command, right_command = cross_hold_commands(cross_hold_error, cross_hold_bias)
                set_wheel_commands(
                    left_command,
                    right_command,
                    bypass_limit=cross_hold_bypass
                )
                continue

            cross_approach_now = is_cross_approach_detected()
            if (
                not cross_cooldown_active and
                (cross_approach_active or cross_approach_now)
            ):
                cross_approach_tail_now = (
                    cross_approach_active and
                    is_cross_approach_tail_detected()
                )

                if cross_approach_now and not cross_approach_active:
                    cross_approach_active = True
                    cross_approach_start = now
                    cross_approach_error = cross_entry_error(position)
                    cross_approach_bypass = is_circle_entry_cross(cross_count)

                if (
                    (cross_approach_now or cross_approach_tail_now) and
                    ticks_diff(now, cross_approach_start) < CROSS_APPROACH_GUARD_MS
                ):
                    state = "CROSS_APPROACH"
                    corner_candidate_direction = 0
                    corner_candidate_frames = 0
                    reset_pid_history()
                    left_command, right_command = cross_approach_commands(cross_approach_error, first_cross_bias(cross_count))
                    set_wheel_commands(
                        left_command,
                        right_command,
                        bypass_limit=cross_approach_bypass
                    )
                    continue

                cross_approach_active = False
                cross_approach_error = 0.0
                cross_approach_bypass = False

            corner_direction_candidate = get_corner_direction(
                block_cross_approach_corner
            )
            if not cross_cooldown_active and corner_direction_candidate != 0:
                if corner_direction_candidate == corner_candidate_direction:
                    corner_candidate_frames += 1
                else:
                    corner_candidate_direction = corner_direction_candidate
                    corner_candidate_frames = 1

                if corner_candidate_frames >= CORNER_CONFIRM_FRAMES:
                    corner_direction = corner_candidate_direction
                    corner_candidate_direction = 0
                    corner_candidate_frames = 0
                    reset_pid_history()

                    if CORNER_BRAKE_ENABLE:
                        state = "CORNER_BRAKE"
                        corner_brake_active = True
                        corner_brake_direction = corner_direction
                        corner_brake_start = now
                        left_command = 0
                        right_command = 0
                        set_wheel_commands(
                            left_command,
                            right_command,
                            bypass_limit=True
                        )
                        continue

                    state = "CORNER_FULL"
                    corner_active = True
                    corner_start = now
                    corner_reacquire_frames = 0
                    left_command, right_command = corner_full_commands(corner_direction)
                    set_wheel_commands(
                        left_command,
                        right_command,
                        bypass_limit=True
                    )
                    continue
            else:
                corner_candidate_direction = 0
                corner_candidate_frames = 0

            left_command, right_command, error = follow_line_step(base_command)
            set_wheel_commands(left_command, right_command)

        else:
            cross_approach_active = False
            cross_approach_error = 0.0
            cross_approach_bypass = False
            corner_candidate_direction = 0
            corner_candidate_frames = 0
            reset_pid_history()

            if not seen_line_once:
                if ticks_diff(now, start_search_start) < START_NO_LINE_FORWARD_MS:
                    state = "START_SEARCH"
                    left_command = START_NO_LINE_COMMAND
                    right_command = START_NO_LINE_COMMAND
                    set_wheel_commands(left_command, right_command)
                else:
                    state = "NO_LINE"
                    stop_all()
            else:
                state = "LOST"
                left_command, right_command = calculate_lost_commands(base_command)
                set_wheel_commands(left_command, right_command)

        if DEBUG_PRINT and ticks_diff(now, last_debug_time) >= DEBUG_PERIOD_MS:
            last_debug_time = now
            print(
                "raw=%s p=%s s=%s pos=%s err=%s flag=%d L=%d R=%d state=%s last_seen=%d" %
                (
                    raw_values,
                    p_values,
                    s_values,
                    position,
                    error,
                    last_flag,
                    left_command,
                    right_command,
                    state,
                    ticks_diff(now, last_seen_time)
                )
            )


gc.collect()
stop_all()

try:
    track_loop()
except KeyboardInterrupt:
    stop_all()
    print()
    print("stopped, motors off")
except Exception as error:
    stop_all()
    print()
    print("error, motors off:")
    print(repr(error))
    raise
finally:
    stop_all()




