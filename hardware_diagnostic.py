"""
Standalone hardware diagnostics for the ESP32 line-following car.

Default mode is sensor-only and does not move the car. Change TEST_MODE to
"motors" only after lifting the wheels off the ground.
"""

from machine import Pin, PWM, ADC
from time import sleep_ms, ticks_ms, ticks_diff

try:
    from machine import Encoder as HardwareEncoder
except ImportError:
    HardwareEncoder = None


# ============================================================
# Test mode
# ============================================================

TEST_MODE = "sensors"          # "sensors", "stream", "encoders_hand", or "motors".
SENSOR_TEST_MS = 6000          # Static sensor sampling time; keep car still during this window.
SENSOR_SAMPLE_MS = 8           # Sensor sampling period; close to the line-following control loop.
STREAM_PERIOD_MS = 100         # Live stream print interval for watching raw/p/s/position changes.
STREAM_TEST_MS = 15000         # Live stream duration; useful while moving the car by hand.

MOTOR_SETTLE_MS = 250          # Wait after applying PWM before measuring encoder speed.
MOTOR_MEASURE_MS = 900         # Encoder measurement window for each motor stage.
MOTOR_BRAKE_MS = 450           # Stop time between stages to avoid carrying momentum into next test.
MOTOR_TEST_COMMANDS = (450, 600, 750)  # PWM points used to compare left/right wheel response.

SENSOR_NORM_SPAN_WARN = 0.10   # Stationary normalized jitter above this is suspicious.
SENSOR_FLIP_WARN = 1           # Any binary flip on a stable white/black area should be investigated.
RPM_MISMATCH_WARN = 0.15       # More than 15% left/right RPM mismatch at same PWM is suspicious.
MIN_ACTIVE_RPM = 20.0          # Below this during motor test means slipping, stalled motor, or bad encoder.


# ============================================================
# Hardware constants copied from main.py
# ============================================================

LEFT_IN1_PIN = 15
LEFT_IN2_PIN = 13
RIGHT_IN1_PIN = 25
RIGHT_IN2_PIN = 14

LEFT_INVERT = False
RIGHT_INVERT = True

PWM_FREQ = 20000
PWM_MAX = 1000

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

SENSOR_PINS = (27, 33, 32, 35, 34)
POSITION_WEIGHTS = (0, 1000, 2000, 3000, 4000)
LINE_CENTER = 2000

WHITE_VALUES = (104.914, 59.683, 57.817, 66.747, 139.306)
BLACK_VALUES = (1967.5, 815.5, 239.5, 1295.5, 2735.5)
SENSOR_THRESHOLDS = (943.078, 399.801, 139.574, 619.686, 1307.594)
LINE_IS_HIGH = True
SENSOR_AVERAGE_SAMPLES = 4
BINARY_ACTIVE_NORM = 0.42
POSITION_SUM_MIN = 80


def clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


class Motor:
    def __init__(self, in1_pin, in2_pin, invert):
        self.pin1 = Pin(in1_pin, Pin.OUT, value=0)
        self.pin2 = Pin(in2_pin, Pin.OUT, value=0)
        self.pwm1 = PWM(self.pin1, freq=PWM_FREQ)
        self.pwm2 = PWM(self.pin2, freq=PWM_FREQ)
        self.invert = invert
        self.has_duty_u16 = hasattr(self.pwm1, "duty_u16")
        self.stop()

    def _write_pwm(self, pwm, command):
        command = int(clamp(command, 0, PWM_MAX))
        if self.has_duty_u16:
            pwm.duty_u16(command * 65535 // PWM_MAX)
        else:
            pwm.duty(command * 1023 // PWM_MAX)

    def set(self, command):
        command = int(clamp(command, -PWM_MAX, PWM_MAX))
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
                    filter_ns=ENCODER_FILTER_NS,
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
                    filter_ns=ENCODER_FILTER_NS,
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


left_motor = Motor(LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_INVERT)
right_motor = Motor(RIGHT_IN1_PIN, RIGHT_IN2_PIN, RIGHT_INVERT)

left_encoder = EncoderCounter(0, LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN)
right_encoder = EncoderCounter(1, RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN)

sensor_adcs = []
for pin_number in SENSOR_PINS:
    adc = ADC(Pin(pin_number))
    adc.atten(ADC.ATTN_11DB)
    try:
        adc.width(ADC.WIDTH_12BIT)
    except Exception:
        pass
    sensor_adcs.append(adc)


def stop_all():
    left_motor.stop()
    right_motor.stop()


def read_left_count():
    return left_encoder.value() * LEFT_ENCODER_SIGN


def read_right_count():
    return right_encoder.value() * RIGHT_ENCODER_SIGN


def reset_encoders():
    try:
        left_encoder.value(0)
        right_encoder.value(0)
    except Exception:
        pass


def calculate_rpm(delta_count, dt_ms, counts_per_rev):
    if dt_ms <= 0:
        return 0.0
    return (float(delta_count) * 60000.0) / (counts_per_rev * float(dt_ms))


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


def read_sensor_frame():
    raw_values = []
    norm_values = []
    p_values = []
    s_values = []

    for index, adc in enumerate(sensor_adcs):
        raw = read_adc_average(adc)
        norm = normalize_sensor(raw, index)
        p_value = int(norm * 1000)

        if LINE_IS_HIGH:
            active_raw = raw >= SENSOR_THRESHOLDS[index]
        else:
            active_raw = raw <= SENSOR_THRESHOLDS[index]
        active_norm = norm >= BINARY_ACTIVE_NORM
        active = 1 if (active_raw or active_norm) else 0

        raw_values.append(int(raw))
        norm_values.append(norm)
        p_values.append(p_value)
        s_values.append(active)

    return raw_values, norm_values, p_values, s_values


def compute_position(p_values, s_values):
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


def run_sensor_static_test():
    print("")
    print("=== SENSOR STATIC TEST ===")
    print("Keep the car completely still on one area: white, black line, or cross.")
    print("Healthy result on a stable area: flips=0 and norm_span <= %.2f." % SENSOR_NORM_SPAN_WARN)
    print("")

    raw_min = [999999, 999999, 999999, 999999, 999999]
    raw_max = [0, 0, 0, 0, 0]
    raw_sum = [0, 0, 0, 0, 0]
    norm_min = [999.0, 999.0, 999.0, 999.0, 999.0]
    norm_max = [0.0, 0.0, 0.0, 0.0, 0.0]
    flips = [0, 0, 0, 0, 0]
    last_s = None
    samples = 0

    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < SENSOR_TEST_MS:
        raw, norm, p_values, s_values = read_sensor_frame()
        for index in range(5):
            raw_min[index] = min(raw_min[index], raw[index])
            raw_max[index] = max(raw_max[index], raw[index])
            raw_sum[index] += raw[index]
            norm_min[index] = min(norm_min[index], norm[index])
            norm_max[index] = max(norm_max[index], norm[index])
            if last_s is not None and s_values[index] != last_s[index]:
                flips[index] += 1
        last_s = list(s_values)
        samples += 1
        sleep_ms(SENSOR_SAMPLE_MS)

    raw, norm, p_values, s_values = read_sensor_frame()
    print("idx pin   avg_raw min_raw max_raw raw_span  norm_span flips active threshold")

    any_warning = False
    for index, pin_number in enumerate(SENSOR_PINS):
        avg_raw = raw_sum[index] / max(samples, 1)
        raw_span = raw_max[index] - raw_min[index]
        norm_span = norm_max[index] - norm_min[index]
        warning = (
            norm_span > SENSOR_NORM_SPAN_WARN or
            flips[index] >= SENSOR_FLIP_WARN
        )
        if warning:
            any_warning = True
        marker = "  WARN" if warning else ""
        print(
            "%d   %2d  %7.1f %7d %7d %8d   %7.3f %5d   %d    %7.1f%s" %
            (
                index + 1,
                pin_number,
                avg_raw,
                raw_min[index],
                raw_max[index],
                raw_span,
                norm_span,
                flips[index],
                s_values[index],
                SENSOR_THRESHOLDS[index],
                marker,
            )
        )

    position = compute_position(p_values, s_values)
    if position is None:
        print("position=None p=%s s=%s raw=%s" % (p_values, s_values, raw))
    else:
        print(
            "position=%.1f error=%.1f p=%s s=%s raw=%s" %
            (position, position - LINE_CENTER, p_values, s_values, raw)
        )

    if any_warning:
        print("")
        print("Result: suspicious sensor instability.")
        print("Check sensor board height/angle, loose sensor wires, 3.3V/GND, and dirty or reflective tape.")
    else:
        print("")
        print("Result: static sensor readings look stable for this surface.")


def run_sensor_stream():
    print("")
    print("=== SENSOR LIVE STREAM ===")
    print("Move the car by hand over white, line edge, cross, and curve. Watch for sudden jumps.")
    print("")

    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < STREAM_TEST_MS:
        raw, norm, p_values, s_values = read_sensor_frame()
        position = compute_position(p_values, s_values)
        if position is None:
            error_text = "None"
        else:
            error_text = "%.0f" % (position - LINE_CENTER)
        print("raw=%s p=%s s=%s err=%s" % (raw, p_values, s_values, error_text))
        sleep_ms(STREAM_PERIOD_MS)


def run_hand_encoder_test():
    print("")
    print("=== HAND ENCODER TEST ===")
    print("Slowly turn each wheel forward by hand for 8 seconds.")
    print("Forward wheel rotation should make the matching signed count increase.")
    print("")

    reset_encoders()
    start = ticks_ms()
    last_print = start
    while ticks_diff(ticks_ms(), start) < 8000:
        now = ticks_ms()
        if ticks_diff(now, last_print) >= 250:
            print("left=%d right=%d" % (read_left_count(), read_right_count()))
            last_print = now
        sleep_ms(20)
    print("final left=%d right=%d" % (read_left_count(), read_right_count()))


def measure_stage(left_command, right_command):
    stop_all()
    sleep_ms(MOTOR_BRAKE_MS)
    left_motor.set(left_command)
    right_motor.set(right_command)
    sleep_ms(MOTOR_SETTLE_MS)
    reset_encoders()
    start = ticks_ms()
    start_left = read_left_count()
    start_right = read_right_count()
    sleep_ms(MOTOR_MEASURE_MS)
    end = ticks_ms()
    end_left = read_left_count()
    end_right = read_right_count()
    stop_all()
    dt_ms = ticks_diff(end, start)
    left_rpm = calculate_rpm(end_left - start_left, dt_ms, LEFT_COUNTS_PER_REV)
    right_rpm = calculate_rpm(end_right - start_right, dt_ms, RIGHT_COUNTS_PER_REV)
    sleep_ms(MOTOR_BRAKE_MS)
    return left_rpm, right_rpm


def print_rpm_warning(label, left_rpm, right_rpm, expected_left, expected_right):
    warnings = []
    if expected_left != 0:
        if left_rpm * expected_left <= 0 or abs(left_rpm) < MIN_ACTIVE_RPM:
            warnings.append("left encoder/motor weak or wrong sign")
    if expected_right != 0:
        if right_rpm * expected_right <= 0 or abs(right_rpm) < MIN_ACTIVE_RPM:
            warnings.append("right encoder/motor weak or wrong sign")

    if expected_left != 0 and expected_right != 0:
        biggest = max(abs(left_rpm), abs(right_rpm), 1.0)
        mismatch = abs(abs(left_rpm) - abs(right_rpm)) / biggest
        if mismatch > RPM_MISMATCH_WARN:
            warnings.append("left/right RPM mismatch %.0f%%" % (mismatch * 100.0))

    if warnings:
        print("%s -> WARN: %s" % (label, "; ".join(warnings)))


def run_motor_test():
    print("")
    print("=== MOTOR + ENCODER TEST ===")
    print("Lift the wheels off the ground now. Motors will start after 3 seconds.")
    print("If the car is on the track, stop/reset the board immediately.")
    sleep_ms(3000)

    for command in MOTOR_TEST_COMMANDS:
        label = "left forward pwm=%d" % command
        left_rpm, right_rpm = measure_stage(command, 0)
        print("%s  left_rpm=%.1f right_rpm=%.1f" % (label, left_rpm, right_rpm))
        print_rpm_warning(label, left_rpm, right_rpm, 1, 0)

        label = "right forward pwm=%d" % command
        left_rpm, right_rpm = measure_stage(0, command)
        print("%s left_rpm=%.1f right_rpm=%.1f" % (label, left_rpm, right_rpm))
        print_rpm_warning(label, left_rpm, right_rpm, 0, 1)

        label = "both forward pwm=%d" % command
        left_rpm, right_rpm = measure_stage(command, command)
        print("%s  left_rpm=%.1f right_rpm=%.1f" % (label, left_rpm, right_rpm))
        print_rpm_warning(label, left_rpm, right_rpm, 1, 1)

    reverse_command = -600
    label = "left reverse pwm=600"
    left_rpm, right_rpm = measure_stage(reverse_command, 0)
    print("%s  left_rpm=%.1f right_rpm=%.1f" % (label, left_rpm, right_rpm))
    print_rpm_warning(label, left_rpm, right_rpm, -1, 0)

    label = "right reverse pwm=600"
    left_rpm, right_rpm = measure_stage(0, reverse_command)
    print("%s left_rpm=%.1f right_rpm=%.1f" % (label, left_rpm, right_rpm))
    print_rpm_warning(label, left_rpm, right_rpm, 0, -1)

    stop_all()
    print("")
    print("Motor test done. Large mismatch means battery, motor driver, motor, wheel friction, or encoder issue.")


def main():
    try:
        stop_all()
        if TEST_MODE == "sensors":
            run_sensor_static_test()
        elif TEST_MODE == "stream":
            run_sensor_stream()
        elif TEST_MODE == "encoders_hand":
            run_hand_encoder_test()
        elif TEST_MODE == "motors":
            run_motor_test()
        else:
            print("Unknown TEST_MODE: %s" % TEST_MODE)
    finally:
        stop_all()


main()
