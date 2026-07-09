# ============================================================
# ESP32 五路红外循迹标定程序
# 适配当前闭环循迹代码：生成 line_track_config.json
#
# 用法：
#   1. 把本文件上传到 ESP32，可命名为 calibration.py 或 main.py。
#   2. 先把五路传感器全部放在白底，按回车采集白底。
#   3. 按提示在 10 秒内左右移动小车/传感器，让每一路都扫过黑线。
#   4. 程序会保存 line_track_config.json。
#   5. 再运行你的循迹主程序；主程序会自动读取该 JSON 标定。
# ============================================================

from machine import Pin, ADC
from time import sleep_ms, ticks_ms, ticks_diff

try:
    import ujson as json
except ImportError:
    import json


# ============================================================
# 必须与当前循迹主程序保持一致
# ============================================================

SENSOR_PINS = (27, 33, 32, 35, 34)
SENSOR_POSITION_MM = (-35.0, -15.0, 0.0, 15.0, 35.0)
CONFIG_FILE = "line_track_config.json"

# 当前闭环代码的 SENSOR_THRESHOLDS 约等于：
# threshold = white + 0.45 * (black - white)
# 不建议设到 0.50，因为部分传感器黑白跨度不一致，0.45 更容易提前识别黑线。
THRESHOLD_RATIO = 0.45

# 采样设置
ADC_AVERAGE_SAMPLES = 8
WHITE_SAMPLE_MS = 2500
BLACK_SWEEP_MS = 10000
PRINT_PERIOD_MS = 300

# 如果某一路黑白差小于这个值，说明该路扫线不充分或高度/光照异常
MIN_VALID_SPAN = 80


sensor_adcs = []

for pin_number in SENSOR_PINS:
    adc = ADC(Pin(pin_number))
    adc.atten(ADC.ATTN_11DB)
    try:
        adc.width(ADC.WIDTH_12BIT)
    except Exception:
        pass
    sensor_adcs.append(adc)


def read_one_channel(adc):
    total = 0
    for _ in range(ADC_AVERAGE_SAMPLES):
        total += adc.read()
        sleep_ms(1)
    return total / ADC_AVERAGE_SAMPLES


def read_all_channels():
    return [read_one_channel(adc) for adc in sensor_adcs]


def format_values(values, digits=1):
    return "[" + ", ".join(("{:.%df}" % digits).format(v) for v in values) + "]"


def wait_enter(message):
    print(message)
    try:
        input()
    except Exception:
        # 某些串口环境 input 不可用时，给 3 秒准备时间
        print("input不可用，3秒后自动开始。")
        sleep_ms(3000)


def collect_white_values():
    print("\n============================================================")
    print("步骤1：采集白底")
    print("============================================================")
    wait_enter("请把五路传感器全部放在白色/浅色底面上，保持实际高度和供电状态，然后按回车。")

    sums = [0.0] * 5
    count = 0
    start = ticks_ms()
    last_print = start

    while ticks_diff(ticks_ms(), start) < WHITE_SAMPLE_MS:
        values = read_all_channels()
        for i in range(5):
            sums[i] += values[i]
        count += 1

        now = ticks_ms()
        if ticks_diff(now, last_print) >= PRINT_PERIOD_MS:
            print("白底实时:", format_values(values, 1))
            last_print = now

    white_values = [s / count for s in sums]
    print("\n白底采集完成:", format_values(white_values, 1))
    return white_values


def collect_black_values(white_values):
    print("\n============================================================")
    print("步骤2：采集黑线")
    print("============================================================")
    print("按回车后有 {} 秒采集时间。".format(BLACK_SWEEP_MS // 1000))
    print("请左右移动传感器板/小车，让黑线依次经过五路传感器。")
    print("重点：每一路都要扫到黑线，尤其最左、最右。")
    wait_enter("准备好后按回车开始采集黑线。")

    max_values = [0.0] * 5
    min_values = [4095.0] * 5
    start = ticks_ms()
    last_print = start

    while ticks_diff(ticks_ms(), start) < BLACK_SWEEP_MS:
        values = read_all_channels()

        for i in range(5):
            if values[i] > max_values[i]:
                max_values[i] = values[i]
            if values[i] < min_values[i]:
                min_values[i] = values[i]

        now = ticks_ms()
        if ticks_diff(now, last_print) >= PRINT_PERIOD_MS:
            remain = (BLACK_SWEEP_MS - ticks_diff(now, start)) // 1000
            print("黑线实时:", format_values(values, 1), "剩余", remain, "s")
            last_print = now

    # 自动判断黑线方向：多数通道黑线比白底高，则 LINE_IS_HIGH=True
    high_votes = 0
    low_votes = 0
    for i in range(5):
        up_span = abs(max_values[i] - white_values[i])
        down_span = abs(white_values[i] - min_values[i])
        if up_span >= down_span:
            high_votes += 1
        else:
            low_votes += 1

    line_is_high = high_votes >= low_votes

    if line_is_high:
        black_values = max_values
    else:
        black_values = min_values

    print("\n黑线采集完成。")
    print("黑线最大值:", format_values(max_values, 1))
    print("黑线最小值:", format_values(min_values, 1))
    print("判定 LINE_IS_HIGH =", line_is_high)
    print("采用黑线值:", format_values(black_values, 1))

    return black_values, line_is_high


def make_thresholds(white_values, black_values):
    thresholds = []
    spans = []

    for i in range(5):
        span = black_values[i] - white_values[i]
        threshold = white_values[i] + THRESHOLD_RATIO * span
        thresholds.append(threshold)
        spans.append(abs(span))

    return thresholds, spans


def save_config(white_values, black_values, thresholds, line_is_high):
    channels = []
    for i in range(5):
        channels.append({
            "index": i,
            "pin": SENSOR_PINS[i],
            "position_mm": SENSOR_POSITION_MM[i],
            "white": round(float(white_values[i]), 3),
            "black": round(float(black_values[i]), 3),
            "threshold": round(float(thresholds[i]), 3)
        })

    config = {
        "pins": list(SENSOR_PINS),
        "positions_mm": list(SENSOR_POSITION_MM),
        "line_is_high": bool(line_is_high),
        "threshold_ratio": THRESHOLD_RATIO,
        "channels": channels
    }

    with open(CONFIG_FILE, "w") as file:
        json.dump(config, file)

    print("\n已保存:", CONFIG_FILE)


def print_code_constants(white_values, black_values, thresholds, line_is_high):
    print("\n============================================================")
    print("可直接粘贴进主程序的常量")
    print("============================================================")
    print("WHITE_VALUES =", tuple(round(float(v), 3) for v in white_values))
    print("BLACK_VALUES =", tuple(round(float(v), 3) for v in black_values))
    print("SENSOR_THRESHOLDS =", tuple(round(float(v), 3) for v in thresholds))
    print("LINE_IS_HIGH =", bool(line_is_high))


def check_result(white_values, black_values, thresholds, spans):
    print("\n============================================================")
    print("标定检查")
    print("============================================================")
    print("白底:", format_values(white_values, 1))
    print("黑线:", format_values(black_values, 1))
    print("阈值:", format_values(thresholds, 1))
    print("跨度:", format_values(spans, 1))

    warning = False
    for i, span in enumerate(spans):
        if span < MIN_VALID_SPAN:
            warning = True
            print("警告：第{}路黑白跨度过小，span={:.1f}，可能没有充分扫到黑线。".format(i + 1, span))

    if not warning:
        print("黑白跨度检查通过。")


def main():
    print("============================================================")
    print("ESP32 五路红外黑白标定")
    print("适配当前闭环循迹代码 line_track_config.json 格式")
    print("============================================================")
    print("传感器顺序：最左、左中、中间、右中、最右")
    print("传感器GPIO：", SENSOR_PINS)
    print("传感器位置：", SENSOR_POSITION_MM)
    print("阈值比例：white + {:.2f} * (black - white)".format(THRESHOLD_RATIO))

    white_values = collect_white_values()
    black_values, line_is_high = collect_black_values(white_values)
    thresholds, spans = make_thresholds(white_values, black_values)

    check_result(white_values, black_values, thresholds, spans)
    save_config(white_values, black_values, thresholds, line_is_high)
    print_code_constants(white_values, black_values, thresholds, line_is_high)

    print("\n完成。现在可以运行循迹主程序。")
    print("如果主程序同目录存在 line_track_config.json，会自动加载新标定。")


main()
