import ast
import pathlib
import unittest


MAIN_PATH = pathlib.Path(__file__).with_name("main.py")
CONFIG_PATH = pathlib.Path(__file__).with_name("line_track_config.json")


def parse_main():
    return ast.parse(MAIN_PATH.read_text(encoding="utf-8"))


def load_constants(tree):
    def literal_value(node):
        if isinstance(node, ast.Constant):
            return node.value
        if (
            isinstance(node, ast.UnaryOp) and
            isinstance(node.op, ast.USub) and
            isinstance(node.operand, ast.Constant)
        ):
            return -node.operand.value
        raise ValueError("not a simple literal")

    constants = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if len(node.targets) != 1 or not isinstance(node.targets[0], ast.Name):
            continue
        value = node.value
        if isinstance(value, ast.Constant):
            constants[node.targets[0].id] = value.value
        elif (
            isinstance(value, ast.UnaryOp) and
            isinstance(value.op, ast.USub) and
            isinstance(value.operand, ast.Constant)
        ):
            constants[node.targets[0].id] = -value.operand.value
        elif isinstance(value, ast.Tuple):
            values = []
            for item in value.elts:
                try:
                    values.append(literal_value(item))
                except ValueError:
                    pass
            constants[node.targets[0].id] = tuple(values)
    return constants


def load_functions_namespace(*function_names):
    source = MAIN_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    namespace = load_constants(tree)
    namespace.update({
        "s_values": [0, 0, 0, 0, 0],
        "p_values": [0, 0, 0, 0, 0],
        "sum": sum,
        "ticks_diff": lambda now, start: now - start,
    })

    for function_name in function_names:
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name == function_name:
                function_source = ast.get_source_segment(source, node)
                exec(function_source, namespace)
                break
        else:
            raise AssertionError("%s is missing from main.py" % function_name)

    return namespace


def assignment_line(source, constant_name):
    prefix = "%s =" % constant_name
    for line in source.splitlines():
        if line.startswith(prefix):
            return line
    raise AssertionError("%s assignment is missing from main.py" % constant_name)


class ControlLogicTests(unittest.TestCase):

    def test_uses_arduino_five_sensor_pid_structure(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        constants = load_constants(parse_main())

        self.assertIn("light_judge", source)
        self.assertIn("flag_judge", source)
        self.assertIn("follow_line_step", source)
        self.assertEqual(
            constants["POSITION_WEIGHTS"],
            (0, 1000, 2000, 3000, 4000)
        )
        self.assertEqual(constants["LINE_CENTER"], 2000)
        self.assertIn("last_flag", source)

    def test_embeds_user_calibration_and_config_loader(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        constants = load_constants(parse_main())

        self.assertTrue(CONFIG_PATH.exists())
        self.assertEqual(constants["SENSOR_PINS"], (27, 33, 32, 35, 34))
        self.assertEqual(
            constants["SENSOR_POSITION_MM"],
            (-35.0, -15.0, 0.0, 15.0, 35.0)
        )
        self.assertEqual(
            constants["WHITE_VALUES"],
            (139.1, 80.2, 76.3, 89.2, 581.2)
        )
        self.assertEqual(
            constants["BLACK_VALUES"],
            (1583.5, 1167.5, 1007.5, 1359.5, 1775.5)
        )
        self.assertEqual(
            constants["SENSOR_THRESHOLDS"],
            (789.1, 569.5, 495.3, 660.8, 1118.6)
        )
        self.assertIn("load_tracking_config", source)
        self.assertIn("line_track_config.json", source)

    def test_binary_detection_uses_normalized_scores_only(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        constants = load_constants(parse_main())
        namespace = load_functions_namespace("clamp", "normalize_sensor")

        self.assertLessEqual(constants["BINARY_ACTIVE_NORM"], 0.36)
        self.assertIn("active_norm = norm >= BINARY_ACTIVE_NORM", source)
        self.assertNotIn("active_raw", source)
        self.assertEqual(namespace["normalize_sensor"](581.2, 4), 0.0)
        self.assertGreater(
            namespace["normalize_sensor"](1118.6, 4),
            constants["BINARY_ACTIVE_NORM"]
        )

    def test_speed_closed_loop_is_present_and_startup_safe(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        tree = parse_main()
        constants = load_constants(tree)

        direct_encoder_import = False
        for node in tree.body:
            if isinstance(node, ast.ImportFrom) and node.module == "machine":
                direct_encoder_import = any(
                    alias.name == "Encoder" for alias in node.names
                )

        self.assertFalse(direct_encoder_import)
        self.assertTrue(constants["USE_SPEED_CLOSED_LOOP"])
        self.assertIn("HardwareEncoder = None", source)
        self.assertIn("class EncoderCounter", source)
        self.assertIn("command_to_target_rpm", source)
        self.assertLessEqual(constants["SPEED_PERIOD_MS"], 10)
        self.assertGreaterEqual(constants["CLOSED_LOOP_REFERENCE_RPM"], 210.0)

    def test_corner_and_cross_handling_are_explicit(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        constants = load_constants(parse_main())

        self.assertIn("is_cross_detected", source)
        self.assertIn("get_corner_direction", source)
        self.assertIn('state = "CROSS_HOLD"', source)
        self.assertIn('state = "CORNER_FULL"', source)
        self.assertIn('state = "CORNER_SEARCH"', source)
        self.assertGreater(constants["CORNER_FULL_POWER_MS"], 0)
        self.assertGreater(constants["CROSS_HOLD_MS"], 0)

    def test_cross_wide_shape_suppresses_corner_classification(self):
        namespace = load_functions_namespace(
            "is_cross_detected",
            "is_cross_approach_detected",
            "is_long_curve_detected",
            "get_corner_direction"
        )

        namespace["s_values"][:] = [1, 0, 1, 1, 0]
        namespace["p_values"][:] = [620, 120, 760, 640, 90]

        self.assertTrue(namespace["is_cross_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 0)

    def test_skewed_cross_with_opposite_hint_is_detected(self):
        namespace = load_functions_namespace(
            "is_cross_detected",
            "is_cross_approach_detected",
            "is_long_curve_detected",
            "get_corner_direction"
        )

        namespace["s_values"][:] = [1, 1, 1, 0, 0]
        namespace["p_values"][:] = [720, 620, 650, 230, 70]

        self.assertTrue(namespace["is_cross_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 0)

        namespace["s_values"][:] = [0, 0, 1, 1, 1]
        namespace["p_values"][:] = [70, 230, 650, 620, 720]

        self.assertTrue(namespace["is_cross_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 0)

    def test_skewed_right_angle_without_opposite_hint_is_not_cross(self):
        namespace = load_functions_namespace(
            "is_cross_detected",
            "is_cross_approach_detected",
            "is_long_curve_detected",
            "get_corner_direction"
        )

        namespace["s_values"][:] = [1, 1, 1, 0, 0]
        namespace["p_values"][:] = [780, 520, 420, 90, 40]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertFalse(namespace["is_cross_approach_detected"]())
        self.assertFalse(namespace["is_long_curve_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), -1)

        namespace["s_values"][:] = [0, 0, 1, 1, 1]
        namespace["p_values"][:] = [40, 90, 420, 520, 780]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertFalse(namespace["is_cross_approach_detected"]())
        self.assertFalse(namespace["is_long_curve_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 1)

    def test_skewed_cross_approach_suppresses_corner_before_full_cross(self):
        namespace = load_functions_namespace(
            "is_cross_detected",
            "is_cross_approach_detected",
            "is_cross_approach_tail_detected",
            "is_long_curve_detected",
            "get_corner_direction"
        )

        namespace["s_values"][:] = [1, 1, 1, 0, 0]
        namespace["p_values"][:] = [720, 620, 650, 150, 70]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertTrue(namespace["is_cross_approach_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 0)
        self.assertEqual(namespace["get_corner_direction"](False), -1)

        namespace["s_values"][:] = [0, 0, 1, 1, 1]
        namespace["p_values"][:] = [70, 150, 650, 630, 720]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertTrue(namespace["is_cross_approach_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 0)
        self.assertEqual(namespace["get_corner_direction"](False), 1)

    def test_weak_center_skewed_cross_approach_does_not_become_corner(self):
        namespace = load_functions_namespace(
            "is_cross_detected",
            "is_cross_approach_detected",
            "is_long_curve_detected",
            "get_corner_direction"
        )

        namespace["s_values"][:] = [1, 1, 0, 0, 0]
        namespace["p_values"][:] = [720, 640, 260, 170, 70]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertTrue(namespace["is_cross_approach_detected"]())
        self.assertFalse(namespace["is_long_curve_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 0)
        self.assertEqual(namespace["get_corner_direction"](False), -1)

        namespace["s_values"][:] = [0, 0, 0, 1, 1]
        namespace["p_values"][:] = [70, 170, 260, 640, 720]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertTrue(namespace["is_cross_approach_detected"]())
        self.assertFalse(namespace["is_long_curve_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 0)
        self.assertEqual(namespace["get_corner_direction"](False), 1)

    def test_cross_approach_tail_keeps_short_guard_on_severe_skew(self):
        namespace = load_functions_namespace(
            "is_cross_detected",
            "is_cross_approach_tail_detected",
            "is_cross_approach_detected",
            "is_long_curve_detected",
            "get_corner_direction"
        )

        namespace["s_values"][:] = [1, 1, 0, 0, 0]
        namespace["p_values"][:] = [720, 620, 90, 40, 30]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertTrue(namespace["is_cross_approach_tail_detected"]())
        self.assertEqual(namespace["get_corner_direction"](False), -1)

        namespace["s_values"][:] = [0, 0, 0, 1, 1]
        namespace["p_values"][:] = [35, 45, 90, 630, 720]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertTrue(namespace["is_cross_approach_tail_detected"]())
        self.assertEqual(namespace["get_corner_direction"](False), 1)

    def test_corner_analog_edge_triggers_before_binary_pair(self):
        namespace = load_functions_namespace(
            "is_cross_detected",
            "is_cross_approach_detected",
            "is_long_curve_detected",
            "get_corner_direction"
        )

        namespace["s_values"][:] = [1, 0, 1, 0, 0]
        namespace["p_values"][:] = [620, 210, 240, 45, 35]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertFalse(namespace["is_cross_approach_detected"]())
        self.assertFalse(namespace["is_long_curve_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), -1)

        namespace["s_values"][:] = [0, 0, 1, 0, 1]
        namespace["p_values"][:] = [35, 45, 240, 210, 620]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertFalse(namespace["is_cross_approach_detected"]())
        self.assertFalse(namespace["is_long_curve_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 1)

    def test_loaded_car_weaker_corner_edge_still_triggers_before_running_wide(self):
        namespace = load_functions_namespace(
            "is_cross_detected",
            "is_cross_approach_detected",
            "is_long_curve_detected",
            "get_corner_direction"
        )

        namespace["s_values"][:] = [1, 0, 0, 0, 0]
        namespace["p_values"][:] = [520, 210, 250, 60, 35]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertFalse(namespace["is_cross_approach_detected"]())
        self.assertFalse(namespace["is_long_curve_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), -1)

        namespace["s_values"][:] = [0, 0, 0, 0, 1]
        namespace["p_values"][:] = [35, 60, 250, 210, 520]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertFalse(namespace["is_cross_approach_detected"]())
        self.assertFalse(namespace["is_long_curve_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 1)

    def test_long_curve_shape_slows_without_becoming_corner(self):
        namespace = load_functions_namespace(
            "is_cross_detected",
            "is_cross_approach_detected",
            "is_long_curve_detected",
            "get_corner_direction"
        )

        namespace["s_values"][:] = [0, 1, 1, 0, 0]
        namespace["p_values"][:] = [180, 560, 610, 120, 40]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertFalse(namespace["is_cross_approach_detected"]())
        self.assertTrue(namespace["is_long_curve_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 0)

        namespace["s_values"][:] = [0, 0, 1, 1, 0]
        namespace["p_values"][:] = [40, 120, 610, 560, 180]

        self.assertFalse(namespace["is_cross_detected"]())
        self.assertFalse(namespace["is_cross_approach_detected"]())
        self.assertTrue(namespace["is_long_curve_detected"]())
        self.assertEqual(namespace["get_corner_direction"](), 0)

    def test_corner_entry_requires_confirmation_frames(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        constants = load_constants(parse_main())

        self.assertGreaterEqual(constants["CORNER_CONFIRM_FRAMES"], 2)
        self.assertLessEqual(constants["CORNER_CONFIRM_FRAMES"], 3)
        self.assertLessEqual(constants["CONTROL_PERIOD_MS"], 5)
        self.assertEqual(constants["CORNER_CONFIRM_FRAMES"], 2)
        self.assertGreaterEqual(constants["CORNER_FULL_POWER_MS"], 128)
        self.assertLessEqual(constants["CORNER_FULL_POWER_MS"], 136)
        self.assertGreaterEqual(constants["CORNER_REVERSE_COMMAND"], 915)
        self.assertLessEqual(constants["CORNER_REVERSE_COMMAND"], 945)
        self.assertGreaterEqual(constants["CORNER_SEARCH_OUTER_COMMAND"], 810)
        self.assertLessEqual(constants["CORNER_SEARCH_OUTER_COMMAND"], 835)
        self.assertGreaterEqual(constants["CORNER_SEARCH_REVERSE_COMMAND"], 155)
        self.assertLessEqual(constants["CORNER_SEARCH_REVERSE_COMMAND"], 190)
        self.assertGreaterEqual(constants["CORNER_MIN_HOLD_MS"], 120)
        self.assertLessEqual(constants["CORNER_MIN_HOLD_MS"], 135)
        self.assertLessEqual(constants["CORNER_REACQUIRE_FRAMES"], 1)
        self.assertGreaterEqual(constants["CORNER_EXIT_ERROR"], 620)
        self.assertLessEqual(constants["CORNER_CENTER_MIN"], 260)
        self.assertGreaterEqual(constants["CORNER_EDGE_MIN"], 500)
        self.assertLessEqual(constants["CORNER_EDGE_MIN"], 520)
        self.assertGreaterEqual(constants["CORNER_SIDE_SUM_MIN"], 680)
        self.assertLessEqual(constants["CORNER_SIDE_SUM_MIN"], 720)
        self.assertGreaterEqual(constants["CORNER_EDGE_DELTA"], 70)
        self.assertIn("corner_candidate_direction", source)
        self.assertIn("corner_candidate_frames", source)
        self.assertIn('state = "CROSS_APPROACH"', source)
        self.assertIn("CROSS_APPROACH_GUARD_MS", source)
        self.assertNotIn("block_cross_approach_corner = False", source)

    def test_confirmed_corner_runs_before_cross_detection_in_loop(self):
        source = MAIN_PATH.read_text(encoding="utf-8")

        self.assertLess(
            source.index("if corner_active:"),
            source.index("cross_detected_now = is_cross_detected()")
        )

    def test_corner_candidate_uses_slow_preturn_before_full_lock(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        constants = load_constants(parse_main())
        namespace = load_functions_namespace("corner_preturn_commands")

        left_turn = namespace["corner_preturn_commands"](-1)
        right_turn = namespace["corner_preturn_commands"](1)

        self.assertIn('state = "CORNER_PRETURN"', source)
        self.assertIn("corner_preturn_commands(", source)
        self.assertIn("corner_direction_candidate", source)
        self.assertGreaterEqual(constants["CORNER_PRETURN_OUTER_COMMAND"], 540)
        self.assertLessEqual(constants["CORNER_PRETURN_OUTER_COMMAND"], 590)
        self.assertGreaterEqual(constants["CORNER_PRETURN_REVERSE_COMMAND"], 330)
        self.assertLessEqual(constants["CORNER_PRETURN_REVERSE_COMMAND"], 390)
        self.assertLess(
            constants["CORNER_PRETURN_OUTER_COMMAND"],
            constants["CORNER_OUTER_COMMAND"]
        )
        self.assertLess(
            constants["CORNER_PRETURN_REVERSE_COMMAND"],
            constants["CORNER_REVERSE_COMMAND"]
        )
        self.assertLess(left_turn[0], 0)
        self.assertGreater(left_turn[1], 0)
        self.assertGreater(right_turn[0], 0)
        self.assertLess(right_turn[1], 0)

    def test_corner_search_keeps_reversing_inner_wheel_for_tighter_turn(self):
        constants = load_constants(parse_main())
        namespace = load_functions_namespace("corner_search_commands")

        left_turn = namespace["corner_search_commands"](-1)
        right_turn = namespace["corner_search_commands"](1)

        self.assertLess(left_turn[0], 0)
        self.assertGreaterEqual(left_turn[1], constants["CORNER_SEARCH_OUTER_COMMAND"])
        self.assertGreaterEqual(right_turn[0], constants["CORNER_SEARCH_OUTER_COMMAND"])
        self.assertLess(right_turn[1], 0)
        self.assertGreaterEqual(
            abs(left_turn[0]),
            constants["CORNER_SEARCH_REVERSE_COMMAND"]
        )
        self.assertGreaterEqual(
            abs(right_turn[1]),
            constants["CORNER_SEARCH_REVERSE_COMMAND"]
        )

    def test_speed_profile_is_raised_without_touching_corner_guard(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        compact_source = "".join(source.split())
        constants = load_constants(parse_main())

        self.assertGreaterEqual(constants["BASE_COMMAND"], 965)
        self.assertLessEqual(constants["BASE_COMMAND"], 975)
        self.assertGreaterEqual(constants["CLOSED_LOOP_REFERENCE_RPM"], 285.0)
        self.assertLessEqual(constants["CLOSED_LOOP_REFERENCE_RPM"], 288.0)
        self.assertGreaterEqual(constants["CLOSED_LOOP_MAX_RPM"], 424.0)
        self.assertLessEqual(constants["CLOSED_LOOP_MAX_RPM"], 430.0)
        self.assertGreaterEqual(constants["CLOSED_LOOP_TARGET_STEP_RPM"], 82.0)
        self.assertLessEqual(constants["CLOSED_LOOP_TARGET_STEP_RPM"], 87.0)
        self.assertGreaterEqual(constants["CROSS_HOLD_COMMAND"], 750)
        self.assertEqual(constants["CORNER_CONFIRM_FRAMES"], 2)
        self.assertGreaterEqual(constants["CROSS_APPROACH_GUARD_MS"], 24)
        self.assertLessEqual(constants["CROSS_APPROACH_GUARD_MS"], 36)
        self.assertGreaterEqual(constants["CROSS_APPROACH_COMMAND"], 920)
        self.assertLessEqual(constants["CROSS_APPROACH_COMMAND"], 940)
        self.assertIn(
            "cross_straight_commands(cross_hold_command,cross_hold_error,cross_hold_bias)",
            compact_source
        )
        self.assertIn(
            "cross_approach_commands(cross_approach_error,counterclockwise_entry_cross_bias(cross_count))",
            compact_source
        )

    def test_keeps_latest_code_cross_state_machine_without_extra_circle_mode(self):
        source = MAIN_PATH.read_text(encoding="utf-8")

        self.assertIn('state = "CROSS_APPROACH"', source)
        self.assertIn('state = "CROSS_HOLD"', source)
        self.assertIn('state = "CORNER_FULL"', source)
        self.assertNotIn("ENTRY_CROSS_GUARD", source)
        self.assertNotIn("CIRCLE_RIGHT", source)
        self.assertNotIn("circle_mode_active", source)
        self.assertNotIn("first_cross_recovery_command", source)

    def test_cross_hold_keeps_entry_heading_with_limited_correction(self):
        constants = load_constants(parse_main())
        namespace = load_functions_namespace(
            "clamp",
            "cross_straight_commands",
            "cross_hold_commands"
        )

        straight = namespace["cross_hold_commands"](0)
        right_correction = namespace["cross_hold_commands"](900)
        left_correction = namespace["cross_hold_commands"](-900)
        saturated = namespace["cross_hold_commands"](5000)
        first_cross = namespace["cross_hold_commands"](
            0,
            constants["COUNTERCLOCKWISE_FIRST_ENTRY_BIAS"]
        )
        later_entry = namespace["cross_hold_commands"](
            0,
            constants["COUNTERCLOCKWISE_ENTRY_LEFT_BIAS"]
        )

        self.assertEqual(
            straight,
            (
                constants["CROSS_HOLD_COMMAND"],
                constants["CROSS_HOLD_COMMAND"]
            )
        )
        self.assertGreater(right_correction[0], constants["CROSS_HOLD_COMMAND"])
        self.assertLess(right_correction[1], constants["CROSS_HOLD_COMMAND"])
        self.assertLess(left_correction[0], constants["CROSS_HOLD_COMMAND"])
        self.assertGreater(left_correction[1], constants["CROSS_HOLD_COMMAND"])
        self.assertLessEqual(
            saturated[0] - constants["CROSS_HOLD_COMMAND"],
            constants["CROSS_HOLD_MAX_CORRECTION"]
        )
        self.assertGreaterEqual(
            saturated[1],
            constants["CROSS_HOLD_COMMAND"] -
            constants["CROSS_HOLD_MAX_CORRECTION"]
        )
        self.assertLess(first_cross[0], constants["CROSS_HOLD_COMMAND"])
        self.assertGreater(first_cross[1], constants["CROSS_HOLD_COMMAND"])
        self.assertLessEqual(
            constants["CROSS_HOLD_COMMAND"] - first_cross[0],
            constants["CROSS_HOLD_MAX_CORRECTION"]
        )
        self.assertEqual(later_entry[0], constants["CROSS_HOLD_COMMAND"])
        self.assertEqual(later_entry[1], constants["CROSS_HOLD_COMMAND"])
        self.assertGreaterEqual(constants["CROSS_HOLD_MS"], 115)
        self.assertLessEqual(constants["CROSS_HOLD_MS"], 130)
        self.assertGreaterEqual(constants["CROSS_HOLD_MAX_CORRECTION"], 70)
        self.assertLessEqual(constants["CROSS_HOLD_MAX_CORRECTION"], 82)
        self.assertLess(constants["COUNTERCLOCKWISE_FIRST_ENTRY_BIAS"], 0)
        self.assertLessEqual(
            abs(constants["COUNTERCLOCKWISE_ENTRY_LEFT_BIAS"]),
            constants["CROSS_HOLD_MAX_CORRECTION"]
        )
        self.assertEqual(constants["COUNTERCLOCKWISE_ENTRY_LEFT_BIAS"], 0)

    def test_counterclockwise_entry_cross_bias_is_neutral_for_all_crosses(self):
        constants = load_constants(parse_main())
        source = MAIN_PATH.read_text(encoding="utf-8")
        namespace = load_functions_namespace(
            "should_guard_entry_cross",
            "counterclockwise_entry_cross_bias"
        )

        expected_biases = (
            constants["COUNTERCLOCKWISE_FIRST_ENTRY_BIAS"],
            0.0,
            constants["COUNTERCLOCKWISE_FIRST_ENTRY_BIAS"],
            0.0,
            constants["COUNTERCLOCKWISE_FIRST_ENTRY_BIAS"],
        )

        self.assertEqual(
            tuple(
                namespace["counterclockwise_entry_cross_bias"](cross_count)
                for cross_count in range(5)
            ),
            expected_biases
        )
        self.assertTrue(namespace["should_guard_entry_cross"](4))
        self.assertNotIn("def clockwise_entry_cross_bias", source)

    def test_counterclockwise_exit_cross_holds_straight_without_circle_error(self):
        constants = load_constants(parse_main())
        namespace = load_functions_namespace(
            "clamp",
            "cross_entry_error",
            "should_guard_entry_cross",
            "counterclockwise_cross_entry_error",
            "counterclockwise_cross_hold_error",
            "counterclockwise_cross_hold_ms",
            "counterclockwise_cross_hold_command"
        )

        namespace["last_error"] = 650.0

        self.assertNotEqual(namespace["cross_entry_error"](2500), 0.0)
        self.assertEqual(namespace["counterclockwise_cross_entry_error"](0, 2500), 0.0)
        self.assertEqual(namespace["counterclockwise_cross_entry_error"](1, 2500), 0.0)
        self.assertEqual(namespace["counterclockwise_cross_entry_error"](2, 2500), 0.0)
        self.assertEqual(namespace["counterclockwise_cross_entry_error"](3, None), 0.0)
        self.assertEqual(
            namespace["counterclockwise_cross_hold_error"](0, True, -650.0, 2500),
            0.0
        )
        self.assertEqual(
            namespace["counterclockwise_cross_hold_error"](1, True, -650.0, 2500),
            0.0
        )
        self.assertEqual(
            namespace["counterclockwise_cross_hold_error"](2, True, -320.0, 2500),
            0.0
        )
        self.assertEqual(
            namespace["counterclockwise_cross_hold_ms"](0),
            constants["CROSS_HOLD_MS"]
        )
        self.assertGreater(
            namespace["counterclockwise_cross_hold_ms"](1),
            namespace["counterclockwise_cross_hold_ms"](0)
        )
        self.assertEqual(
            namespace["counterclockwise_cross_hold_command"](0),
            constants["CROSS_HOLD_COMMAND"]
        )
        self.assertEqual(
            namespace["counterclockwise_cross_hold_command"](2),
            constants["CROSS_HOLD_COMMAND"]
        )
        self.assertLess(
            namespace["counterclockwise_cross_hold_command"](1),
            namespace["counterclockwise_cross_hold_command"](0)
        )
        self.assertGreaterEqual(constants["CROSS_EXIT_HOLD_MS"], 170)
        self.assertLessEqual(constants["CROSS_EXIT_HOLD_MS"], 190)
        self.assertGreaterEqual(constants["CROSS_EXIT_HOLD_COMMAND"], 600)
        self.assertLessEqual(constants["CROSS_EXIT_HOLD_COMMAND"], 640)

    def test_cross_exit_lost_line_uses_short_straight_reacquire(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        constants = load_constants(parse_main())
        namespace = load_functions_namespace("post_cross_reacquire_commands")

        self.assertIn('state = "CROSS_REACQUIRE"', source)
        self.assertIn("post_cross_reacquire_active", source)
        self.assertIn("POST_CROSS_REACQUIRE_MS", source)
        self.assertGreaterEqual(constants["POST_CROSS_REACQUIRE_MS"], 80)
        self.assertLessEqual(constants["POST_CROSS_REACQUIRE_MS"], 115)
        self.assertGreaterEqual(constants["POST_CROSS_REACQUIRE_COMMAND"], 500)
        self.assertLessEqual(constants["POST_CROSS_REACQUIRE_COMMAND"], 560)
        self.assertEqual(
            namespace["post_cross_reacquire_commands"](),
            (
                constants["POST_CROSS_REACQUIRE_COMMAND"],
                constants["POST_CROSS_REACQUIRE_COMMAND"]
            )
        )

    def test_cross_rearm_requires_clear_zone_before_next_count(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        constants = load_constants(parse_main())
        namespace = load_functions_namespace("update_cross_rearm")

        clear_frames, armed = namespace["update_cross_rearm"](5, True, 1000)
        self.assertEqual(clear_frames, 0)
        self.assertFalse(armed)

        clear_frames, armed = namespace["update_cross_rearm"](
            constants["CROSS_REARM_CLEAR_FRAMES"],
            False,
            constants["CROSS_REARM_MIN_MS"] - 1
        )
        self.assertFalse(armed)

        clear_frames = 0
        armed = False
        for _ in range(constants["CROSS_REARM_CLEAR_FRAMES"] - 1):
            clear_frames, armed = namespace["update_cross_rearm"](
                clear_frames,
                False,
                constants["CROSS_REARM_MIN_MS"]
            )
            self.assertFalse(armed)

        clear_frames, armed = namespace["update_cross_rearm"](
            clear_frames,
            False,
            constants["CROSS_REARM_MIN_MS"]
        )
        self.assertTrue(armed)
        self.assertGreaterEqual(constants["CROSS_REARM_CLEAR_FRAMES"], 6)
        self.assertLessEqual(constants["CROSS_REARM_CLEAR_FRAMES"], 12)
        self.assertGreaterEqual(constants["CROSS_REARM_MIN_MS"], 600)
        self.assertLessEqual(constants["CROSS_REARM_MIN_MS"], 900)
        self.assertIn("cross_armed", source)
        self.assertIn("cross_clear_frames", source)
        self.assertIn("cross_rearm_start", source)

    def test_curve_base_command_slows_medium_and_fast_changing_errors(self):
        constants = load_constants(parse_main())
        namespace = load_functions_namespace("clamp", "curve_base_command")
        base_command = constants["BASE_COMMAND"]

        self.assertEqual(
            namespace["curve_base_command"](base_command, 0),
            base_command
        )
        self.assertLessEqual(
            namespace["curve_base_command"](base_command, 250),
            base_command
        )
        self.assertLessEqual(
            namespace["curve_base_command"](base_command, 250),
            base_command
        )
        self.assertGreaterEqual(
            namespace["curve_base_command"](base_command, 250),
            base_command - 35
        )
        self.assertLessEqual(
            namespace["curve_base_command"](base_command, 250),
            base_command - 15
        )
        fast_changing = namespace["curve_base_command"](
            base_command,
            100,
            320
        )
        self.assertLessEqual(fast_changing, base_command - 5)
        self.assertGreaterEqual(fast_changing, base_command - 25)

        medium = namespace["curve_base_command"](base_command, 800)
        large = namespace["curve_base_command"](base_command, 1500)

        self.assertLess(medium, base_command)
        self.assertLessEqual(medium, base_command - 115)
        self.assertGreaterEqual(medium, base_command - 135)
        self.assertLessEqual(large, base_command - 150)
        self.assertGreaterEqual(large, base_command - 165)

    def test_straight_settle_damps_after_curve_reacquire(self):
        constants = load_constants(parse_main())
        namespace = load_functions_namespace(
            "clamp",
            "should_start_straight_settle",
            "apply_straight_settle",
            "start_straight_settle",
            "reset_pid_history"
        )

        self.assertTrue(namespace["should_start_straight_settle"](620, 120))
        self.assertTrue(namespace["should_start_straight_settle"](-620, -120))
        self.assertFalse(namespace["should_start_straight_settle"](120, 80))
        self.assertFalse(namespace["should_start_straight_settle"](620, 420))

        damped_base, damped_turn = namespace["apply_straight_settle"](
            constants["BASE_COMMAND"],
            180.0
        )
        self.assertLessEqual(
            damped_base,
            constants["BASE_COMMAND"] -
            constants["STRAIGHT_SETTLE_COMMAND_DROP"]
        )
        self.assertLess(abs(damped_turn), 180.0)
        self.assertGreaterEqual(constants["STRAIGHT_SETTLE_FRAMES"], 45)
        self.assertLessEqual(constants["STRAIGHT_SETTLE_FRAMES"], 75)
        self.assertGreaterEqual(constants["STRAIGHT_SETTLE_TURN_SCALE"], 0.55)
        self.assertLessEqual(constants["STRAIGHT_SETTLE_TURN_SCALE"], 0.70)

        namespace["straight_settle_frames"] = 0
        namespace["start_straight_settle"]()
        self.assertEqual(
            namespace["straight_settle_frames"],
            constants["STRAIGHT_SETTLE_FRAMES"]
        )
        namespace["reset_pid_history"]()
        self.assertEqual(namespace["straight_settle_frames"], 0)

    def test_startup_is_softened_while_cruise_speed_is_high(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        constants = load_constants(parse_main())
        namespace = load_functions_namespace("startup_base_command")

        self.assertGreaterEqual(constants["STARTUP_SOFT_START_MS"], 650)
        self.assertLessEqual(constants["STARTUP_SOFT_START_COMMAND"], 680)
        self.assertLessEqual(constants["LEFT_START_BOOST_PWM"], 680)
        self.assertLessEqual(constants["RIGHT_START_BOOST_PWM"], 680)
        self.assertLessEqual(constants["START_BOOST_MAX_MS"], 60)
        self.assertLessEqual(constants["START_BOOST_EXIT_RPM"], 20.0)
        self.assertIn("startup_base_command(now, start_search_start)", source)

        early = namespace["startup_base_command"](0, 0)
        middle = namespace["startup_base_command"](
            constants["STARTUP_SOFT_START_MS"] // 2,
            0
        )
        full = namespace["startup_base_command"](
            constants["STARTUP_SOFT_START_MS"] + 1,
            0
        )

        self.assertEqual(early, constants["STARTUP_SOFT_START_COMMAND"])
        self.assertLess(early, middle)
        self.assertLess(middle, constants["BASE_COMMAND"])
        self.assertEqual(full, constants["BASE_COMMAND"])

    def test_filtered_derivative_smooths_circle_turn_response(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        constants = load_constants(parse_main())
        namespace = load_functions_namespace(
            "update_filtered_derivative",
            "reset_pid_history"
        )

        self.assertGreaterEqual(constants["PID_KP"], 0.212)
        self.assertLessEqual(constants["PID_KP"], 0.222)
        self.assertGreaterEqual(constants["PID_KD"], 0.30)
        self.assertLessEqual(constants["PID_KD"], 0.34)
        self.assertGreaterEqual(constants["COMMAND_FILTER_ALPHA"], 0.56)
        self.assertLessEqual(constants["COMMAND_FILTER_ALPHA"], 0.60)
        self.assertGreaterEqual(constants["DERIVATIVE_FILTER_ALPHA"], 0.24)
        self.assertLessEqual(constants["DERIVATIVE_FILTER_ALPHA"], 0.28)
        self.assertIn("update_filtered_derivative(raw_derivative)", source)
        self.assertIn("last_derivative", source)
        self.assertNotIn("apply_curve_turn_floor", source)
        self.assertNotIn("apply_circle_turn_memory", source)
        self.assertNotIn("circle_turn_memory", source)

        namespace["last_derivative"] = 0.0
        first = namespace["update_filtered_derivative"](400.0)
        second = namespace["update_filtered_derivative"](400.0)
        reversal = namespace["update_filtered_derivative"](-400.0)

        self.assertGreaterEqual(first, 95.0)
        self.assertLess(first, 125.0)
        self.assertGreater(second, first)
        self.assertGreater(reversal, -80.0)

        namespace["reset_pid_history"]()
        self.assertEqual(namespace["last_derivative"], 0.0)
        self.assertGreaterEqual(constants["SPEED_FILTER_ALPHA"], 0.26)
        self.assertLessEqual(constants["SPEED_FILTER_ALPHA"], 0.30)
        self.assertLessEqual(constants["LEFT_SPEED_KP"], 0.42)
        self.assertLessEqual(constants["RIGHT_SPEED_KP"], 0.42)
        self.assertLessEqual(constants["LEFT_SPEED_KI"], 1.05)
        self.assertLessEqual(constants["RIGHT_SPEED_KI"], 1.05)

    def test_key_runtime_parameters_have_inline_notes(self):
        source = MAIN_PATH.read_text(encoding="utf-8")
        key_constants = (
            "BASE_COMMAND",
            "CLOSED_LOOP_REFERENCE_RPM",
            "CLOSED_LOOP_MAX_RPM",
            "PID_KP",
            "PID_KD",
            "COMMAND_FILTER_ALPHA",
            "CURVE_SLOWDOWN_START_ERROR",
            "CURVE_SLOWDOWN_FULL_ERROR",
            "CURVE_SLOWDOWN_MAX_COMMAND",
            "CURVE_DERIVATIVE_SLOWDOWN_GAIN",
            "STRAIGHT_SETTLE_SOURCE_ERROR",
            "STRAIGHT_SETTLE_ENTRY_ERROR",
            "STRAIGHT_SETTLE_FRAMES",
            "STRAIGHT_SETTLE_COMMAND_DROP",
            "STRAIGHT_SETTLE_TURN_SCALE",
            "LONG_CURVE_COMMAND_DROP",
            "LONG_CURVE_TURN_SCALE",
            "CROSS_APPROACH_GUARD_MS",
            "CROSS_HOLD_COMMAND",
            "CROSS_EXIT_HOLD_COMMAND",
            "CROSS_EXIT_HOLD_MS",
            "POST_CROSS_REACQUIRE_MS",
            "POST_CROSS_REACQUIRE_COMMAND",
            "COUNTERCLOCKWISE_FIRST_ENTRY_BIAS",
            "COUNTERCLOCKWISE_ENTRY_LEFT_BIAS",
            "CROSS_REARM_CLEAR_FRAMES",
            "CROSS_REARM_MIN_MS",
            "CORNER_PRETURN_OUTER_COMMAND",
            "CORNER_PRETURN_REVERSE_COMMAND",
            "CORNER_FULL_POWER_MS",
            "CORNER_SEARCH_REVERSE_COMMAND",
            "CORNER_EDGE_MIN",
            "LOST_REVERSE_COMMAND",
        )

        for constant_name in key_constants:
            self.assertIn("#", assignment_line(source, constant_name))


if __name__ == "__main__":
    unittest.main()
