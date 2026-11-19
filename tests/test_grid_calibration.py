import ast
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MAIN_PATH = ROOT / "main.py"


def load_calibration_namespace():
    tree = ast.parse(MAIN_PATH.read_text(encoding="utf-8"))
    wanted_assignments = {
        "COLS", "ROWS", "x0", "y0", "dx", "dy",
        "my_threshold", "my_threshold_fallback", "LAB_THRESHOLDS",
        "active_wall_threshold", "CALIB_WALL_THRESHOLD",
        "CALIB_INTERVAL_MS", "CALIB_SCAN_STEP", "EDGE_SAMPLE_STEP",
        "EDGE_OFFSET", "MIN_EDGE_COVERAGE", "EDGE_CANDIDATE_N",
        "AXIS_PAIR_N", "RECT_CANDIDATE_N", "MIN_CELL", "MAX_CELL",
        "MIN_CELL_RATIO", "MAX_CELL_RATIO", "MIN_RECT_QUALITY",
        "MIN_RECT_MARGIN", "CALIB_CONSENSUS_N", "CALIB_POSITION_TOL",
        "CALIB_CELL_TOL",
    }
    wanted_functions = {
        "get_lab_mean", "is_wall_lab", "is_calib_wall_lab",
        "is_floor_lab", "is_known_inner_lab", "edge_evidence",
        "vertical_edge_scores", "horizontal_edge_scores",
        "top_candidates", "refine_vertical_candidates",
        "refine_horizontal_candidates", "choose_axis_pairs", "median",
        "rectangle_quality", "detect_grid_once",
        "same_calibration_cluster",
    }

    nodes = []
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = [target.id for target in node.targets
                     if isinstance(target, ast.Name)]
            if any(name in wanted_assignments for name in names):
                nodes.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in wanted_functions:
            nodes.append(node)

    module = ast.Module(body=nodes, type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "in_threshold": lambda l, a, b, th: (
            th[0] <= l <= th[1]
            and th[2] <= a <= th[3]
            and th[4] <= b <= th[5]
        )
    }
    exec(compile(module, str(MAIN_PATH), "exec"), namespace)
    return namespace


class FakeStatistics:
    def __init__(self, lab):
        self._lab = lab

    def l_mean(self):
        return self._lab[0]

    def a_mean(self):
        return self._lab[1]

    def b_mean(self):
        return self._lab[2]


class SyntheticMapImage:
    def __init__(self, x0=None, y0=None, dx=16.0, dy=16.0):
        self.x0 = x0
        self.y0 = y0
        self.dx = dx
        self.dy = dy

    def width(self):
        return 320

    def height(self):
        return 240

    def _lab_at(self, x, y):
        if self.x0 is None:
            return (5, 0, 0)

        left = self.x0 - self.dx * 0.5
        right = self.x0 + 15.5 * self.dx
        top = self.y0 - self.dy * 0.5
        bottom = self.y0 + 11.5 * self.dy
        if not (left <= x <= right and top <= y <= bottom):
            return (5, 0, 0)

        col = int((x - left) / self.dx)
        row = int((y - top) / self.dy)
        col = max(0, min(15, col))
        row = max(0, min(11, row))

        if row in (0, 11) or col in (0, 15):
            return (65, 5, -10)

        # 加入长内墙，验证它不会抢走外围墙的四边联合判定。
        if ((col == 5 and 2 <= row <= 8)
                or (row == 4 and 5 <= col <= 11)
                or (col == 11 and 4 <= row <= 9)):
            return (65, 5, -10)
        return (42, 0, -95)

    def get_statistics(self, roi):
        x, y, w, h = roi
        return FakeStatistics(self._lab_at(x + w // 2, y + h // 2))


class GridCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ns = load_calibration_namespace()

    def assert_detects(self, x0, y0, dx, dy):
        result = self.ns["detect_grid_once"](
            SyntheticMapImage(x0=x0, y0=y0, dx=dx, dy=dy)
        )
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[0], x0, delta=2.0)
        self.assertAlmostEqual(result[1], y0, delta=2.0)
        self.assertAlmostEqual(result[2], dx, delta=0.5)
        self.assertAlmostEqual(result[3], dy, delta=0.5)

    def test_shifted_map_is_not_anchored_to_old_fallback(self):
        self.assert_detects(48.0, 24.0, 15.5, 15.0)

    def test_map_near_right_side_is_still_detected(self):
        self.assert_detects(64.0, 26.0, 15.0, 15.0)

    def test_moderate_non_square_camera_scale_is_allowed(self):
        self.assert_detects(24.0, 25.0, 17.0, 15.0)

    def test_blank_frame_is_rejected(self):
        result = self.ns["detect_grid_once"](SyntheticMapImage())
        self.assertIsNone(result)

    def test_consensus_rejects_a_large_grid_jump(self):
        same = self.ns["same_calibration_cluster"]
        self.assertTrue(same((30, 25, 16, 15, 80),
                             (32, 27, 16.4, 15.5, 82)))
        self.assertFalse(same((30, 25, 16, 15, 80),
                              (38, 25, 16, 15, 82)))


if __name__ == "__main__":
    unittest.main()
