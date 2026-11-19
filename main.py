import sensor
import time
from machine import UART
from tracker import CarTracker

# ================= 1. 网格物理参数 (提到最前面作为全局基准) =================
COLS = 16
ROWS = 12

x0 = 33   # 自动标定失败时的兜底值
y0 = 35
dx = 16.5
dy = 15.8

# ================= 2. 摄像头、串口与追踪器初始化 =================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# 保持 sensor.reset() 后的默认自动曝光。启动等待同时用于自动曝光稳定。
# tracker.py 只处理 main.py 传入的图像，不重复控制摄像头。

clock = time.clock()

uart = UART(12, baudrate=115200)

my_threshold = (0, 95, -128, -10, -69, 69) # 自动曝光下的旧版稳定主阈值
my_threshold_fallback = (0, 95, -76, -11, -65, 60) # 自动曝光下的暗区备用阈值
# ================= 3. 核心工具与识别函数 (严格阈值判定版) =================

# 日常识别恢复到 2026-05-29 自动曝光版本的稳定阈值。
LAB_THRESHOLDS = {
    ".": (35, 65, 58, 99, -91, -38),      # 目的地：2026-06-15暗区重测
    "$": (0, 100, -38, 4, 45, 122),      # 箱子：B>=45，增强中部箱子识别
    "*": (30, 75, 32, 127, -47, 127),    # 炸弹
    "#": (37, 100, -5, 28, -60, -4),     # 自动曝光下墙体，仅用于爆炸后删墙确认
    "-": (28, 55, -59, 88, -120, -70)    # 空地
}

# 历史字符地图小车阈值：(0, 100, -127, -37, -85, 92)。
# 字符地图不再识别或写入 '@'；小车只由 CarTracker 输出连续坐标。

active_wall_threshold = LAB_THRESHOLDS["#"]

# ================= 3.1 启动期高可信网格标定 =================
# 标定只在开机执行一次，速度不是优先级。与日常墙体分类不同，
# 这里使用独立、较宽的灰墙阈值，不再因日常阈值调整而带偏红框。
CALIB_WALL_THRESHOLD = (15, 100, -13, 59, -62, 4)
CALIB_INTERVAL_MS = 80
CALIB_SCAN_STEP = 2
EDGE_SAMPLE_STEP = 6
EDGE_OFFSET = 4
MIN_EDGE_COVERAGE = 6
EDGE_CANDIDATE_N = 14
AXIS_PAIR_N = 18
RECT_CANDIDATE_N = 12
MIN_CELL = 10
MAX_CELL = 24
MIN_CELL_RATIO = 0.82
MAX_CELL_RATIO = 1.22
MIN_RECT_QUALITY = 60
MIN_RECT_MARGIN = 4
CALIB_CONSENSUS_N = 5
CALIB_POSITION_TOL = 3.0
CALIB_CELL_TOL = 0.8

def get_lab_mean(img, x, y):
    x = max(1, min(img.width() - 2, x))
    y = max(1, min(img.height() - 2, y))
    stat = img.get_statistics(roi=(x - 1, y - 1, 3, 3))
    return stat.l_mean(), stat.a_mean(), stat.b_mean()

def is_wall_lab(l, a, b):
    return in_threshold(l, a, b, active_wall_threshold)

def is_calib_wall_lab(lab):
    return in_threshold(lab[0], lab[1], lab[2], CALIB_WALL_THRESHOLD)

def is_floor_lab(lab):
    return in_threshold(lab[0], lab[1], lab[2], LAB_THRESHOLDS["-"])

def is_known_inner_lab(lab):
    l, a, b = lab
    return (
        in_threshold(l, a, b, LAB_THRESHOLDS["-"])
        or in_threshold(l, a, b, LAB_THRESHOLDS["."])
        or in_threshold(l, a, b, LAB_THRESHOLDS["$"])
        or in_threshold(l, a, b, LAB_THRESHOLDS["*"])
        or in_threshold(l, a, b, my_threshold)
        or in_threshold(l, a, b, my_threshold_fallback)
    )

def edge_evidence(wall_lab, inner_lab):
    if not is_calib_wall_lab(wall_lab):
        return 0, 0
    if is_floor_lab(inner_lab):
        return 8, 1
    if is_known_inner_lab(inner_lab):
        return 4, 1
    return 1, 0

def vertical_edge_scores(img, x):
    left_score = 0
    left_coverage = 0
    right_score = 0
    right_coverage = 0
    for y in range(4, img.height() - 4, EDGE_SAMPLE_STEP):
        lab_left = get_lab_mean(img, x - EDGE_OFFSET, y)
        lab_right = get_lab_mean(img, x + EDGE_OFFSET, y)
        score, coverage = edge_evidence(lab_left, lab_right)
        left_score += score
        left_coverage += coverage
        score, coverage = edge_evidence(lab_right, lab_left)
        right_score += score
        right_coverage += coverage
    return left_score, left_coverage, right_score, right_coverage

def horizontal_edge_scores(img, y):
    top_score = 0
    top_coverage = 0
    bottom_score = 0
    bottom_coverage = 0
    for x in range(4, img.width() - 4, EDGE_SAMPLE_STEP):
        lab_top = get_lab_mean(img, x, y - EDGE_OFFSET)
        lab_bottom = get_lab_mean(img, x, y + EDGE_OFFSET)
        score, coverage = edge_evidence(lab_top, lab_bottom)
        top_score += score
        top_coverage += coverage
        score, coverage = edge_evidence(lab_bottom, lab_top)
        bottom_score += score
        bottom_coverage += coverage
    return top_score, top_coverage, bottom_score, bottom_coverage

def top_candidates(score_list, limit=EDGE_CANDIDATE_N):
    candidates = []
    for pos, score, coverage in score_list:
        if coverage >= MIN_EDGE_COVERAGE:
            candidates.append((pos, score + coverage * 12))

    # x±EDGE_OFFSET 分别落在墙/地面时会形成数像素宽的高分平台。
    # 平台中点才是真正分界线，不能把平台两端当成两条边。
    candidates.sort()
    groups = []
    current = []
    for pos, strength in candidates:
        if current and pos - current[-1][0] > CALIB_SCAN_STEP + 1:
            groups.append(current)
            current = []
        current.append((pos, strength))
    if current:
        groups.append(current)

    ranked = []
    for group in groups:
        total_strength = sum(item[1] for item in group)
        center = int(
            sum(item[0] * item[1] for item in group) / total_strength + 0.5
        )
        strength = max(item[1] for item in group)
        ranked.append((strength, center))

    ranked.sort(reverse=True)
    return [(center, strength) for strength, center in ranked[:limit]]

def refine_vertical_candidates(img, coarse, side):
    scores = []
    seen = []
    for pos, _ in coarse:
        for refined in range(pos - CALIB_SCAN_STEP, pos + CALIB_SCAN_STEP + 1):
            if refined < EDGE_OFFSET + 2 or refined >= img.width() - EDGE_OFFSET - 2:
                continue
            if refined in seen:
                continue
            seen.append(refined)
            left_s, left_c, right_s, right_c = vertical_edge_scores(img, refined)
            if side == "left":
                scores.append((refined, left_s, left_c))
            else:
                scores.append((refined, right_s, right_c))
    return top_candidates(scores)

def refine_horizontal_candidates(img, coarse, side):
    scores = []
    seen = []
    for pos, _ in coarse:
        for refined in range(pos - CALIB_SCAN_STEP, pos + CALIB_SCAN_STEP + 1):
            if refined < EDGE_OFFSET + 2 or refined >= img.height() - EDGE_OFFSET - 2:
                continue
            if refined in seen:
                continue
            seen.append(refined)
            top_s, top_c, bottom_s, bottom_c = horizontal_edge_scores(img, refined)
            if side == "top":
                scores.append((refined, top_s, top_c))
            else:
                scores.append((refined, bottom_s, bottom_c))
    return top_candidates(scores)

def choose_axis_pairs(starts, ends, inner_cells):
    pairs = []
    for start, start_strength in starts:
        for end, end_strength in ends:
            span = end - start
            cell = span / inner_cells
            if MIN_CELL <= cell <= MAX_CELL:
                pairs.append((start_strength + end_strength, start, end, cell))
    pairs.sort(reverse=True)
    return pairs[:AXIS_PAIR_N]

def median(values):
    vals = values[:]
    vals.sort()
    return vals[len(vals) // 2]

def rectangle_quality(img, left, right, top, bottom, cell_x, cell_y):
    # 红线是真正的“外围墙 / 地图内部”分界线：
    # 外侧向墙中心取样，内侧向第一圈内部格取样，不能只看交界反差。
    offset_x = cell_x * 0.38
    offset_y = cell_y * 0.38
    if (left - offset_x < 2 or right + offset_x >= img.width() - 2
            or top - offset_y < 2 or bottom + offset_y >= img.height() - 2):
        return None

    side_stats = []

    # 水平两边各取 14 个分散样本，对应 14 格内边界跨度。
    for edge_y, outside_y, inside_y in (
            (top, top - offset_y, top + offset_y),
            (bottom, bottom + offset_y, bottom - offset_y)):
        wall_hits = 0
        floor_hits = 0
        known_hits = 0
        total = COLS - 2
        for i in range(total):
            x = left + (i + 0.5) * (right - left) / total
            outside_lab = get_lab_mean(img, int(x), int(outside_y))
            inside_lab = get_lab_mean(img, int(x), int(inside_y))
            if is_calib_wall_lab(outside_lab):
                wall_hits += 1
            if is_floor_lab(inside_lab):
                floor_hits += 1
                known_hits += 1
            elif is_known_inner_lab(inside_lab) or is_calib_wall_lab(inside_lab):
                known_hits += 1
        side_stats.append((wall_hits / total,
                           floor_hits / total,
                           known_hits / total))

    # 垂直两边各取 10 个分散样本，对应 10 格内边界跨度。
    for edge_x, outside_x, inside_x in (
            (left, left - offset_x, left + offset_x),
            (right, right + offset_x, right - offset_x)):
        wall_hits = 0
        floor_hits = 0
        known_hits = 0
        total = ROWS - 2
        for i in range(total):
            y = top + (i + 0.5) * (bottom - top) / total
            outside_lab = get_lab_mean(img, int(outside_x), int(y))
            inside_lab = get_lab_mean(img, int(inside_x), int(y))
            if is_calib_wall_lab(outside_lab):
                wall_hits += 1
            if is_floor_lab(inside_lab):
                floor_hits += 1
                known_hits += 1
            elif is_known_inner_lab(inside_lab) or is_calib_wall_lab(inside_lab):
                known_hits += 1
        side_stats.append((wall_hits / total,
                           floor_hits / total,
                           known_hits / total))

    # 每条边单独通过：外侧至少约 70% 是墙，内侧至少一半是蓝地，
    # 且至少 75% 属于已知地图内容。内部 8 格墙和屏幕外杂物无法靠别边补分。
    for wall_ratio, floor_ratio, known_ratio in side_stats:
        if wall_ratio < 0.68 or floor_ratio < 0.50 or known_ratio < 0.75:
            return None

    wall_avg = sum(item[0] for item in side_stats) / len(side_stats)
    floor_avg = sum(item[1] for item in side_stats) / len(side_stats)
    known_avg = sum(item[2] for item in side_stats) / len(side_stats)
    square_penalty = abs(cell_x / cell_y - 1.0) * 35
    return wall_avg * 50 + floor_avg * 30 + known_avg * 20 - square_penalty

def detect_grid_once(img):
    # 恢复原版近邻搜索：摄像头轻微挪动时仍会重新标定，但不会把远处杂物
    # 或大段内部墙直接纳入四边候选。14/10 分别是横纵内边界的格数。
    search_margin = 45
    left_expected = x0 + 0.5 * dx
    right_expected = x0 + (COLS - 1.5) * dx
    top_expected = y0 + 0.5 * dy
    bottom_expected = y0 + (ROWS - 1.5) * dy

    left_scores = []
    right_scores = []
    left_lo = max(EDGE_OFFSET + 2, int(left_expected - search_margin))
    left_hi = min(img.width() - EDGE_OFFSET - 3,
                  int(left_expected + search_margin))
    right_lo = max(EDGE_OFFSET + 2, int(right_expected - search_margin))
    right_hi = min(img.width() - EDGE_OFFSET - 3,
                   int(right_expected + search_margin))
    for x in range(left_lo, left_hi + 1, CALIB_SCAN_STEP):
        left_s, left_c, right_s, right_c = vertical_edge_scores(img, x)
        left_scores.append((x, left_s, left_c))
    for x in range(right_lo, right_hi + 1, CALIB_SCAN_STEP):
        left_s, left_c, right_s, right_c = vertical_edge_scores(img, x)
        right_scores.append((x, right_s, right_c))

    top_scores = []
    bottom_scores = []
    top_lo = max(EDGE_OFFSET + 2, int(top_expected - search_margin))
    top_hi = min(img.height() - EDGE_OFFSET - 3,
                 int(top_expected + search_margin))
    bottom_lo = max(EDGE_OFFSET + 2, int(bottom_expected - search_margin))
    bottom_hi = min(img.height() - EDGE_OFFSET - 3,
                    int(bottom_expected + search_margin))
    for y in range(top_lo, top_hi + 1, CALIB_SCAN_STEP):
        top_s, top_c, bottom_s, bottom_c = horizontal_edge_scores(img, y)
        top_scores.append((y, top_s, top_c))
    for y in range(bottom_lo, bottom_hi + 1, CALIB_SCAN_STEP):
        top_s, top_c, bottom_s, bottom_c = horizontal_edge_scores(img, y)
        bottom_scores.append((y, bottom_s, bottom_c))

    lefts = refine_vertical_candidates(img, top_candidates(left_scores), "left")
    rights = refine_vertical_candidates(img, top_candidates(right_scores), "right")
    tops = refine_horizontal_candidates(img, top_candidates(top_scores), "top")
    bottoms = refine_horizontal_candidates(img, top_candidates(bottom_scores), "bottom")

    x_pairs = choose_axis_pairs(lefts, rights, COLS - 2)
    y_pairs = choose_axis_pairs(tops, bottoms, ROWS - 2)
    if len(x_pairs) == 0 or len(y_pairs) == 0:
        return None

    rectangles = []
    for x_strength, left, right, cell_x in x_pairs:
        for y_strength, top, bottom, cell_y in y_pairs:
            cell_ratio = cell_x / cell_y
            if not (MIN_CELL_RATIO <= cell_ratio <= MAX_CELL_RATIO):
                continue
            pre_score = (
                x_strength + y_strength
                - abs(cell_x - dx) * 80
                - abs(cell_y - dy) * 80
                - abs(cell_ratio - 1.0) * 200
            )
            rectangles.append((pre_score, left, right, top, bottom, cell_x, cell_y))

    rectangles.sort(reverse=True)
    best = None
    second_quality = None
    for item in rectangles[:RECT_CANDIDATE_N]:
        _, left, right, top, bottom, cell_x, cell_y = item
        quality = rectangle_quality(img, left, right, top, bottom, cell_x, cell_y)
        if quality is None or quality < MIN_RECT_QUALITY:
            continue
        detected = (
            left - cell_x * 0.5,
            top - cell_y * 0.5,
            cell_x,
            cell_y,
            quality,
        )
        if best is None or quality > best[4]:
            if best is not None:
                second_quality = best[4]
            best = detected
        elif second_quality is None or quality > second_quality:
            second_quality = quality

    if best is None:
        return None
    if second_quality is not None and best[4] - second_quality < MIN_RECT_MARGIN:
        return None
    return best

def same_calibration_cluster(a, b):
    return (
        abs(a[0] - b[0]) <= CALIB_POSITION_TOL
        and abs(a[1] - b[1]) <= CALIB_POSITION_TOL
        and abs(a[2] - b[2]) <= CALIB_CELL_TOL
        and abs(a[3] - b[3]) <= CALIB_CELL_TOL
    )

def calibrate_grid():
    stable_results = []
    while True:
        img = sensor.snapshot()
        detected = detect_grid_once(img)
        if detected is None:
            stable_results = []
        elif (len(stable_results) == 0
              or same_calibration_cluster(stable_results[0], detected)):
            stable_results.append(detected)
            if len(stable_results) >= CALIB_CONSENSUS_N:
                xs = [item[0] for item in stable_results]
                ys = [item[1] for item in stable_results]
                dxs = [item[2] for item in stable_results]
                dys = [item[3] for item in stable_results]
                return (median(xs), median(ys), median(dxs), median(dys))
        else:
            stable_results = [detected]
        time.sleep_ms(CALIB_INTERVAL_MS)

def grid_point(r, c, x0, y0, dx, dy):
    return int(x0 + c * dx), int(y0 + r * dy)

# 核心判定逻辑：严格检查是否在框内
def in_threshold(l, a, b, th):
    return (th[0] <= l <= th[1]) and (th[2] <= a <= th[3]) and (th[4] <= b <= th[5])

def get_cell_lab(img, cx, cy):
    cx = max(1, min(img.width() - 2, cx))
    cy = max(1, min(img.height() - 2, cy))
    stat = img.get_statistics(roi=(cx - 1, cy - 1, 3, 3))
    return stat.l_mean(), stat.a_mean(), stat.b_mean()

def classify_lab(l, a, b, allow_wall=True):
    # 优先级判定：符合谁的范围，就是谁。
    # 目的地、箱子等特征最明显，放前面。空地作为托底放最后。
    if in_threshold(l, a, b, LAB_THRESHOLDS["."]): return "."
    if in_threshold(l, a, b, LAB_THRESHOLDS["$"]): return "$"
    if in_threshold(l, a, b, LAB_THRESHOLDS["*"]): return "*"
    if allow_wall and in_threshold(l, a, b, LAB_THRESHOLDS["#"]): return "#"
    if in_threshold(l, a, b, LAB_THRESHOLDS["-"]): return "-"

    # 终极防线：如果暗角严重到六亲不认，强制归为空地，防止寻路死锁
    return "-"

def classify_cell_lab(img, cx, cy, allow_wall=False):
    center_lab = get_cell_lab(img, cx, cy)
    return classify_lab(
        center_lab[0], center_lab[1], center_lab[2], allow_wall
    )

def process_once(img, wall_mask=None, xy_car_cell=None,
                 previous_map_rows=None):
    map_rows = []
    observed_rows = []
    valid_wall_mask = is_valid_matrix(wall_mask)
    valid_previous = is_valid_matrix(previous_map_rows)

    for r in range(ROWS):
        row_chars = []
        observed_chars = []
        for c in range(COLS):
            x, y = grid_point(r, c, x0, y0, dx, dy)

            if r == 0 or r == ROWS - 1 or c == 0 or c == COLS - 1:
                observed_tag = "#"
                tag = "#"
            else:
                # observed_tag 只描述当前画面，供清屏/新关卡判断使用。
                observed_tag = classify_cell_lab(img, x, y, allow_wall=True)

                # 正式字符地图只承认本关预先建立的墙掩码。
                # 后续普通元素识别阶段禁止新增墙，避免车身、箱子和反光变成墙。
                if valid_wall_mask and wall_mask[r][c]:
                    tag = "#"
                elif observed_tag == "#":
                    tag = classify_cell_lab(img, x, y, allow_wall=False)
                else:
                    tag = observed_tag

                # 小车不写入字符地图，也不能改变其所在格已有的底层内容。
                if (xy_car_cell is not None
                    and r == xy_car_cell[0] and c == xy_car_cell[1]):
                    if valid_previous:
                        tag = previous_map_rows[r][c]
                    elif not (valid_wall_mask and wall_mask[r][c]):
                        tag = "-"

            row_chars.append(tag)
            observed_chars.append(observed_tag)

        map_rows.append(row_chars)
        observed_rows.append(observed_chars)

    return map_rows, observed_rows

def copy_map_rows(map_rows):
    return [row[:] for row in map_rows]

def is_valid_matrix(matrix):
    if matrix is None or len(matrix) != ROWS:
        return False
    for row in matrix:
        if row is None or len(row) != COLS:
            return False
    return True

def round1_times10(value):
    if value >= 0:
        return int(value * 10 + 0.5)
    return int(value * 10 - 0.5)

def coord_string_from_grid(x_grid, y_grid):
    return "X{}Y{}".format(round1_times10(x_grid), round1_times10(y_grid))

def correct_wall_interference(x_grid, y_grid, wall_cells):
    # 仅修正小车与墙体的真实几何重叠。小车边长按 0.98 格估算，
    # 单次最多推回 0.1 格，避免理论墙中心误差把正常坐标强行推远。
    CAR_HALF = 0.49
    CELL_HALF = 0.5
    MIN_SEPARATION = CAR_HALF + CELL_HALF
    MAX_CORRECTION = 0.1
    best_axis = None
    best_direction = 0
    best_overlap = None

    for wall_r, wall_c in wall_cells:
        wall_x = wall_c
        wall_y = (ROWS - 1) - wall_r
        dx_grid = x_grid - wall_x
        dy_grid = y_grid - wall_y

        if abs(dx_grid) >= MIN_SEPARATION or abs(dy_grid) >= MIN_SEPARATION:
            continue

        overlap_x = MIN_SEPARATION - abs(dx_grid)
        overlap_y = MIN_SEPARATION - abs(dy_grid)

        # 只选择最小、方向明确的一次推出；拐角处不连续叠加多次修正。
        if dx_grid != 0 and (dy_grid == 0 or overlap_x <= overlap_y):
            axis = "x"
            direction = 1 if dx_grid > 0 else -1
            overlap = overlap_x
        elif dy_grid != 0:
            axis = "y"
            direction = 1 if dy_grid > 0 else -1
            overlap = overlap_y
        else:
            # 与墙中心完全重合时没有可靠方向，宁可不猜。
            continue

        if best_overlap is None or overlap < best_overlap:
            best_axis = axis
            best_direction = direction
            best_overlap = overlap

    if best_axis is None:
        return x_grid, y_grid, False

    correction = min(best_overlap, MAX_CORRECTION)
    if best_axis == "x":
        x_grid += best_direction * correction
    else:
        y_grid += best_direction * correction
    return x_grid, y_grid, True

def collect_wall_cells(wall_mask):
    walls = []
    if not is_valid_matrix(wall_mask):
        return walls
    for r in range(ROWS):
        for c in range(COLS):
            if wall_mask[r][c]:
                walls.append((r, c))
    return walls

def get_serial_coords(car, wall_cells=None):
    # LOST 是正常的短暂状态：没有任何成功历史时不进行坐标后处理。
    if len(car.history_x) == 0 or len(car.history_y) == 0:
        return car.get_coords()

    x_grid = car.rel_x / 10.0
    y_grid = car.rel_y / 10.0

    if wall_cells:
        fixed_x, fixed_y, corrected = correct_wall_interference(
            x_grid, y_grid, wall_cells
        )
        if corrected:
            return coord_string_from_grid(fixed_x, fixed_y)

    return coord_string_from_grid(x_grid, y_grid)

def wall_cell_5region(img, cx, cy):
    offset_x = max(2, int(dx * 0.18 + 0.5))
    offset_y = max(2, int(dy * 0.18 + 0.5))
    votes = 0

    for sx, sy in (
        (cx, cy),
        (cx - offset_x, cy),
        (cx + offset_x, cy),
        (cx, cy - offset_y),
        (cx, cy + offset_y)
    ):
        sample = get_cell_lab(img, sx, sy)

        # 自动曝光下命中任一小车阈值的采样点明确按“非墙”处理，
        # 避免开局车身被写入初始墙体许可表。
        if (in_threshold(
                sample[0], sample[1], sample[2],
                my_threshold
            )
            or in_threshold(
                sample[0], sample[1], sample[2],
                my_threshold_fallback
            )):
            continue

        if is_wall_lab(sample[0], sample[1], sample[2]):
            votes += 1

    return votes >= 3

def capture_wall_mask():
    # 自动曝光稳定后采 3 张。每个“点”实际是一个 3x3 LAB 均值区域，
    # 不是单像素；任意一帧达到 3/5 区域支持即记为墙。
    wall_allowed = []
    for r in range(ROWS):
        row = []
        for c in range(COLS):
            row.append(
                r == 0 or r == ROWS - 1 or c == 0 or c == COLS - 1
            )
        wall_allowed.append(row)

    for sample_index in range(3):
        img = sensor.snapshot()
        for r in range(1, ROWS - 1):
            for c in range(1, COLS - 1):
                if wall_allowed[r][c]:
                    continue
                x, y = grid_point(r, c, x0, y0, dx, dy)
                if wall_cell_5region(img, x, y):
                    wall_allowed[r][c] = True
        if sample_index < 2:
            time.sleep_ms(80)

    return wall_allowed

def is_cleared_level_map(map_rows, ignored_cell=None):
    if not is_valid_matrix(map_rows):
        return False

    non_empty_count = 0
    empty_count = 0
    total_count = 0

    for r in range(1, ROWS - 1):
        for c in range(1, COLS - 1):
            if ignored_cell is not None and (r, c) == ignored_cell:
                empty_count += 1
                total_count += 1
                continue
            tag = map_rows[r][c]
            total_count += 1
            if tag == "-":
                empty_count += 1
            else:
                non_empty_count += 1

    # 必须严格全为空地；小车不写入字符地图。
    return non_empty_count == 0 and empty_count == total_count

def has_inner_level_content(map_rows, ignored_cell=None):
    if not is_valid_matrix(map_rows):
        return False
    for r in range(1, ROWS - 1):
        for c in range(1, COLS - 1):
            if ignored_cell is not None and (r, c) == ignored_cell:
                continue
            if map_rows[r][c] != "-":
                return True
    return False

# ================= 4. 主循环与串口通信逻辑 =================
x0, y0, dx, dy = calibrate_grid()

# 启动时先在正常自动曝光下建立墙体种子，随后直接进入日常识别。
current_wall_mask = capture_wall_mask()

# 这里保存的是“最外圈墙的内边缘”，也就是墙与内部地图区域的分界线。
# 不是最外侧墙体的外轮廓。
inner_left = int(x0 + 0.5 * dx)
inner_right = int(x0 + (COLS - 1.5) * dx)
inner_top = int(y0 + 0.5 * dy)
inner_bottom = int(y0 + (ROWS - 1.5) * dy)

# 标定完成后再创建追踪器，让它与本次地图使用同一个原点
car = CarTracker(x0=x0, y0=y0, dx=dx, dy=dy, cols=COLS, rows=ROWS,
                 threshold=my_threshold,
                 fallback_threshold=my_threshold_fallback,
                 box_threshold=LAB_THRESHOLDS["$"],
                 goal_threshold=LAB_THRESHOLDS["."])

precalc_grid_points = []
for r in range(ROWS):
    for c in range(COLS):
        precalc_grid_points.append(grid_point(r, c, x0, y0, dx, dy))

last_map_str = ""
frame_counter = 0

current_map_rows = []

# ================= 4.1 简化后的实时主循环 =================
# 第一关先建立墙体种子。之后不切曝光、不维护墙体/动态数量锁，
# 每次地图识别都直接采用当前结果。后续关卡出现时同样先提取一次墙体种子。
last_complete_map_rows = None
last_good_serial_coord = None
current_wall_cells = collect_wall_cells(current_wall_mask)
waiting_for_new_level = False
new_level_seen_count = 0
clear_map_streak = 0
level_has_content = False

MAP_UPDATE_INTERVAL = 30
NEW_LEVEL_SCAN_INTERVAL = 3
NEW_LEVEL_CONFIRM_N = 2
SERIAL_SETTLE_MS = 200
SERIAL_TRACK_SAMPLES = 3

def refresh_tracker_for_serial():
    # 队友到位后立即发问；这里连续抓 3 帧，让最近 3 次成功中心覆盖移动期历史。
    # 若处理不足 200 ms，补足等待时间；超过则不额外延迟。
    start_ms = time.ticks_ms()
    success_count = 0
    for _ in range(SERIAL_TRACK_SAMPLES):
        sample_img = sensor.snapshot()
        if car.update(sample_img):
            success_count += 1

    elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
    remaining_ms = SERIAL_SETTLE_MS - elapsed_ms
    if remaining_ms > 0:
        time.sleep_ms(remaining_ms)
    return success_count

while True:
    clock.tick()

    # 收到 1/2 后先补抓 3 帧，并在 200 ms 窗口结束时返回三次成功中心平均。
    if uart.any():
        recv_data = uart.read()

        if b'1' in recv_data or 0x01 in recv_data or 1 in recv_data:
            fresh_successes = refresh_tracker_for_serial()
            latest_map = current_map_rows
            if not is_valid_matrix(latest_map):
                latest_map = last_complete_map_rows

            if is_valid_matrix(latest_map):
                final_flat_map = "".join(
                    ["".join(row) for row in latest_map]
                )
                uart.write(bytearray([0xAA]))
                uart.write(final_flat_map)
                uart.write("\r\n")
                uart.write(bytearray([0xFF]))

            if fresh_successes > 0:
                result = get_serial_coords(car, current_wall_cells)
                if result != "LOST":
                    last_good_serial_coord = result
            else:
                # 本次请求没有任何新鲜成功帧时不再冒用历史坐标。
                # 宁可明确 LOST，让队友跳过这次校准。
                result = "LOST"
            uart.write(result + "\n")

        elif b'2' in recv_data or 0x02 in recv_data or 2 in recv_data:
            fresh_successes = refresh_tracker_for_serial()
            if fresh_successes > 0:
                result = get_serial_coords(car, current_wall_cells)
                if result != "LOST":
                    last_good_serial_coord = result
            else:
                result = "LOST"
            uart.write(result + "\n")

    img = sensor.snapshot()
    frame_counter += 1

    # Tracker 每帧独立更新；字符地图不参与小车坐标覆盖。
    car.update(img)
    if car.found:
        final_coord = get_serial_coords(car, current_wall_cells)
        if final_coord != "LOST":
            last_good_serial_coord = final_coord
    xy_car_cell = car.get_grid_cell()

    if waiting_for_new_level:
        map_update_due = (frame_counter % NEW_LEVEL_SCAN_INTERVAL == 0)
    else:
        map_update_due = (
            frame_counter == 1
            or frame_counter % MAP_UPDATE_INTERVAL == 0
        )

    if map_update_due:
        raw_map_rows, observed_map_rows = process_once(
            img, current_wall_mask, xy_car_cell, current_map_rows
        )

        if waiting_for_new_level:
            # 等待期间只观察当前画面，不把旧墙或过渡帧写入正式地图。
            if has_inner_level_content(observed_map_rows, xy_car_cell):
                new_level_seen_count += 1
            else:
                new_level_seen_count = 0

            if new_level_seen_count >= NEW_LEVEL_CONFIRM_N:
                # 新关卡稳定出现后先独立建墙，再识别其他元素。
                # 普通元素阶段只使用这份墙掩码，之后不能新增墙。
                current_wall_mask = capture_wall_mask()
                current_wall_cells = collect_wall_cells(current_wall_mask)
                level_img = sensor.snapshot()
                current_map_rows, _ = process_once(
                    level_img, current_wall_mask, None, None
                )
                waiting_for_new_level = False
                new_level_seen_count = 0
                clear_map_streak = 0
                level_has_content = True
                car.reset_history()
                frame_counter = 0
        else:
            current_map_rows = copy_map_rows(raw_map_rows)

            if (level_has_content
                and is_cleared_level_map(observed_map_rows, xy_car_cell)):
                clear_map_streak += 1
            else:
                clear_map_streak = 0

            if clear_map_streak >= 2:
                waiting_for_new_level = True
                new_level_seen_count = 0
                level_has_content = False
                clear_map_streak = 0
            elif has_inner_level_content(observed_map_rows, xy_car_cell):
                level_has_content = True

        if is_valid_matrix(current_map_rows):
            last_complete_map_rows = copy_map_rows(current_map_rows)

        map_str_with_newlines = "\n".join(
            ["".join(row) for row in current_map_rows]
        )

        if map_str_with_newlines != last_map_str:
            print("\n===== 当前识别的地图 =====")
            print(map_str_with_newlines)
            if car.found:
                final_coord = get_serial_coords(car, current_wall_cells)
                if final_coord != "LOST":
                    last_good_serial_coord = final_coord
            else:
                final_coord = last_good_serial_coord or "LOST"
            print("CAR:", final_coord)
            last_map_str = map_str_with_newlines

    # 所有识别完成后再画辅助图形，避免污染采样。
    car.draw_overlay(img)

    for pt in precalc_grid_points:
        img.draw_cross(pt[0], pt[1], color=(255, 255, 255), size=2)

    img.draw_line(inner_left, inner_top, inner_right, inner_top,
                  color=(255, 0, 0), thickness=1)
    img.draw_line(inner_left, inner_bottom, inner_right, inner_bottom,
                  color=(255, 0, 0), thickness=1)
    img.draw_line(inner_left, inner_top, inner_left, inner_bottom,
                  color=(255, 0, 0), thickness=1)
    img.draw_line(inner_right, inner_top, inner_right, inner_bottom,
                  color=(255, 0, 0), thickness=1)
