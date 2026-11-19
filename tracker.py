class CarTracker:
    def __init__(self, x0, y0, dx, dy, cols, rows, threshold,
                 fallback_threshold=None, box_threshold=None,
                 goal_threshold=None):
        self.x0 = x0
        self.y0 = y0
        self.dx = dx
        self.dy = dy
        self.cols = cols
        self.rows = rows
        self.threshold = threshold
        self.fallback_threshold = fallback_threshold
        self.box_threshold = box_threshold
        self.goal_threshold = goal_threshold
        self.confirmed_boxes = []
        self.fail_count = 0
        self.MAX_JUMP_CELLS = 1.3
        self.REACQUIRE_AFTER_FAILS = 5
        self.REACQUIRE_CONFIRM_N = 2
        self.REACQUIRE_MATCH_CELLS = 0.65
        self.reacquire_x = None
        self.reacquire_y = None
        self.reacquire_count = 0
        self.history_x = []
        self.history_y = []
        self.FILTER_N = 3
        self.rel_x = 0
        self.rel_y = 0
        self.center_x = None
        self.center_y = None
        self.found = False

        # 普通 blob 保持快速路径；候选碰到箱子核心时才裁剪后重搜。
        self.BOX_CORE_HALF = 0.46
        self.RECOVERY_MIN_SIDE_CELLS = 0.18

    def reset_history(self):
        # 每一关重新开始时清空上一关的位置滤波和失败状态。
        # 网格标定参数、阈值和 Tracker 配置本身不变。
        self.fail_count = 0
        self.clear_reacquire_candidate()
        self.history_x = []
        self.history_y = []
        self.rel_x = 0
        self.rel_y = 0
        self.center_x = None
        self.center_y = None
        self.found = False
        self.confirmed_boxes = []

    def clear_reacquire_candidate(self):
        self.reacquire_x = None
        self.reacquire_y = None
        self.reacquire_count = 0

    def target_is_far_from_history(self, target):
        if len(self.history_x) == 0 or len(self.history_y) == 0:
            return False
        last_x = sum(self.history_x) / len(self.history_x)
        last_y = sum(self.history_y) / len(self.history_y)
        max_jump = max(self.dx, self.dy) * self.MAX_JUMP_CELLS
        dist2 = ((target.cx() - last_x) ** 2
                 + (target.cy() - last_y) ** 2)
        return dist2 > max_jump ** 2

    def confirm_reacquire_candidate(self, target):
        max_delta = max(self.dx, self.dy) * self.REACQUIRE_MATCH_CELLS
        if (self.reacquire_x is not None and self.reacquire_y is not None
                and (target.cx() - self.reacquire_x) ** 2
                    + (target.cy() - self.reacquire_y) ** 2
                    <= max_delta ** 2):
            self.reacquire_count += 1
            self.reacquire_x = (
                self.reacquire_x * (self.reacquire_count - 1) + target.cx()
            ) / self.reacquire_count
            self.reacquire_y = (
                self.reacquire_y * (self.reacquire_count - 1) + target.cy()
            ) / self.reacquire_count
        else:
            self.reacquire_x = target.cx()
            self.reacquire_y = target.cy()
            self.reacquire_count = 1
        return self.reacquire_count >= self.REACQUIRE_CONFIRM_N

    def target_center_is_box(self, img, target):
        if self.box_threshold is None:
            return False
        cx = max(2, min(img.width() - 3, target.cx()))
        cy = max(2, min(img.height() - 3, target.cy()))
        stat = img.get_statistics(roi=(cx - 2, cy - 2, 5, 5))
        return self.in_threshold(
            stat.l_mean(), stat.a_mean(), stat.b_mean(),
            self.box_threshold
        )

    def is_inside_play_area(self, x, y):
        # 小车中心只可能在最外围墙体内侧，也就是红框内部。
        # 最外墙体及其外面的干扰元素一律不参与小车候选。
        left = self.x0 + 0.5 * self.dx
        right = self.x0 + (self.cols - 1.5) * self.dx
        top = self.y0 + 0.5 * self.dy
        bottom = self.y0 + (self.rows - 1.5) * self.dy
        return left <= x <= right and top <= y <= bottom

    def history_grid_cell(self):
        if len(self.history_x) == 0 or len(self.history_y) == 0:
            return None
        avg_x = sum(self.history_x) / len(self.history_x)
        avg_y = sum(self.history_y) / len(self.history_y)
        c = int((avg_x - self.x0) / self.dx + 0.5)
        r = int((avg_y - self.y0) / self.dy + 0.5)
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return r, c
        return None

    def in_threshold(self, l, a, b, threshold):
        return (threshold[0] <= l <= threshold[1]
                and threshold[2] <= a <= threshold[3]
                and threshold[4] <= b <= threshold[5])

    def is_box_cell(self, img, r, c):
        if self.box_threshold is None:
            return False
        cx = int(self.x0 + c * self.dx)
        cy = int(self.y0 + r * self.dy)
        cx = max(2, min(img.width() - 3, cx))
        cy = max(2, min(img.height() - 3, cy))
        stat = img.get_statistics(roi=(cx - 2, cy - 2, 5, 5))
        return self.in_threshold(
            stat.l_mean(), stat.a_mean(), stat.b_mean(),
            self.box_threshold
        )

    def is_goal_cell(self, img, r, c):
        if self.goal_threshold is None:
            return False
        cx = int(self.x0 + c * self.dx)
        cy = int(self.y0 + r * self.dy)
        cx = max(2, min(img.width() - 3, cx))
        cy = max(2, min(img.height() - 3, cy))
        stat = img.get_statistics(roi=(cx - 2, cy - 2, 5, 5))
        return self.in_threshold(
            stat.l_mean(), stat.a_mean(), stat.b_mean(),
            self.goal_threshold
        )

    def scan_nearby_boxes(self, img):
        # 每帧只检查上一位置周围 3x3 的九个格子，不跑第二次全图 blob。
        center_cell = self.history_grid_cell()
        if center_cell is None:
            return []

        center_r, center_c = center_cell
        boxes = []
        for r in range(max(1, center_r - 1), min(self.rows - 1, center_r + 2)):
            for c in range(max(1, center_c - 1), min(self.cols - 1, center_c + 2)):
                if self.is_box_cell(img, r, c):
                    boxes.append((r, c))
        return boxes

    def round1_times10(self, value):
        # value 是“格”为单位的小数，串口输出 value * 10 后的整数
        if value >= 0:
            return int(value * 10 + 0.5)
        return int(value * 10 - 0.5)

    def banned_box_core(self, blob):
        # 当前帧确认的箱子核心硬排除，不再使用覆盖比例放行粘连大 blob。
        core_half_x = self.dx * self.BOX_CORE_HALF
        core_half_y = self.dy * self.BOX_CORE_HALF
        bx = blob.cx()
        by = blob.cy()

        for r, c in self.confirmed_boxes:
            box_x = self.x0 + c * self.dx
            box_y = self.y0 + r * self.dy
            if (abs(bx - box_x) <= core_half_x
                and abs(by - box_y) <= core_half_y):
                return True

        return False

    def blob_overlaps_box_core(self, blob, r, c):
        box_x = self.x0 + c * self.dx
        box_y = self.y0 + r * self.dy
        half_x = self.dx * self.BOX_CORE_HALF
        half_y = self.dy * self.BOX_CORE_HALF
        left = max(blob.x(), box_x - half_x)
        right = min(blob.x() + blob.w(), box_x + half_x)
        top = max(blob.y(), box_y - half_y)
        bottom = min(blob.y() + blob.h(), box_y + half_y)
        return right > left and bottom > top

    def car_side_search_strips(self, box_r, box_c):
        # 数字宫格：7 8 9 / 4 5 6 / 1 2 3，5 是箱子。
        # 五格半环用 3 条相互重叠的连通条带表示，不逐格搜索。
        # 例如右侧为 8-9、9-6-3、2-3，车跨在 6/9 边界时仍是完整 blob。
        if len(self.history_x) == 0 or len(self.history_y) == 0:
            return []

        last_x = sum(self.history_x) / len(self.history_x)
        last_y = sum(self.history_y) / len(self.history_y)
        box_x = self.x0 + box_c * self.dx
        box_y = self.y0 + box_r * self.dy
        dx_cells = (last_x - box_x) / max(1, self.dx)
        dy_cells = (last_y - box_y) / max(1, self.dy)

        if (abs(dx_cells) < self.RECOVERY_MIN_SIDE_CELLS
            and abs(dy_cells) < self.RECOVERY_MIN_SIDE_CELLS):
            return []

        if abs(dx_cells) >= abs(dy_cells):
            if dx_cells < 0:
                offset_strips = (
                    ((-1, -1), (-1, 0)),
                    ((-1, -1), (0, -1), (1, -1)),
                    ((1, -1), (1, 0)),
                )
            else:
                offset_strips = (
                    ((-1, 0), (-1, 1)),
                    ((-1, 1), (0, 1), (1, 1)),
                    ((1, 0), (1, 1)),
                )
        else:
            if dy_cells < 0:
                offset_strips = (
                    ((-1, -1), (-1, 0), (-1, 1)),
                    ((0, -1), (-1, -1)),
                    ((-1, 1), (0, 1)),
                )
            else:
                offset_strips = (
                    ((1, -1), (1, 0), (1, 1)),
                    ((0, -1), (1, -1)),
                    ((1, 1), (0, 1)),
                )

        strips = []
        for offsets in offset_strips:
            strip = []
            for dr, dc in offsets:
                r = box_r + dr
                c = box_c + dc
                if 1 <= r < self.rows - 1 and 1 <= c < self.cols - 1:
                    strip.append((r, c))
            if len(strip) > 0:
                strips.append(strip)
        return strips

    def strip_search_roi(self, img, cells):
        min_r = min(cell[0] for cell in cells)
        max_r = max(cell[0] for cell in cells)
        min_c = min(cell[1] for cell in cells)
        max_c = max(cell[1] for cell in cells)
        half_w = self.dx * 0.52
        half_h = self.dy * 0.52
        left = max(0, int(self.x0 + min_c * self.dx - half_w))
        right = min(
            img.width(), int(self.x0 + max_c * self.dx + half_w + 0.5)
        )
        top = max(0, int(self.y0 + min_r * self.dy - half_h))
        bottom = min(
            img.height(), int(self.y0 + max_r * self.dy + half_h + 0.5)
        )
        if right - left < 4 or bottom - top < 4:
            return None
        return (left, top, right - left, bottom - top)

    def allowed_strip_runs(self, img, strip):
        # 条带中遇到其他箱子/目的地就断开，防止跨过特殊格合并。
        runs = []
        current = []
        for r, c in strip:
            if self.is_box_cell(img, r, c) or self.is_goal_cell(img, r, c):
                if len(current) > 0:
                    runs.append(current)
                    current = []
            else:
                current.append((r, c))
        if len(current) > 0:
            runs.append(current)
        return runs

    def recover_car_from_box_side(self, img, blob, threshold):
        # 只处理确实覆盖箱子核心的候选；其他异常候选宁可拒绝。
        if len(self.history_x) == 0 or len(self.history_y) == 0:
            return None

        last_x = sum(self.history_x) / len(self.history_x)
        last_y = sum(self.history_y) / len(self.history_y)
        overlapping_boxes = []
        for r, c in self.confirmed_boxes:
            if self.blob_overlaps_box_core(blob, r, c):
                box_x = self.x0 + c * self.dx
                box_y = self.y0 + r * self.dy
                dist2 = (box_x - last_x) ** 2 + (box_y - last_y) ** 2
                overlapping_boxes.append((dist2, r, c))

        if len(overlapping_boxes) == 0:
            return None
        overlapping_boxes.sort()

        box_r = overlapping_boxes[0][1]
        box_c = overlapping_boxes[0][2]
        search_strips = self.car_side_search_strips(box_r, box_c)
        if len(search_strips) == 0:
            return None

        valid = []
        max_jump = max(self.dx, self.dy) * self.MAX_JUMP_CELLS
        for strip in search_strips:
            for run in self.allowed_strip_runs(img, strip):
                roi = self.strip_search_roi(img, run)
                if roi is None:
                    continue
                side_blobs = img.find_blobs(
                    [threshold], roi=roi,
                    pixels_threshold=10, area_threshold=10, merge=True
                )
                for candidate in side_blobs:
                    if not self.is_inside_play_area(candidate.cx(), candidate.cy()):
                        continue
                    if self.banned_box_core(candidate):
                        continue
                    if self.target_center_is_box(img, candidate):
                        continue
                    if not (4 <= candidate.w() <= self.dx * 1.35
                            and 4 <= candidate.h() <= self.dy * 1.35):
                        continue
                    ratio = candidate.w() / candidate.h()
                    if not (0.4 <= ratio <= 2.5):
                        continue
                    dist2 = ((candidate.cx() - last_x) ** 2
                             + (candidate.cy() - last_y) ** 2)
                    if dist2 > max_jump ** 2:
                        continue
                    valid.append(candidate)

        if len(valid) == 0:
            return None
        return min(
            valid,
            key=lambda candidate: (
                (candidate.cx() - last_x) ** 2
                + (candidate.cy() - last_y) ** 2,
                -candidate.pixels()
            )
        )

    def get_confirmed_boxes(self):
        return self.confirmed_boxes[:]

    def get_search_roi(self, img):
        # 平时只在上一帧附近搜索，减少全图 find_blobs 的耗时。
        # 连续丢失 3 帧后自动扩大到完整游戏区，保证能重新捕获。
        play_left = int(self.x0 + 0.3 * self.dx)
        play_right = int(self.x0 + (self.cols - 1.3) * self.dx)
        play_top = int(self.y0 + 0.3 * self.dy)
        play_bottom = int(self.y0 + (self.rows - 1.3) * self.dy)

        play_left = max(0, min(img.width() - 2, play_left))
        play_right = max(play_left + 1, min(img.width() - 1, play_right))
        play_top = max(0, min(img.height() - 2, play_top))
        play_bottom = max(play_top + 1, min(img.height() - 1, play_bottom))

        if (len(self.history_x) == 0 or len(self.history_y) == 0
            or self.fail_count >= 3):
            return (play_left, play_top,
                    play_right - play_left, play_bottom - play_top)

        last_x = sum(self.history_x) / len(self.history_x)
        last_y = sum(self.history_y) / len(self.history_y)
        half_w = max(24, int(self.dx * 2.2))
        half_h = max(24, int(self.dy * 2.2))
        left = max(play_left, int(last_x - half_w))
        right = min(play_right, int(last_x + half_w))
        top = max(play_top, int(last_y - half_h))
        bottom = min(play_bottom, int(last_y + half_h))
        return (left, top, max(1, right - left), max(1, bottom - top))

    def find_target(self, img, threshold, allow_far=False):
        # 寻找色块 (直接使用传入的 img，不再重复拍照)
        roi = self.get_search_roi(img)
        blobs = img.find_blobs(
            [threshold], roi=roi,
            pixels_threshold=50, area_threshold=50, merge=True
        )
        if not blobs:
            return None

        # 回到旧 blob 主线：用连通块自身帮助排除大面积蓝地。
        # 保留较温和的尺寸/比例过滤，避免明显碎片和细长噪声。
        valid = []
        near_box = len(self.confirmed_boxes) > 0
        for b in blobs:
            if not self.is_inside_play_area(b.cx(), b.cy()):
                continue
            if near_box:
                max_w = max(35, int(self.dx * 2.4))
                max_h = max(35, int(self.dy * 2.4))
                if not (7 <= b.w() <= max_w and 7 <= b.h() <= max_h):
                    continue
            elif not (8 <= b.w() <= 35 and 8 <= b.h() <= 35):
                continue
            ratio = b.w() / b.h()
            if allow_far:
                # 旧锚点过期后只允许“一格小车形状”参与重捕，
                # 且候选中心不能呈现箱子 LAB，防止全图重捕时被箱子夺舍。
                if (b.w() > self.dx * 1.5 or b.h() > self.dy * 1.5
                        or not (0.65 <= ratio <= 1.55)
                        or self.target_center_is_box(img, b)):
                    continue
            if near_box:
                one_cell_w = self.dx * 1.35
                one_cell_h = self.dy * 1.35
                normal_shape = (
                    0.6 <= ratio <= 1.6
                    and b.w() <= one_cell_w
                    and b.h() <= one_cell_h
                )
                overlaps_box_core = False
                for box_r, box_c in self.confirmed_boxes:
                    if self.blob_overlaps_box_core(b, box_r, box_c):
                        overlaps_box_core = True
                        break

                if overlaps_box_core:
                    recovered_target = self.recover_car_from_box_side(
                        img, b, threshold
                    )
                    if recovered_target is not None:
                        valid.append(recovered_target)
                    continue
                if not normal_shape:
                    continue
            elif not (0.6 <= ratio <= 1.6):
                continue

            # 当前帧确认箱子的核心区域不参与小车候选。
            if self.banned_box_core(b):
                continue

            # 如果已有稳定历史位置，小车不应在一帧内跳太远。
            # 远处偶发干扰 blob 直接丢弃，宁可 LOST，不输出飞走的坐标。
            if (not allow_far
                    and len(self.history_x) > 0 and len(self.history_y) > 0):
                last_x = sum(self.history_x) / len(self.history_x)
                last_y = sum(self.history_y) / len(self.history_y)
                max_jump = max(self.dx, self.dy) * self.MAX_JUMP_CELLS
                dist2 = (b.cx() - last_x) ** 2 + (b.cy() - last_y) ** 2
                if dist2 > max_jump ** 2:
                    continue

            valid.append(b)

        if not valid:
            return None

        if len(self.history_x) > 0 and len(self.history_y) > 0:
            last_x = sum(self.history_x) / len(self.history_x)
            last_y = sum(self.history_y) / len(self.history_y)
            if allow_far:
                max_jump = max(self.dx, self.dy) * self.MAX_JUMP_CELLS
                near_valid = []
                for candidate in valid:
                    dist2 = ((candidate.cx() - last_x) ** 2
                             + (candidate.cy() - last_y) ** 2)
                    if dist2 <= max_jump ** 2:
                        near_valid.append(candidate)
                if len(near_valid) == 0:
                    return max(valid, key=lambda b: b.pixels())
                valid = near_valid
            # 先选离上一位置最近的；距离相同时再选像素更多的。
            return min(
                valid,
                key=lambda b: (
                    (b.cx() - last_x) ** 2 + (b.cy() - last_y) ** 2,
                    -b.pixels()
                )
            )

        return max(valid, key=lambda b: b.pixels())

    def update(self, img):
        self.confirmed_boxes = self.scan_nearby_boxes(img)
        allow_far = self.fail_count >= self.REACQUIRE_AFTER_FAILS
        target = self.find_target(img, self.threshold, allow_far=allow_far)

        # 主阈值失败时立即尝试暗区阈值。箱子核心仍由 banned_box_core 排除，
        # 不再因为“靠近箱子”而禁用备用阈值，否则暗区贴箱会直接 LOST。
        if (target is None
            and self.fallback_threshold is not None):
            target = self.find_target(
                img, self.fallback_threshold, allow_far=allow_far
            )

        self.found = False
        if target is not None:
            if allow_far and self.target_is_far_from_history(target):
                if not self.confirm_reacquire_candidate(target):
                    self.fail_count += 1
                    return False
                # 连续两帧在同一新位置看到真车，旧锚点正式作废。
                self.history_x = []
                self.history_y = []
                self.center_x = None
                self.center_y = None
            self.clear_reacquire_candidate()
            self.fail_count = 0

            # 滤波
            self.history_x.append(target.cx())
            self.history_y.append(target.cy())
            if len(self.history_x) > self.FILTER_N:
                self.history_x.pop(0)
                self.history_y.pop(0)

            avg_x = sum(self.history_x) / len(self.history_x)
            avg_y = sum(self.history_y) / len(self.history_y)
            self.center_x = avg_x
            self.center_y = avg_y

            # 显示、搜索和串口统一使用最近 3 次成功中心平均。
            # 串口请求会主动补抓 3 帧，因此停稳后不再继承移动期单帧偏差。
            x_grid = (avg_x - self.x0) / self.dx
            bottom_y = self.y0 + (self.rows - 1) * self.dy
            y_grid = (bottom_y - avg_y) / self.dy
            self.rel_x = self.round1_times10(x_grid)
            self.rel_y = self.round1_times10(y_grid)
            self.found = True

        else:
            self.clear_reacquire_candidate()
            self.fail_count += 1
        return self.found

    def draw_overlay(self, img):
        # 必须在所有识别完成后调用，避免红框/绿十字污染地图采样。
        if not self.found or self.center_x is None or self.center_y is None:
            return

        box_w = max(10, int(self.dx))
        box_h = max(10, int(self.dy))
        img.draw_rectangle(
            int(self.center_x - box_w / 2),
            int(self.center_y - box_h / 2),
            box_w,
            box_h,
            color=(255, 0, 0)
        )

    def get_coords(self):
        if self.found:
            return "X{}Y{}".format(self.rel_x, self.rel_y)
        return "LOST"

    def get_pixel_center(self):
        if self.found:
            return self.center_x, self.center_y
        return None

    def get_grid_cell(self):
        # 只把现有 XY 结果换算成字符地图下标，不改变 XY 的计算和串口输出。
        if not self.found:
            return None

        c = int(self.rel_x / 10.0 + 0.5)
        y_from_bottom = int(self.rel_y / 10.0 + 0.5)
        r = (self.rows - 1) - y_from_bottom

        if 0 <= r < self.rows and 0 <= c < self.cols:
            return r, c
        return None
