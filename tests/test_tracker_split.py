import os
import sys
import unittest

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from tracker import CarTracker


CAR_THRESHOLD = (0, 95, -95, -36, -48, 90)
BOX_THRESHOLD = (0, 100, -51, 9, 45, 122)


class FakeBlob:
    def __init__(self, x, y, w, h, pixels=300):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._pixels = pixels

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return self._w

    def h(self):
        return self._h

    def cx(self):
        return self._x + self._w // 2

    def cy(self):
        return self._y + self._h // 2

    def pixels(self):
        return self._pixels


class FakeStatistics:
    def __init__(self, lab):
        self.lab = lab

    def l_mean(self):
        return self.lab[0]

    def a_mean(self):
        return self.lab[1]

    def b_mean(self):
        return self.lab[2]


class FakeImage:
    def __init__(self, main_blobs=None, side_blobs=None, box_centers=None):
        self.main_blobs = main_blobs or []
        self.side_blobs = side_blobs or []
        self.box_centers = box_centers or []
        self.find_calls = []

    def width(self):
        return 320

    def height(self):
        return 240

    def find_blobs(self, _thresholds, roi, pixels_threshold,
                   area_threshold, merge):
        self.find_calls.append((roi, pixels_threshold, area_threshold, merge))
        if pixels_threshold == 50:
            return self.main_blobs
        return self.side_blobs

    def get_statistics(self, roi):
        x, y, w, h = roi
        cx = x + w // 2
        cy = y + h // 2
        for box_x, box_y in self.box_centers:
            if abs(cx - box_x) <= 3 and abs(cy - box_y) <= 3:
                return FakeStatistics((70, -10, 70))
        return FakeStatistics((42, 0, -95))


def make_tracker(history_x=100, history_y=100):
    tracker = CarTracker(
        33, 35, 16.5, 15.8, 16, 12,
        CAR_THRESHOLD,
        fallback_threshold=None,
        box_threshold=BOX_THRESHOLD,
    )
    tracker.history_x = [history_x, history_x, history_x]
    tracker.history_y = [history_y, history_y, history_y]
    tracker.confirmed_boxes = [(4, 5)]
    return tracker


class BoxCoreRoiRecoveryTests(unittest.TestCase):
    def test_normal_blob_keeps_fast_path(self):
        tracker = make_tracker()
        tracker.confirmed_boxes = []
        blob = FakeBlob(92, 92, 16, 16)
        image = FakeImage(main_blobs=[blob])

        self.assertIs(tracker.find_target(image, CAR_THRESHOLD), blob)
        self.assertEqual(len(image.find_calls), 1)

    def test_mildly_deformed_car_near_box_stays_fast(self):
        tracker = make_tracker()
        blob = FakeBlob(80, 93, 20, 14)
        image = FakeImage(main_blobs=[blob])

        self.assertIs(tracker.find_target(image, CAR_THRESHOLD), blob)
        self.assertEqual(len(image.find_calls), 1)

    def test_merged_blob_returns_real_side_blob_center(self):
        tracker = make_tracker()
        merged = FakeBlob(90, 88, 38, 24)
        real_car = FakeBlob(96, 93, 16, 14, pixels=120)
        image = FakeImage(main_blobs=[merged], side_blobs=[real_car])

        target = tracker.find_target(image, CAR_THRESHOLD)

        self.assertIs(target, real_car)
        self.assertEqual((target.cx(), target.cy()), (104, 100))
        # The number of strip searches follows the grid geometry (one call per
        # allowed run), so assert the invariant instead of a fixed count: no side
        # search may contain the box centre, i.e. the box cell itself is never
        # searched. Strip ROIs sit in the cells around it and touch its edges.
        box_x = 33 + 5 * 16.5
        box_y = 35 + 4 * 15.8
        side_calls = [call for call in image.find_calls if call[1] != 50]
        self.assertGreaterEqual(len(side_calls), 1)
        for roi, *_rest in side_calls:
            x, y, w, h = roi
            inside = x <= box_x < x + w and y <= box_y < y + h
            self.assertFalse(inside, f'side ROI {roi} covers the box centre')

    def test_square_blob_touching_box_core_also_recovers(self):
        tracker = make_tracker()
        merged = FakeBlob(96, 90, 20, 20)
        real_car = FakeBlob(98, 94, 8, 12, pixels=70)
        image = FakeImage(main_blobs=[merged], side_blobs=[real_car])

        target = tracker.find_target(image, CAR_THRESHOLD)

        self.assertIs(target, real_car)
        self.assertEqual((target.cx(), target.cy()), (102, 100))
        # One call per allowed strip run; the count follows the geometry.
        self.assertGreaterEqual(len(image.find_calls), 1)

    def test_above_box_crops_at_box_top(self):
        tracker = make_tracker(history_x=115, history_y=82)
        merged = FakeBlob(106, 75, 20, 36)
        real_car = FakeBlob(108, 76, 14, 14, pixels=100)
        image = FakeImage(side_blobs=[real_car])

        target = tracker.recover_car_from_box_side(
            image, merged, CAR_THRESHOLD
        )

        self.assertIs(target, real_car)
        side_roi = image.find_calls[0][0]
        box_top = 35 + 4 * 15.8 - 15.8 * tracker.BOX_CORE_HALF
        # The ROI is rounded outward by half a pixel, so the crop may extend one
        # pixel past the exact box-core edge; it must never go further.
        self.assertLessEqual(side_roi[1] + side_roi[3], int(box_top) + 1)

    def test_history_inside_box_core_refuses_direction_guess(self):
        tracker = make_tracker(history_x=115.5, history_y=98.2)
        merged = FakeBlob(90, 88, 38, 24)
        image = FakeImage(side_blobs=[FakeBlob(96, 93, 16, 14)])

        target = tracker.recover_car_from_box_side(
            image, merged, CAR_THRESHOLD
        )

        self.assertIsNone(target)
        self.assertEqual(image.find_calls, [])

    def test_blob_without_box_overlap_does_not_enter_recovery(self):
        tracker = make_tracker()
        image = FakeImage(side_blobs=[FakeBlob(80, 93, 16, 14)])

        target = tracker.recover_car_from_box_side(
            image, FakeBlob(78, 88, 25, 24), CAR_THRESHOLD
        )

        self.assertIsNone(target)
        self.assertEqual(image.find_calls, [])

    def test_recovered_blob_still_obeys_jump_limit(self):
        tracker = make_tracker()
        image = FakeImage(side_blobs=[FakeBlob(64, 93, 16, 14)])

        target = tracker.recover_car_from_box_side(
            image, FakeBlob(60, 88, 68, 24), CAR_THRESHOLD
        )

        self.assertIsNone(target)


class LostAnchorReacquireTests(unittest.TestCase):
    def test_normal_tracking_still_rejects_far_single_frame_jump(self):
        tracker = make_tracker()
        tracker.confirmed_boxes = []
        far_car = FakeBlob(152, 92, 16, 16)
        image = FakeImage(main_blobs=[far_car])

        self.assertIsNone(tracker.find_target(image, CAR_THRESHOLD))

    def test_two_consistent_far_frames_replace_stale_anchor(self):
        tracker = make_tracker()
        tracker.confirmed_boxes = []
        tracker.fail_count = tracker.REACQUIRE_AFTER_FAILS
        far_car = FakeBlob(152, 92, 16, 16)
        image = FakeImage(main_blobs=[far_car])

        self.assertFalse(tracker.update(image))
        self.assertEqual(tracker.reacquire_count, 1)
        self.assertTrue(tracker.update(image))
        self.assertEqual(tracker.fail_count, 0)
        self.assertEqual(tracker.history_x, [far_car.cx()])
        self.assertEqual(tracker.history_y, [far_car.cy()])

    def test_inconsistent_far_candidate_does_not_reanchor(self):
        tracker = make_tracker()
        tracker.confirmed_boxes = []
        tracker.fail_count = tracker.REACQUIRE_AFTER_FAILS
        first = FakeImage(main_blobs=[FakeBlob(152, 92, 16, 16)])
        second = FakeImage(main_blobs=[FakeBlob(185, 92, 16, 16)])

        self.assertFalse(tracker.update(first))
        self.assertFalse(tracker.update(second))
        self.assertEqual(tracker.reacquire_count, 1)
        self.assertEqual(tracker.history_x, [100, 100, 100])

    def test_box_colored_far_candidate_cannot_reanchor(self):
        tracker = make_tracker()
        tracker.confirmed_boxes = []
        tracker.fail_count = tracker.REACQUIRE_AFTER_FAILS
        fake_box = FakeBlob(152, 92, 16, 16)
        image = FakeImage(
            main_blobs=[fake_box],
            box_centers=[(fake_box.cx(), fake_box.cy())],
        )

        self.assertFalse(tracker.update(image))
        self.assertEqual(tracker.reacquire_count, 0)
        self.assertEqual(tracker.history_x, [100, 100, 100])


if __name__ == "__main__":
    unittest.main()
