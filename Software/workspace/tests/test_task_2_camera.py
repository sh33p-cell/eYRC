"""Regression checks using initial frames from the supplied MuJoCo arenas."""

from pathlib import Path
import unittest

import cv2
import numpy as np

from test_task_2 import load


class TaskTwoCameraTests(unittest.TestCase):
    def test_captured_arenas_have_reachable_starts_and_goals(self):
        for task in ('task_2b', 'task_2c'):
            with self.subTest(task=task):
                p = load(task, 'path_planner.py')
                frame = cv2.imread(str(Path(__file__).parent / 'fixtures' /
                                       (task.replace('_', '') + '_start.jpg')))
                self.assertIsNotNone(frame)
                starts = dict(zip(('r1', 'r2', 'r3'), p.START_CIRCLES))
                starts = {r: starts[r] for r in p.ROBOTS}
                goals = p.GOALS if task == 'task_2b' else list(p.GOALS.values())
                occupied = p.frame_to_occupancy(frame)
                cleaned = p.remove_drivable(frame, occupied, list(starts.values()), goals)
                blocked = p.planning_grid(cleaned)
                for point in [*starts.values(), *goals]:
                    self.assertFalse(blocked[p.to_cell(point)], str(point))
                if task == 'task_2b':
                    plan = p.plan_route(blocked, starts['r2'])
                    self.assertIsNotNone(plan)
                    plans = {'r2': plan}
                else:
                    plans = p.plan_all(blocked, starts)
                    self.assertIsNotNone(plans)
                    self.assertEqual(set(plans), set(p.ROBOTS))
                for robot, plan in plans.items():
                    self.assertEqual(plan[0][1:], starts[robot])
                    for a, b in zip(plan, plan[1:]):
                        self.assertTrue(p.segment_clear(blocked, a[1:], b[1:]))

    def test_start_cleanup_does_not_clip_a_nearby_pillar(self):
        for task in ('task_2b', 'task_2c'):
            with self.subTest(task=task):
                p = load(task, 'path_planner.py')
                hsv = np.full((720, 1280, 3), (20, 80, 220), dtype=np.uint8)
                robot = p.START_CIRCLES[1]
                pillar = (robot[0] + .12, robot[1])
                pixel = (round(p.ARENA_LEFT + pillar[0] * p.ARENA_PIXELS / p.ARENA_SIZE),
                         round(p.ARENA_TOP + pillar[1] * p.ARENA_PIXELS / p.ARENA_SIZE))
                cv2.circle(hsv, pixel, 12, (0, 0, 130), -1)
                frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                occupied = p.frame_to_occupancy(frame)
                cleaned = p.remove_drivable(frame, occupied, [robot], [])
                # The pillar overlaps the start-clearing disc. Preserve even
                # its inner edge, not just the part outside that disc.
                edge = (robot[0] + .095, robot[1])
                cell = (int(edge[1] / p.CELL_SIZE), int(edge[0] / p.CELL_SIZE))
                self.assertTrue(occupied[cell])
                self.assertTrue(cleaned[cell])
                self.assertTrue(p.planning_grid(cleaned)[p.to_cell(robot)])


if __name__ == '__main__':
    unittest.main()
