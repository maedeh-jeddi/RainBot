#!/usr/bin/env python3
"""Check a mapping tour against every obstacle footprint in hospital_lab.sdf.

The first tour was checked only AT its waypoints, which missed that the
straight line between two clear waypoints can still run through furniture --
the run aborted 0.53 m from ward_scrubs on exactly such a segment. This checks
the segments themselves.

Boxes are axis-aligned world-frame footprints, taken from the same measured
mesh extents the world file was laid out with (rotated models have their
extents swapped here). The two pedestrians are approximated by a square rather
than their mesh, deliberately oversized.

Clearance required:
  DRIVE_CLEAR at any point along a segment  - the Husky's half width is
      0.335 m, so this is that plus margin.
  TURN_CLEAR in a disc around every waypoint - the robot pivots on the spot
      there, sweeping its circumscribed radius sqrt(0.495^2+0.335^2)=0.598 m.
"""
import math
import sys

DRIVE_CLEAR = 0.45
TURN_CLEAR = 0.62

BOXES = {
    # --- lab props ---
    'lab_bench_collect':   (-6.03, 2.72, -3.97, 3.78),
    'lab_worktable':       (-6.70, -3.80, -4.30, -2.80),
    'lab_cabinet_0':       (-8.875, 2.195, -8.425, 3.005),
    'lab_cabinet_1':       (-8.875, 1.195, -8.425, 2.005),
    'lab_storage':         (-8.855, -2.13, -8.145, -0.87),
    'lab_instrument_cart': (-2.305, 2.54, -1.695, 3.06),
    'lab_trolley':         (-2.64, -2.685, -1.76, -2.315),
    'lab_technician':      (-4.215, 1.715, -3.785, 2.285),
    'lab_scrubs':          (-7.66, 2.22, -7.34, 2.78),
    # --- corridor props ---
    'corridor_trolley':    (-0.235, -1.86, 0.835, -1.34),
    'corridor_ivstand':    (1.98, 1.475, 2.42, 1.925),
    'corridor_parking':    (4.55, -1.85, 5.45, -1.25),
    # --- ward props ---
    'lab_bench_deliver':   (13.85, -1.03, 14.88, 1.03),
    'ward_nurse_desk':     (10.59, 2.23, 15.41, 3.77),
    'ward_bed_0':          (10.43, -3.445, 12.57, -2.355),
    'ward_bed_1':          (13.81, -3.325, 15.79, -2.475),
    'ward_bedside_table':  (12.65, -2.49, 13.15, -1.91),
    'ward_bp_monitor':     (16.215, 0.93, 16.785, 1.47),
    'ward_wheelchair':     (9.64, 0.98, 10.76, 1.62),
    'ward_chair':          (16.05, -1.905, 16.75, -1.095),
    'ward_sofa':           (16.04, 1.96, 16.76, 2.64),
    'ward_garbage':        (15.865, -3.865, 16.835, -2.735),
    'ward_visitor_0':      (16.05, -3.005, 16.75, -2.195),
    'ward_visitor_1':      (16.05, -0.405, 16.75, 0.405),
    'ward_nurse':          (9.715, -1.415, 10.285, -0.985),
    'ward_scrubs':         (12.34, 1.22, 12.66, 1.78),
    # --- walls ---
    'wall_lab_west':       (-9.075, -4.075, -8.925, 4.075),
    'wall_lab_north':      (-9.075, 3.925, -0.925, 4.075),
    'wall_lab_south':      (-9.075, -4.075, -0.925, -3.925),
    'wall_lab_east_n':     (-1.075, 1.00, -0.925, 4.00),
    'wall_lab_east_s':     (-1.075, -4.00, -0.925, -1.00),
    'wall_corridor_n':     (-1.00, 1.925, 9.00, 2.075),
    'wall_corridor_s':     (-1.00, -2.075, 9.00, -1.925),
    'wall_ward_east':      (16.925, -4.075, 17.075, 4.075),
    'wall_ward_north':     (8.925, 3.925, 17.075, 4.075),
    'wall_ward_south':     (8.925, -4.075, 17.075, -3.925),
    'wall_ward_west_n':    (8.925, 1.00, 9.075, 4.00),
    'wall_ward_west_s':    (8.925, -4.00, 9.075, -1.00),
}


def point_box_dist(px, py, box):
    x0, y0, x1, y1 = box
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy)


def segment_box_dist(ax, ay, bx, by, box, steps=200):
    """Sampled minimum distance from segment AB to a box. Sampling is fine
    here: at 200 steps the sample spacing over the longest segment is under
    2 cm, far below the margins being checked."""
    best = float('inf')
    for i in range(steps + 1):
        t = i / steps
        best = min(best, point_box_dist(ax + (bx - ax) * t,
                                        ay + (by - ay) * t, box))
    return best


def check(tour):
    bad = False
    for i, (wx, wy) in enumerate(tour):
        for name, box in BOXES.items():
            d = point_box_dist(wx, wy, box)
            if d < TURN_CLEAR:
                print(f'  TURN  wp{i + 1} ({wx:+.2f},{wy:+.2f}) '
                      f'{d:.2f} m from {name}  (need {TURN_CLEAR})')
                bad = True
    for i in range(len(tour) - 1):
        ax, ay = tour[i]
        bx, by = tour[i + 1]
        for name, box in BOXES.items():
            d = segment_box_dist(ax, ay, bx, by, box)
            if d < DRIVE_CLEAR:
                print(f'  SEG   wp{i + 1}->wp{i + 2} '
                      f'({ax:+.2f},{ay:+.2f})->({bx:+.2f},{by:+.2f}) '
                      f'{d:.2f} m from {name}  (need {DRIVE_CLEAR})')
                bad = True
    return bad


def main():
    from pickplace_arm_bringup.hospital_map_drive import TOUR
    print(f'checking {len(TOUR)} waypoints')
    if check(TOUR):
        print('FAIL')
        return 1
    print('OK: every waypoint and segment clears')
    return 0


if __name__ == '__main__':
    sys.exit(main())
