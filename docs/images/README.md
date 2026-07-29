# Images used by the top-level README

Empty placeholder files with these exact names already exist in this directory.
Overwrite them with the real captures and the top-level README will pick them
up as-is — no README edits needed.

| File | Type | What it should show |
| --- | --- | --- |
| `banner.gif` | animated GIF | The headline shot. A short loop of one full pick-and-place: the base driving up to the table, the arm descending and grasping a cube, then placing it on its column. Keep it a few seconds and reasonably small (a few MB) — GitHub will not lazy-load it. |
| `gazebo_overall.png` | screenshot | The Gazebo Harmonic view: the robot in the Tugbot warehouse with the table, the three cubes and the three coloured columns visible in one frame. |
| `gazebo_closeup.png` | screenshot | Close-up in Gazebo on the arm and gripper mid-grasp, picking a cube off the table. |
| `rviz_overall.png` | screenshot | The RViz mission layout: robot model, map and costmaps, the LIDAR scan, both camera image panels and the MotionPlanning panel. |
| `rviz_closeup.png` | screenshot | RViz *Gripper close-up* saved view: both camera point clouds and the MotionPlanning panel together during a pick. |
| `rviz_mapping.png` | screenshot | RViz while driving and mapping: the growing occupancy grid and the live LIDAR scan feeding slam_toolbox. |

Suggested capture settings:

- Run with `use_rviz:=true` so both views are available in the same run.
- For `banner.gif`, screen-record the Gazebo window and convert, e.g.
  `ffmpeg -i capture.mp4 -vf "fps=12,scale=960:-1:flags=lanczos" -loop 0 banner.gif`
- Keep the aspect ratio wide-ish (roughly 2:1) for the banner so it sits well at
  the top of the README.
- Capture `rviz_mapping.png` during `mapping.launch.py`, before the map is
  saved — that's what makes it visually distinct from `rviz_overall.png`.
