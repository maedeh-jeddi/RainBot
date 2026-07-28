# Images used by the top-level README

Drop the files here under exactly these names and the README will pick them up.

| File | Type | What it should show |
| --- | --- | --- |
| `banner.gif` | animated GIF | The headline shot. A short loop of one full pick-and-place: the base driving up to the table, the arm descending and grasping a cube, then placing it on its column. Keep it a few seconds and reasonably small (a few MB) — GitHub will not lazy-load it. |
| `gazebo.png` | screenshot | The Gazebo Harmonic view: the robot in the Tugbot warehouse with the table, the three cubes and the three coloured columns visible in one frame. |
| `rviz.png` | screenshot | The RViz mission layout: robot model, map and costmaps, the LIDAR scan, the front-camera image panel and the MotionPlanning panel. |

Suggested capture settings:

- Run with `use_rviz:=true` so both views are available in the same run.
- For `banner.gif`, screen-record the Gazebo window and convert, e.g.
  `ffmpeg -i capture.mp4 -vf "fps=12,scale=960:-1:flags=lanczos" -loop 0 banner.gif`
- Keep the aspect ratio wide-ish (roughly 2:1) for the banner so it sits well at
  the top of the README.
