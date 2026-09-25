## What this is
A ROS2 Python node (`catcher_node.py`) implementing a reactive pursuit
algorithm: it steers toward the Runner's live position while using LiDAR
for obstacle avoidance, and declares capture once it stays within a set
radius of the Runner for a short confirmation window.

## Before you submit — confirm these against the official rulebook
The event page only describes the *rules of the round*, not the technical
interface (topic names, message types, arena dimensions, platform specs).
Check **FAQs & Discussions** on the Unstop page, or any rulebook/starter
repo the organizers share, for:

| Item | Placeholder used here | Where to fix it |
|---|---|---|
| Your robot's odometry topic | `/odom` | `SELF_ODOM_TOPIC` in `catcher_node.py` |
| Runner's position topic/type | `/target_pose` (PoseStamped) | `TARGET_TOPIC` — may need to be `Odometry`, a custom msg, or derived from camera/LiDAR if no direct pose is given |
| Velocity command topic | `/cmd_vel` | `CMD_VEL_TOPIC` |
| Max linear/angular speed | 0.30 m/s / 1.80 rad/s | `MAX_LINEAR_SPEED`, `MAX_ANGULAR_SPEED` — match TurtleBot3 or TurtleBot4 Lite spec, whichever they use |
| Capture radius | 0.30 m | `CAPTURE_RADIUS` — use their official definition of "caught" if stated |

If the Runner's position isn't published directly (e.g. you only get
`/scan` or a camera feed and must detect the target yourself), you'll need
to add a detection/tracking step before this pursuit logic — happy to help
build that once you know the actual sensing setup.

## How to run (once topics are confirmed)
```bash
# from your ROS2 workspace, after sourcing setup.bash
python3 catcher_node.py
# or, if packaged as a proper ROS2 package:
ros2 run <your_package> catcher_node
```

## Algorithm summary
1. **Track**: subscribe to own odometry and the Runner's pose.
2. **Pursue**: compute bearing to Runner, turn proportionally to heading
   error, drive forward scaled by how well-aligned the heading is.
3. **Avoid obstacles**: reactive LiDAR check — steers away from the
   nearest close obstacle and slows/stops if too close.
4. **Capture**: confirmed after staying within `CAPTURE_RADIUS` of the
   Runner for `CAPTURE_HOLD_TIME` seconds (avoids one noisy frame
   triggering a false capture).

## Suggested improvements once you have real specs
- Swap the simple proportional turn for a PD controller (add a derivative
  term on heading error) for smoother tracking.
- Add basic prediction (estimate Runner velocity, aim at a lead point
  instead of its current position) if the Runner moves fast/erratically.
- Tune `MAX_LINEAR_SPEED` / gains empirically in simulation to minimize
  capture time, since that's the qualification metric.
