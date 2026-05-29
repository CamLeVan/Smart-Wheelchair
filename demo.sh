#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

LAUNCH_FILE="smart_wheelchair_navigation gazebo_demo.launch.py"

source_ros() {
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ERROR: /opt/ros/humble/setup.bash not found. Install/source ROS 2 Humble first."
    exit 1
  fi

  set +u
  # shellcheck source=/opt/ros/humble/setup.bash
  source /opt/ros/humble/setup.bash
  set -u

  if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
    echo "ERROR: install/setup.bash not found. Run: ./demo.sh build"
    exit 1
  fi

  set +u
  # shellcheck source=/dev/null
  source "$ROOT_DIR/install/setup.bash"
  set -u
}

build_workspace() {
  set +u
  # shellcheck source=/opt/ros/humble/setup.bash
  source /opt/ros/humble/setup.bash
  set -u
  colcon build --symlink-install
}

launch_mode() {
  local mode="$1"
  shift || true
  source_ros
  exec ros2 launch $LAUNCH_FILE mode:="$mode" "$@"
}

run_teleop() {
  source_ros
  exec ros2 run teleop_twist_keyboard teleop_twist_keyboard
}

echo_cmd_vel() {
  source_ros
  exec ros2 topic echo /cmd_vel
}

topic_status() {
  source_ros
  ros2 topic list --no-daemon | sort
}

astar_test() {
  source_ros
  timeout 25 python3 - <<'PY'
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose


class PathProbe(Node):
    def __init__(self):
        super().__init__('astar_path_probe')
        self.client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')


rclpy.init()
node = PathProbe()

if not node.client.wait_for_server(timeout_sec=10.0):
    raise RuntimeError('Nav2 is not ready. Run ./demo.sh nav first, then retry ./demo.sh astar-test in another terminal.')

goal = ComputePathToPose.Goal()
goal.planner_id = 'GridBased'
goal.use_start = True
goal.start.header.frame_id = 'map'
goal.start.pose.position.x = 0.0
goal.start.pose.position.y = 0.0
goal.start.pose.orientation.w = 1.0
goal.goal.header.frame_id = 'map'
goal.goal.pose.position.x = 7.5
goal.goal.pose.position.y = 2.5
goal.goal.pose.orientation.w = 1.0

send_future = node.client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
goal_handle = send_future.result()
if goal_handle is None or not goal_handle.accepted:
    raise RuntimeError('A* path request was rejected.')

result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future, timeout_sec=15.0)
result = result_future.result().result
poses = result.path.poses

length = 0.0
for a, b in zip(poses, poses[1:]):
    dx = b.pose.position.x - a.pose.position.x
    dy = b.pose.position.y - a.pose.position.y
    length += math.hypot(dx, dy)

print('A* path accepted')
print(f'poses: {len(poses)}')
print(f'length_m: {length:.2f}')
print(f'start: ({poses[0].pose.position.x:.2f}, {poses[0].pose.position.y:.2f})')
print(f'goal: ({poses[-1].pose.position.x:.2f}, {poses[-1].pose.position.y:.2f})')

node.destroy_node()
rclpy.shutdown()
PY
}

show_help() {
  cat <<'EOF'
Smart Wheelchair Gazebo demo helper

Usage:
  ./demo.sh                 Open menu
  ./demo.sh build           Build workspace
  ./demo.sh world           Demo 1: Gazebo robot, sensors, manual teleop
  ./demo.sh slam            Demo 2: SLAM Toolbox mapping
  ./demo.sh nav             Demo 3: Nav2 + A* planning
  ./demo.sh follow          Demo 4: YOLO follow-me
  ./demo.sh follow-debug    Follow-me with OpenCV debug window
  ./demo.sh teleop          Keyboard control in a second terminal
  ./demo.sh cmd             Watch /cmd_vel
  ./demo.sh topics          List ROS topics
  ./demo.sh astar-test      Test A* path while nav mode is running

Stop the current demo with Ctrl+C before starting another mode.
Do not run teleop, nav, and follow at the same time because all publish /cmd_vel.
EOF
}

show_menu() {
  cat <<'EOF'

Smart Wheelchair Gazebo Demo

Main demos:
  1) world  - Gazebo + robot + sensors + manual teleop
  2) slam   - SLAM Toolbox builds a 2D hospital map
  3) nav    - Nav2 autonomous navigation with A*
  4) follow - YOLO + Lidar follow-me

Tools:
  5) teleop       - keyboard control in this terminal
  6) cmd          - watch /cmd_vel
  7) astar-test   - verify A* path while nav is running
  8) topics       - list ROS topics
  9) build        - rebuild workspace
  q) quit

EOF
  read -r -p "Choose: " choice
  case "$choice" in
    1) launch_mode world ;;
    2) launch_mode slam ;;
    3) launch_mode nav ;;
    4) launch_mode follow use_rviz:=false ;;
    5) run_teleop ;;
    6) echo_cmd_vel ;;
    7) astar_test ;;
    8) topic_status ;;
    9) build_workspace ;;
    q|Q) exit 0 ;;
    *) echo "Unknown choice: $choice"; exit 1 ;;
  esac
}

command="${1:-menu}"

case "$command" in
  menu) show_menu ;;
  help|-h|--help) show_help ;;
  build) build_workspace ;;
  world) launch_mode world ;;
  slam) launch_mode slam ;;
  nav) launch_mode nav ;;
  follow) launch_mode follow use_rviz:=false ;;
  follow-debug) launch_mode follow use_rviz:=false show_debug_view:=true ;;
  teleop) run_teleop ;;
  cmd|cmd_vel) echo_cmd_vel ;;
  topics) topic_status ;;
  astar-test) astar_test ;;
  *)
    echo "Unknown command: $command"
    echo
    show_help
    exit 1
    ;;
esac
