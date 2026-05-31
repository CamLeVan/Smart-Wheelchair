#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

LAUNCH_FILE="smart_wheelchair_navigation gazebo_demo.launch.py"

sanitize_snap_env() {
  # GUI ROS tools can accidentally load Snap runtime libraries when launched
  # from a snap-packaged VS Code terminal. Remove those variables for children.
  local name
  for name in ${!SNAP@}; do
    unset "$name"
  done

  unset GTK_EXE_PREFIX GTK_PATH GTK_IM_MODULE_FILE GIO_MODULE_DIR
  unset GIO_LAUNCHED_DESKTOP_FILE GIO_LAUNCHED_DESKTOP_FILE_PID
  unset LOCPATH SNAP_LIBRARY_PATH
  unset XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG XDG_DATA_DIRS_VSCODE_SNAP_ORIG

  if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
    local cleaned="" part
    IFS=':' read -ra parts <<< "$LD_LIBRARY_PATH"
    for part in "${parts[@]}"; do
      [[ "$part" == /snap/* || "$part" == /var/lib/snapd/* ]] && continue
      if [[ -z "$cleaned" ]]; then
        cleaned="$part"
      else
        cleaned="$cleaned:$part"
      fi
    done
    export LD_LIBRARY_PATH="$cleaned"
  fi

  if [[ "${SMART_WHEELCHAIR_SOFTWARE_GL:-0}" == "1" ]]; then
    export LIBGL_ALWAYS_SOFTWARE=1
    export QT_XCB_FORCE_SOFTWARE_OPENGL=1
  fi

  local fastdds_profile="$ROOT_DIR/src/smart_wheelchair_navigation/config/fastdds_no_shm.xml"
  if [[ "${SMART_WHEELCHAIR_FASTDDS_SHM:-0}" != "1" && -f "$fastdds_profile" ]]; then
    export FASTDDS_DEFAULT_PROFILES_FILE="$fastdds_profile"
    export FASTRTPS_DEFAULT_PROFILES_FILE="$fastdds_profile"
  fi
}

source_ros() {
  sanitize_snap_env

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
  sanitize_snap_env

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

costmap_save() {
  local topic="${1:-/global_costmap/costmap}"
  local out_dir="$ROOT_DIR/demo_outputs/costmaps"
  mkdir -p "$out_dir"
  source_ros
  timeout 30 python3 - "$topic" "$out_dir" <<'PY'
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


topic = sys.argv[1]
out_dir = sys.argv[2]
stamp = time.strftime("%Y%m%d_%H%M%S")
name = topic.strip("/").replace("/", "_") or "costmap"
prefix = os.path.join(out_dir, f"{name}_{stamp}")


class CostmapSaver(Node):
    def __init__(self):
        super().__init__("costmap_saver")
        self.msg = None
        live_qos = QoSProfile(depth=10)
        live_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        live_qos.durability = DurabilityPolicy.VOLATILE
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, topic, self.on_costmap, live_qos)
        self.create_subscription(OccupancyGrid, topic, self.on_costmap, latched_qos)

    def on_costmap(self, msg):
        self.msg = msg


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def cost_to_gray(value):
    if value < 0:
        return 205
    value = max(0, min(100, int(value)))
    return 254 - round(value * 254 / 100)


rclpy.init()
node = CostmapSaver()
deadline = time.monotonic() + 20.0
while rclpy.ok() and node.msg is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)

msg = node.msg
node.destroy_node()
rclpy.shutdown()

if msg is None:
    raise RuntimeError(
        f"No costmap received from {topic}. Run './demo.sh nav' first, "
        "then run './demo.sh costmap' in another terminal."
    )

width = msg.info.width
height = msg.info.height
resolution = msg.info.resolution
origin = msg.info.origin
data = list(msg.data)

pgm_path = prefix + ".pgm"
yaml_path = prefix + ".yaml"
txt_path = prefix + ".txt"

with open(pgm_path, "wb") as pgm:
    pgm.write(f"P5\n# ROS2 Nav2 costmap from {topic}\n{width} {height}\n255\n".encode("ascii"))
    for y in range(height - 1, -1, -1):
        row = data[y * width:(y + 1) * width]
        pgm.write(bytes(cost_to_gray(v) for v in row))

yaw = yaw_from_quat(origin.orientation)
with open(yaml_path, "w", encoding="ascii") as yaml:
    yaml.write(f"image: {os.path.basename(pgm_path)}\n")
    yaml.write(f"resolution: {resolution:.8f}\n")
    yaml.write(
        "origin: "
        f"[{origin.position.x:.8f}, {origin.position.y:.8f}, {yaw:.8f}]\n"
    )
    yaml.write("negate: 0\n")
    yaml.write("occupied_thresh: 0.65\n")
    yaml.write("free_thresh: 0.196\n")

unknown = sum(1 for v in data if v < 0)
occupied = sum(1 for v in data if v >= 65)
inflated = sum(1 for v in data if 1 <= v < 65)
free = sum(1 for v in data if v == 0)
with open(txt_path, "w", encoding="ascii") as txt:
    txt.write(f"topic: {topic}\n")
    txt.write(f"width: {width}\n")
    txt.write(f"height: {height}\n")
    txt.write(f"resolution_m_per_cell: {resolution:.8f}\n")
    txt.write(f"origin_x: {origin.position.x:.8f}\n")
    txt.write(f"origin_y: {origin.position.y:.8f}\n")
    txt.write(f"origin_yaw: {yaw:.8f}\n")
    txt.write(f"free_cells: {free}\n")
    txt.write(f"inflated_cells: {inflated}\n")
    txt.write(f"occupied_cells: {occupied}\n")
    txt.write(f"unknown_cells: {unknown}\n")

print("Saved costmap")
print(f"topic: {topic}")
print(f"pgm: {pgm_path}")
print(f"yaml: {yaml_path}")
print(f"summary: {txt_path}")
print(f"size: {width}x{height}, resolution: {resolution:.3f} m/cell")
print(f"cells: free={free}, inflated={inflated}, occupied={occupied}, unknown={unknown}")
PY
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
  ./demo.sh nav             Demo 3: A* planning (gzserver + RViz)
  ./demo.sh nav-gui         A* planning with Gazebo GUI on faster machines
  ./demo.sh nav-full        Experimental full Nav2 stack on faster machines
  ./demo.sh follow          Demo 4: follow-me sensor fusion (gzserver only)
  ./demo.sh follow-gui      Follow-me with Gazebo GUI on faster machines
  ./demo.sh follow-yolo     Follow-me with real YOLO backend
  ./demo.sh follow-debug    Follow-me with OpenCV debug window
  ./demo.sh teleop          Keyboard control in a second terminal
  ./demo.sh cmd             Watch /cmd_vel
  ./demo.sh topics          List ROS topics
  ./demo.sh astar-test      Test A* path while nav mode is running
  ./demo.sh costmap [topic] Save costmap PGM/YAML while nav is running

Stop the current demo with Ctrl+C before starting another mode.
Do not run teleop, nav, and follow at the same time because all publish /cmd_vel.
If RViz has OpenGL issues in VirtualBox, try:
  SMART_WHEELCHAIR_SOFTWARE_GL=1 ./demo.sh <mode>
FastDDS shared-memory transport is disabled by default for VirtualBox stability.
Use SMART_WHEELCHAIR_FASTDDS_SHM=1 ./demo.sh <mode> to restore it.
EOF
}

show_menu() {
  cat <<'EOF'

Smart Wheelchair Gazebo Demo

Main demos:
  1) world  - Gazebo + robot + sensors + manual teleop
  2) slam   - SLAM Toolbox builds a 2D hospital map
  3) nav    - A* path planning
  4) follow - Lidar/camera follow-me

Tools:
  5) teleop       - keyboard control in this terminal
  6) cmd          - watch /cmd_vel
  7) astar-test   - verify A* path while nav is running
  8) topics       - list ROS topics
  9) build        - rebuild workspace
  10) costmap     - save /global_costmap/costmap to demo_outputs
  q) quit

EOF
  read -r -p "Choose: " choice
  case "$choice" in
    1) launch_mode world rviz_config:="$ROOT_DIR/src/smart_wheelchair_navigation/config/world_config.rviz" ;;
    2) launch_mode slam ;;
    3) launch_mode nav gazebo_gui:=false ;;
    4) launch_mode follow gazebo_gui:=false use_rviz:=false vision_backend:=sim_scan ;;
    5) run_teleop ;;
    6) echo_cmd_vel ;;
    7) astar_test ;;
    8) topic_status ;;
    9) build_workspace ;;
    10) costmap_save ;;
    q|Q) exit 0 ;;
    *) echo "Unknown choice: $choice"; exit 1 ;;
  esac
}

command="${1:-menu}"

case "$command" in
  menu) show_menu ;;
  help|-h|--help) show_help ;;
  build) build_workspace ;;
  world) launch_mode world rviz_config:="$ROOT_DIR/src/smart_wheelchair_navigation/config/world_config.rviz" ;;
  slam) launch_mode slam ;;
  nav) launch_mode nav gazebo_gui:=false ;;
  nav-gui) launch_mode nav ;;
  nav-full) launch_mode nav_full gazebo_gui:=false ;;
  follow) launch_mode follow gazebo_gui:=false use_rviz:=false vision_backend:=sim_scan ;;
  follow-gui) launch_mode follow use_rviz:=false vision_backend:=sim_scan ;;
  follow-yolo) launch_mode follow gazebo_gui:=false use_rviz:=false vision_backend:=yolo ;;
  follow-debug) launch_mode follow gazebo_gui:=false use_rviz:=false show_debug_view:=true vision_backend:=sim_scan ;;
  teleop) run_teleop ;;
  cmd|cmd_vel) echo_cmd_vel ;;
  topics) topic_status ;;
  astar-test) astar_test ;;
  costmap|costmap-save) costmap_save "${2:-/global_costmap/costmap}" ;;
  *)
    echo "Unknown command: $command"
    echo
    show_help
    exit 1
    ;;
esac
