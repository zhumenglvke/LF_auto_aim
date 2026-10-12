#!/usr/bin/env bash
# Start the ROS serial link, gimbal manager, and single-camera auto aim.
# Intended for a graphical desktop session (auto_aim_test_all_ros opens a window).
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_setup=/opt/ros/humble/setup.bash
workspace_setup="$project_dir/build/ros_install/setup.bash"
vendor_lib_dir="$project_dir/build/ros_vendor/opt/ros/humble/lib"
vision_bin="$project_dir/build/auto_aim_test_all_ros"

absolute_path() {
  case "$1" in
    /*) printf '%s\n' "$1" ;;
    *) printf '%s/%s\n' "$project_dir" "$1" ;;
  esac
}

vision_config="$(absolute_path "${SP_VISION_CONFIG:-configs/demo.yaml}")"
serial_config="$(absolute_path "${SP_VISION_SERIAL_CONFIG:-build/ros_install/standard_robot_pp_ros2/share/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml}")"

fail() {
  echo "[sp_vision] ERROR: $*" >&2
  exit 1
}

if [[ $# -gt 1 || ( $# -eq 1 && ${1:-} != --check ) ]]; then
  fail "usage: $0 [--check]"
fi

if [[ ${1:-} != --check ]]; then
  state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/sp_vision_single_camera"
  mkdir -p -- "$state_dir"
  exec 9>"$state_dir/autostart.lock"
  flock -n 9 || fail "already running"
  log_file="$state_dir/$(date +%Y%m%d-%H%M%S).log"
  exec >>"$log_file" 2>&1
  echo "[sp_vision] $(date --iso-8601=seconds) starting; log: $log_file"
fi

[[ -f "$ros_setup" ]] || fail "ROS setup missing: $ros_setup"
[[ -f "$workspace_setup" ]] || fail "workspace setup missing: $workspace_setup"
[[ -f "$vendor_lib_dir/libio_context.so" ]] || fail "ROS serial dependency missing: $vendor_lib_dir/libio_context.so"
[[ -f "$vendor_lib_dir/libserial_driver.so" ]] || fail "ROS serial dependency missing: $vendor_lib_dir/libserial_driver.so"
[[ -x "$vision_bin" ]] || fail "single-camera executable missing: $vision_bin"
[[ -f "$vision_config" ]] || fail "vision config missing: $vision_config"
[[ -f "$serial_config" ]] || fail "serial config missing: $serial_config"

source "$ros_setup"
source "$workspace_setup"
# The serial packages were built against libraries staged outside ros_install.
export LD_LIBRARY_PATH="$vendor_lib_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

package_prefix="$(ros2 pkg prefix standard_robot_pp_ros2)" || fail "standard_robot_pp_ros2 is not available"
serial_bin="$package_prefix/lib/standard_robot_pp_ros2/standard_robot_pp_ros2_node"
manager_bin="$package_prefix/lib/standard_robot_pp_ros2/gimbal_manager_node"
[[ -x "$serial_bin" ]] || fail "serial node missing: $serial_bin"
[[ -x "$manager_bin" ]] || fail "gimbal manager missing: $manager_bin"
serial_library="$package_prefix/lib/libstandard_robot_pp_ros2.so"
[[ -f "$serial_library" ]] || fail "serial library missing: $serial_library"

check_libraries() {
  local binary=$1 report missing
  report="$(ldd "$binary" 2>&1)" || fail "cannot inspect shared libraries for $binary: $report"
  missing="$(awk '/not found/ { print $1 }' <<< "$report")"
  [[ -z "$missing" ]] || fail "unresolved libraries for $binary: $missing"
}
for binary in "$serial_library" "$serial_bin" "$manager_bin" "$vision_bin"; do
  check_libraries "$binary"
done

serial_device="$(awk '$1 == "device_name:" { print $2; exit }' "$serial_config")"
serial_device="${serial_device%\"}"
serial_device="${serial_device#\"}"
[[ -n "$serial_device" ]] || fail "device_name missing in $serial_config"
command -v xdpyinfo >/dev/null || fail "xdpyinfo is required to check the graphical display"

serial_in_use() {
  command -v fuser >/dev/null && fuser -s "$serial_device" 2>/dev/null
}

if [[ ${1:-} == --check ]]; then
  [[ -c "$serial_device" ]] || fail "serial device missing: $serial_device"
  [[ -r "$serial_device" && -w "$serial_device" ]] || fail "no read/write permission for $serial_device"
  if serial_in_use; then fail "serial device is already in use: $serial_device"; fi
  [[ -n ${DISPLAY:-} ]] || fail "DISPLAY is unset; run from the graphical session"
  timeout 5s xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 || fail "cannot access X display: $DISPLAY"
  echo "[sp_vision] preflight OK"
  echo "  vision: $vision_bin"
  echo "  serial: $serial_bin"
  echo "  manager: $manager_bin"
  echo "  device: $serial_device"
  echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
  exit 0
fi

cd "$project_dir"

serial_pid=
manager_pid=
vision_pid=
imu_pid=
cleanup() {
  trap - EXIT INT TERM HUP
  for pid in "$imu_pid" "$vision_pid" "$manager_pid" "$serial_pid"; do
    [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
  done
  for pid in "$imu_pid" "$vision_pid" "$manager_pid" "$serial_pid"; do
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

echo "[sp_vision] waiting for X display $DISPLAY"
[[ -n ${DISPLAY:-} ]] || fail "DISPLAY is unset; graphical login is required"
for ((attempt = 0; attempt < 30; ++attempt)); do
  timeout 2s xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 && break
  sleep 1
done
timeout 2s xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 || fail "cannot access X display: $DISPLAY"

echo "[sp_vision] waiting for $serial_device"
for ((attempt = 0; attempt < 60; ++attempt)); do
  [[ -c "$serial_device" ]] && break
  sleep 1
done
[[ -c "$serial_device" ]] || fail "serial device did not appear: $serial_device"
[[ -r "$serial_device" && -w "$serial_device" ]] || fail "no read/write permission for $serial_device"
echo "[sp_vision] waiting for exclusive access to $serial_device"
for ((attempt = 0; attempt < 30; ++attempt)); do
  serial_in_use || break
  sleep 1
done
if serial_in_use; then fail "serial device is already in use: $serial_device"; fi
[[ -c "$serial_device" ]] || fail "serial device disappeared before startup: $serial_device"

"$serial_bin" --ros-args -r __node:=standard_robot_pp_ros2 --params-file "$serial_config" 9>&- &
serial_pid=$!
"$manager_bin" 9>&- &
manager_pid=$!

echo "[sp_vision] waiting for /serial/imu"
timeout 90s ros2 topic echo --no-daemon --once /serial/imu sensor_msgs/msg/Imu >/dev/null 9>&- &
imu_pid=$!
while kill -0 "$imu_pid" 2>/dev/null; do
  kill -0 "$serial_pid" 2>/dev/null || fail "serial node exited before /serial/imu was received"
  kill -0 "$manager_pid" 2>/dev/null || fail "gimbal manager exited before /serial/imu was received"
  sleep 1
done
if wait "$imu_pid"; then
  :
else
  imu_status=$?
  if [[ $imu_status -eq 124 ]]; then
    fail "no /serial/imu message within 90 seconds"
  fi
  fail "ROS IMU check failed (status $imu_status)"
fi
imu_pid=
kill -0 "$serial_pid" 2>/dev/null || fail "serial node exited"
kill -0 "$manager_pid" 2>/dev/null || fail "gimbal manager exited"

echo "[sp_vision] starting single-camera auto aim"
"$vision_bin" "$vision_config" 9>&- &
vision_pid=$!

exited_pid=
if wait -n -p exited_pid "$serial_pid" "$manager_pid" "$vision_pid"; then
  exit_status=0
else
  exit_status=$?
fi
if [[ ${exited_pid:-} == "$vision_pid" && $exit_status -eq 0 ]]; then
  echo "[sp_vision] single-camera auto aim exited normally"
  exit 0
fi
case ${exited_pid:-} in
  "$serial_pid") process_name="serial node" ;;
  "$manager_pid") process_name="gimbal manager" ;;
  "$vision_pid") process_name="single-camera auto aim" ;;
  *) process_name="a child process" ;;
esac
fail "$process_name exited (status $exit_status)"

