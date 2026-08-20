#!/usr/bin/env bash
# Generate official URDF files from the fixed franka_description tag (2.8.1).
#
# Outputs (deterministic, committed as project-derived baselines):
#   source/generated/mobile_fr3_duo_visual.urdf
#   source/generated/mobile_fr3_duo_self_collision.urdf
#   source/generated/collision_exclusions.yaml
#
# Requires ROS 2 Humble xacro and the fixed official checkout in the dev cache.
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE_DIR="${MOBILE_FR3_CACHE_DIR:-/home/siyuey/workspace/mujoco/_third_party_cache}"
FD_DIR="$CACHE_DIR/franka_description"
OUT_DIR="$ROOT/source/generated"
SRDF="$FD_DIR/robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.srdf.xacro"

if [[ ! -f "$FD_DIR/robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.urdf.xacro" ]]; then
  echo "error: franka_description 2.8.1 not found under $FD_DIR" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

# ROS 2 Humble xacro resolves $(find ...) via ament index. Build a temporary
# prefix that registers the fixed franka_description checkout as a package.
AMENT_PREFIX="$(mktemp -d)"
trap 'rm -rf "$AMENT_PREFIX"' EXIT
mkdir -p "$AMENT_PREFIX/share" \
         "$AMENT_PREFIX/share/ament_index/resource_index/packages"
ln -sfn "$FD_DIR" "$AMENT_PREFIX/share/franka_description"
printf 'share/franka_description\n' \
  > "$AMENT_PREFIX/share/ament_index/resource_index/packages/franka_description"
export AMENT_PREFIX_PATH="$AMENT_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

set -uo pipefail

XACRO_BASE=(
  "robot_types:=['tmrv0_2','fr3v2_1','fr3v2_1']"
  "hand:=true"
  "gazebo:=false"
  "reduced_version:=false"
)

xacro "$FD_DIR/robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.urdf.xacro" \
  "${XACRO_BASE[@]}" "with_sc:=false" \
  > "$OUT_DIR/mobile_fr3_duo_visual.urdf"

xacro "$FD_DIR/robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.urdf.xacro" \
  "${XACRO_BASE[@]}" "with_sc:=true" \
  > "$OUT_DIR/mobile_fr3_duo_self_collision.urdf"

xacro "$FD_DIR/robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.urdf.xacro" \
  "${XACRO_BASE[@]}" "with_sc:=false" "reduced_version:=true" \
  > "$OUT_DIR/mobile_fr3_duo_reduced.urdf"

# Freeze the official SRDF semantic collision exclusions that the production
# builder consumes.  Filter against the generated visual URDF just as the
# builder does, so this is a complete and directly usable derived input.
xacro "$SRDF" "${XACRO_BASE[@]}" \
  | python3 "$ROOT/tools/extract_collision_exclusions.py" \
      --urdf "$OUT_DIR/mobile_fr3_duo_visual.urdf" \
      --output "$OUT_DIR/collision_exclusions.yaml" \
      --source "franka_description@2.8.1/robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.srdf.xacro"

# Strip absolute dev-cache paths from the xacro header comments so the
# committed deliverables contain no absolute paths.
sed -i "s#$FD_DIR#franka_description@2.8.1#g" \
  "$OUT_DIR/mobile_fr3_duo_visual.urdf" \
  "$OUT_DIR/mobile_fr3_duo_self_collision.urdf" \
  "$OUT_DIR/mobile_fr3_duo_reduced.urdf"

echo "generated:"
echo "  $OUT_DIR/mobile_fr3_duo_visual.urdf"
echo "  $OUT_DIR/mobile_fr3_duo_self_collision.urdf"
echo "  $OUT_DIR/mobile_fr3_duo_reduced.urdf"
echo "  $OUT_DIR/collision_exclusions.yaml"
