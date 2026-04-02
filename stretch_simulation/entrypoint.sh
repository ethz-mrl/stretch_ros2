#!/usr/bin/env bash
set -euo pipefail

# Initialize ROS environment for all container commands.
# ROS setup scripts may read optional unset vars (e.g. AMENT_TRACE_SETUP_FILES),
# so temporarily disable nounset while sourcing them.
set +u
source /opt/ros/humble/setup.bash
source /root/ament_ws/install/setup.bash
set -u

# Optional: only refresh mounted stretch_mujoco as editable when explicitly enabled.
# Default is off to avoid startup failures when host repo differs from image-time layout.
if [ -d /root/repos/stretch_mujoco ]; then
  robocasa_assets_root="/root/repos/stretch_mujoco/third_party/robocasa/robocasa/models/assets"
  if [ ! -d "$robocasa_assets_root/objects" ] || [ ! -d "$robocasa_assets_root/textures" ]; then
    echo "[stretch-sim-entrypoint] WARNING: Robocasa assets appear incomplete (missing objects/textures)."
    echo "[stretch-sim-entrypoint] Run once inside container: python3 /root/repos/stretch_mujoco/third_party/robocasa/robocasa/scripts/download_kitchen_assets.py"
  fi

  # If mounted source tree is present, expose local third_party python packages.
  if [ -d /root/repos/stretch_mujoco/third_party/robocasa/robocasa ]; then
    export PYTHONPATH="/root/repos/stretch_mujoco/third_party/robocasa:${PYTHONPATH:-}"
  fi
  if [ -d /root/repos/stretch_mujoco/third_party/robosuite/robosuite ]; then
    export PYTHONPATH="/root/repos/stretch_mujoco/third_party/robosuite:${PYTHONPATH:-}"
  fi

  if [ "${STRETCH_MUJOCO_EDITABLE_INSTALL:-0}" = "1" ]; then
    echo "[stretch-sim-entrypoint] Mounted stretch_mujoco detected; running editable install refresh"
    export PIP_CONSTRAINT=/tmp/stretch_mujoco_constraints.txt
    echo "mujoco==3.2.6" > "$PIP_CONSTRAINT"

    cd /root/repos/stretch_mujoco
    git submodule update --init --recursive || true

    # During Docker build, robosuite is installed non-editably (pip copies files to dist-packages
    # but does NOT include large binary assets such as STL meshes, which are not in package_data).
    # The non-editable install must be removed first so that pip replaces it with an editable
    # reference pointing to the mounted source tree where the actual mesh files exist.
    echo "[stretch-sim-entrypoint] Removing image-built non-editable packages before editable reinstall"
    python3 -m pip uninstall -y --root-user-action=ignore robosuite robocasa hello-robot-stretch-mujoco 2>/dev/null || true

    install_editable_if_python_project() {
      local pkg_path="$1"
      if [ -f "$pkg_path/pyproject.toml" ] || [ -f "$pkg_path/setup.py" ]; then
        python3 -m pip install --root-user-action=ignore --no-deps -e "$pkg_path"
      else
        echo "[stretch-sim-entrypoint] Skipping $pkg_path (not a Python project)"
      fi
    }

    # Editable install only; do not re-resolve or downgrade dependencies at container startup.
    python3 -m pip install --root-user-action=ignore --no-deps -e ".[robocasa]"

    if [ -d third_party/robocasa ]; then
      install_editable_if_python_project third_party/robocasa
    fi

    if [ -d third_party/robosuite ]; then
      install_editable_if_python_project third_party/robosuite
    fi
  else
    echo "[stretch-sim-entrypoint] Mounted stretch_mujoco detected; using image-installed deps (set STRETCH_MUJOCO_EDITABLE_INSTALL=1 to enable runtime editable install)"
  fi
fi

cd /root/ament_ws
exec "$@"
