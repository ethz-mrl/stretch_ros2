# Docker Dev Workflow for Mounted stretch_mujoco

This guide is for development mode where you mount a local `stretch_mujoco` checkout into the container and edit it live.

It documents:
- What each file does
- One-time setup commands
- Day-to-day commands
- Common failures and fixes

## 1. Why This Is Different From Image-Only Mode

Without a mount, the container uses the copy prepared during Docker build.

With a mount, your host repo overlays `/root/repos/stretch_mujoco` inside the container. This means:
- Your host checkout must be complete (submodules, assets)
- Python imports and assets come from host-mounted files
- You can modify `stretch_mujoco` live and test immediately

## 2. Files Used In This Workflow

- `docker-compose.yml`
  - Mounts your local `stretch_mujoco` into `/root/repos/stretch_mujoco`
  - Passes environment toggles into the container

- `.env`
  - `STRETCH_MUJOCO_HOST_DIR`: absolute host path to your local `stretch_mujoco`
  - `STRETCH_MUJOCO_EDITABLE_INSTALL`: set to `1` for editable development mode

- `entrypoint.sh`
  - Sources ROS setup safely
  - Exposes mounted Robocasa and Robosuite on `PYTHONPATH`
  - Optionally runs editable pip install path when enabled
  - Warns if Robocasa assets are missing

- `Dockerfile`
  - Builds the base image and initial workspace

## 3. One-Time Host Setup (Required)

Run these on the host machine (not inside container):

    cd <path-to-repo>/stretch_mujoco
    git submodule sync --recursive
    git submodule update --init --recursive

This ensures `third_party/robocasa` and `third_party/robosuite` are populated.

## 4. Configure .env For Dev Mode

Set these values in `.env` next to `docker-compose.yml`:

    STRETCH_MUJOCO_HOST_DIR=/absolute/path/to/your/stretch_mujoco
    STRETCH_MUJOCO_EDITABLE_INSTALL=1

Example:

    STRETCH_MUJOCO_HOST_DIR=<path-to-repo>/stretch_mujoco
    STRETCH_MUJOCO_EDITABLE_INSTALL=1

## 5. Build And Start Container

From `stretch_simulation`:

    docker compose down
    docker compose up --build

## 6. One-Time In-Container Robocasa Asset Download

If this is your first mounted checkout, run once inside the container:

    docker compose exec stretch-simulation bash
    cd /root/repos/stretch_mujoco
    python3 third_party/robocasa/robocasa/scripts/setup_macros.py
    python3 third_party/robocasa/robocasa/scripts/download_kitchen_assets.py

This downloads required kitchen assets (multi-GB).

## 7. Quick Validation Commands

Run inside the container:

    python3 -c "import robocasa, robosuite; print(robocasa.__file__)"

Expected: path under `/root/repos/stretch_mujoco/third_party/robocasa/...`

Optional asset sanity check:

    python3 - <<'PY'
    import os
    base = "/root/repos/stretch_mujoco/third_party/robocasa/robocasa/models/assets"
    print("objects", os.path.isdir(base + "/objects"))
    print("textures", os.path.isdir(base + "/textures"))
    PY

Both should be `True`.

## 8. Launch Example

    ros2 launch stretch_simulation stretch_mujoco_driver.launch.py \
      use_mujoco_viewer:=true \
      mode:=navigation \
      use_rviz:=false \
      robocasa_layout:='G-shaped' \
      robocasa_style:=Modern_1

## 9. Common Errors And Fixes

### ModuleNotFoundError: No module named robocasa

Cause:
- Empty or missing `third_party/robocasa`

Fix:

    cd <path-to-repo>/stretch_mujoco
    git submodule update --init --recursive
    docker compose restart stretch-simulation

### ValueError: Error opening file .../dist-packages/robosuite/.../link0.stl

Cause:
- Robosuite was installed non-editably during Docker build. Pip copies only declared package_data, which does not include large binary assets (STL meshes). When robosuite then generates the MuJoCo XML, it embeds the pip path, and MuJoCo cannot open the missing files.

Fix:
- Restart the container so the entrypoint runs again. With `STRETCH_MUJOCO_EDITABLE_INSTALL=1`, it now uninstalls the non-editable version first and reinstalls it from the mounted source tree where the mesh files exist.

    docker compose restart stretch-simulation

### ValueError: probabilities contain NaN

Cause:
- Robocasa object/texture assets missing

Fix:

    docker compose exec stretch-simulation bash -lc "cd /root/repos/stretch_mujoco && python3 third_party/robocasa/robocasa/scripts/download_kitchen_assets.py"

### /opt/ros/humble/setup.bash: AMENT_TRACE_SETUP_FILES: unbound variable

Status:
- Already handled by current compose and entrypoint setup.

## 10. Daily Development Loop

1. Edit files in your host `stretch_mujoco` repo.
2. Keep compose service running.
3. Relaunch ROS command in the container.
4. If dependency metadata changed significantly, restart service:

    docker compose restart stretch-simulation

## 11. Notes

- Editable mode is for rapid iteration and can be less deterministic than image-only mode.
- For reproducible CI-like behavior, disable mount and rely on build-time setup.
