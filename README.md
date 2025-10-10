# RoboPicker

SO-101 robotic arm simulation in MuJoCo with LeRobot integration for behavior cloning dataset collection.

## Quick Start

### Recording Demonstrations

```bash
source .venv/bin/activate
python lerobot/src/lerobot/scripts/lerobot_record.py \
    --config configs/so101_mujoco_record.yaml
```

**Note**: Edit `configs/so101_mujoco_record.yaml` to set your dataset repo ID and recording parameters.

**Keyboard Controls:**
- **W/S**: Move forward/backward (Y-axis)
- **A/D**: Move left/right (X-axis)
- **Q/E**: Move up/down (Z-axis)
- **[/]**: Rotate wrist left/right
- **O/C**: Open/close gripper
- **ESC**: Stop recording

### Replaying Episodes

```bash
source .venv/bin/activate
python lerobot/src/lerobot/scripts/lerobot_replay.py \
    --robot.type=so101_mujoco \
    --robot.xml_path=SO101/pick_scene.xml \
    --dataset.repo_id=your_username/dataset_name \
    --dataset.episode=0
```

**Note**: Cube positions are automatically restored from episode metadata.

## Overview

This project integrates the SO-101 5-DOF robot arm with HuggingFace LeRobot for sim-to-real behavior cloning research. The system enables intuitive keyboard teleoperation in simulation to collect high-quality demonstration datasets.

### LeRobot Integration

We extend LeRobot's `Robot` base class to support MuJoCo simulation with custom teleoperation:

- **Custom Robot Class**: `SO101MujocoRobot` implements LeRobot's robot interface for MuJoCo
- **Keyboard Teleop**: Purpose-built teleoperator converts keyboard input to end-effector velocities
- **Multi-Rate Control**: Records actions at 30 Hz while running control at 180 Hz and physics at 360 Hz
- **Dataset Compatibility**: Produces standard LeRobot datasets compatible with training pipelines

The robot implementation lives in `lerobot/src/lerobot/robots/so101_mujoco/` and integrates seamlessly with LeRobot's recording and replay scripts.

### SO-101 Assets

The simulation uses physically accurate SO-101 assets located in `SO101/`:

- **Robot Model**: `pick_scene.xml` contains the complete scene with robot, table, objects, and cameras
- **Collision Geometry**: Uses convex decomposition for accurate gripper collision (see [asset_processing.md](SO101/asset_processing.md))
- **Scene Objects**: Includes manipulable cube and container for pick-and-place tasks

**Important**: The original gripper meshes created a large "invisible hitbox" due to MuJoCo's single-convex-hull limitation. We solved this using CoACD to decompose the gripper into 24 tight-fitting convex hulls (12 per jaw). This provides accurate collision without performance penalty. See `SO101/asset_processing.md` for technical details.

## Installation

**Prerequisites:**
- Python 3.10+
- [uv](https://github.com/astral-sh/uv) package manager

**Setup:**
```bash
# Clone repository
git clone --recursive https://github.com/lohpaul9/robopicker.git
cd robopicker

# Install dependencies
uv sync
source .venv/bin/activate
```

**macOS mjpython fix:**
```bash
mkdir -p .venv/lib
ln -sf ~/.local/share/uv/python/cpython-3.10.*/lib/libpython3.10.dylib .venv/lib/
```
See [MuJoCo #1923](https://github.com/google-deepmind/mujoco/issues/1923)

**Alternative macOS mjpython fix:**

NOTE: This didn't fix it for me on macOS. Instead I had to use the Python from `pyenv` instead of the one from `uv`
(maybe this isn't a problem any more now that we don't use gym-hil?)
```bash
brew install pyenv
export PYTHON_CONFIGURE_OPTS="--enable-framework"
# Also need to install `xz` otherwise you'll see "No module named '_lzma'" error
# when running LeRobot record.
brew install xz
```

Add pyenv init lines to ~/.zshrc, then reload your shell:
```bash
# Add these lines if they’re not already present
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.zshrc
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.zshrc
echo 'eval "$(pyenv init -)"' >> ~/.zshrc
exec $SHELL -l    # reload login shell
```

Now install Python 3.10.13 and recreate venv
```bash
pyenv install 3.10.13
cd /path/to/robopicker
pyenv local 3.10.13        # scoped to this repo only (creates .python-version)
pyenv rehash

# recreate .venv
rm -rf .venv
python -m venv .venv
source .venv/bin/activate
pip install -U uv
uv sync
```

## Project Structure

```
robopicker/
├── lerobot/                      # LeRobot fork (submodule)
│   └── src/lerobot/robots/
│       └── so101_mujoco/         # SO-101 robot implementation
├── SO101/                        # Scene files and robot assets
│   ├── pick_scene.xml           # Main scene definition
│   ├── asset_processing.md      # Collision geometry docs
│   └── assets/                  # Robot meshes and collision hulls
├── configs/
│   ├── so101_mujoco_record.yaml # Recording configuration
│   └── cube_positions.json      # Predefined object positions
└── datasets/                     # Local dataset storage
```

## Configuration

Edit `configs/so101_mujoco_record.yaml` to customize:

- **Control frequencies**: `record_fps`, `control_fps`, `physics_fps`
- **Camera settings**: Resolution, camera names, collision geometry visibility
- **Dataset options**: HuggingFace repo, local storage, video encoding
- **Object positions**: Path to predefined cube positions JSON

## Key Features

### Multi-Rate Control Architecture
- **30 Hz**: Dataset recording frequency
- **180 Hz**: Internal control loop (Jacobian, IK, gravity compensation)
- **360 Hz**: MuJoCo physics timestep

All frequencies are exact multiples to prevent timing drift.

### Episode Metadata
Each episode stores custom metadata (object positions, task parameters) in the dataset's parquet files for reproducibility and analysis.

### Predefined Object Positions
Episodes use configured object positions from `configs/cube_positions.json` instead of random placement, ensuring consistent initial conditions.

## Development Progress

**Completed:**
- ✅ SO-101 MuJoCo robot implementation with LeRobot integration
- ✅ Keyboard teleoperation with end-effector control
- ✅ Multi-rate control (30/180/360 Hz)
- ✅ Multi-camera rendering with GLFW visualization
- ✅ Episode metadata storage for object positions
- ✅ Predefined object position system
- ✅ Gripper collision geometry fix via convex decomposition
- ✅ Recording and replay functionality
- ✅ Scene objects (cube, container)

**In Progress:**
- Training policies on collected datasets
- Sim-to-real transfer experiments

## Credits

- **LeRobot**: HuggingFace team - behavior cloning framework
- **SO-101 Model**: [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
- **Collision Fix**: CoACD convex decomposition algorithm

## References

- **Robot Implementation**: `lerobot/src/lerobot/robots/so101_mujoco/README.md`
- **Collision Geometry**: `SO101/asset_processing.md`
