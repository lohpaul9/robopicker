# RoboPicker

SO-101 robotic arm simulation with IK-based Cartesian control for teleoperation and LeRobot integration.

## Quick Start

```bash
# Install dependencies
uv sync
source .venv/bin/activate

# Run teleoperation
MUJOCO_GL=glfw mjpython gym-hil/gym_hil/so101/scripts/so101_teleop_continuous.py

# Run validation test
MUJOCO_GL=glfw mjpython gym-hil/gym_hil/so101/tests/test_so101_teleop.py
```

**Keyboard Controls:**
- Arrow keys: Move in X-Y plane
- Shift/Shift_R: Move in Z axis
- O/C keys: Open/close gripper
- Enter/Backspace: Success/failure
- Space: Toggle intervention

## What's Working

✅ **IK-based Cartesian Control** - Position-delta based teleoperation with excellent stability
- X-axis movement: 100% accuracy
- Position holding: <3mm drift
- Orthogonal stability: <3.5mm drift in Y/Z during X motion

## Technical Details

**IK Control Approach:**
1. Actions are position deltas that accumulate to a target position
2. Levenberg-Marquardt IK computes target joint angles
3. Joint-space PD control reaches those angles
4. Zero-action transitions lock target to current position (prevents drift)

**Why IK instead of Operational Space Control:**
- Panda's operational space controller moves SO-101 backwards (incompatible task-space inertia matrix)
- IK-based approach avoids problematic matrices and achieves 100% accuracy

## Installation

**Prerequisites:**
- Python 3.10
- [uv](https://github.com/astral-sh/uv) package manager

**Setup:**
```bash
# Clone with submodules
git clone --recursive https://github.com/lohpaul9/robopicker.git
cd robopicker

# Install
uv sync
source .venv/bin/activate
```

**macOS mjpython fix:**
```bash
mkdir -p .venv/lib
ln -sf ~/.local/share/uv/python/cpython-3.10.*/lib/libpython3.10.dylib .venv/lib/
```
See [MuJoCo #1923](https://github.com/google-deepmind/mujoco/issues/1923)

## Project Structure

- `gym-hil/` - Gym-HIL submodule (editable install)
  - `gym_hil/so101/` - SO-101 environments and scripts
  - `gym_hil/controllers/` - IK control implementation
  - `gym_hil/assets/SO101/` - Robot model
- `pyproject.toml` - Main dependencies

## Credits

- IK implementation: [Basic IK in MuJoCo](https://alefram.github.io/posts/Basic-inverse-kinematics-in-Mujoco)
- Base gym environments: HuggingFace LeRobot team
- SO-101 model: [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
