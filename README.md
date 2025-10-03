# RoboPicker - SO-101 MuJoCo Simulation

Pick and place demonstrations with SO-101 robot in MuJoCo simulation, integrated with LeRobot for training and evaluation.

## 📋 Prerequisites

- Python 3.10 (required)
- [uv](https://github.com/astral-sh/uv) package manager
- macOS (tested on macOS, should work on Linux)

## 🚀 Quick Start

```bash
# Install dependencies
uv sync

# Activate environment
source .venv/bin/activate

# macOS: Set rendering backend (add to ~/.zshrc)
export MUJOCO_GL=glfw

# Test MuJoCo rendering
mjpython robopicker/tests/test_mujoco_render.py
```

## 📁 Project Structure

```
robopicker/
├── assets/              # MuJoCo MJCF models (to be added)
├── envs/                # Gymnasium environments
├── robots/              # Robot interface wrappers
├── teleoperators/       # Teleoperation interfaces
├── scripts/             # Standalone test scripts
└── tests/               # Unit tests
```

## Dependencies

### Philosophy
Our SO-101 simulation is **orthogonal to gym-hil** (which is for Franka). We only include what LeRobot doesn't provide:

### Installation

**Using uv native dependency management:**
```bash
# Install all dependencies (automatic via setup script)
uv sync

# Add new dependencies
uv add <package-name>

# Remove dependencies
uv remove <package-name>
```

⚠️ **Always use `uv` commands, NOT `uv pip`** - we use uv's native mode for better dependency resolution and lockfile management.

**First-time setup:**
```bash
uv sync  # Creates .venv, installs deps, fixes macOS dylib automatically
source .venv/bin/activate
```

## 🔧 macOS-Specific Setup

### mjpython dylib Fix

**Issue:** `mjpython` (MuJoCo's interactive viewer) fails on macOS with uv virtual environments:
```
Library not loaded: @executable_path/../lib/libpython3.10.dylib
```

**Solution:** Create symlink to Python dylib in venv (see [issue #1923](https://github.com/google-deepmind/mujoco/issues/1923)):
```bash
mkdir -p .venv/lib
ln -sf ~/.local/share/uv/python/cpython-3.10.18-macos-aarch64-none/lib/libpython3.10.dylib \
       .venv/lib/libpython3.10.dylib
```

This is automatically applied by `./setup_env.sh`.

**Why this works:** mjpython looks for the Python library at a relative path from the executable. uv stores Python in `~/.local/share/uv/python/...` but the venv only has symlinks to binaries. Creating this symlink makes the dylib available where mjpython expects it.

## 🎯 Development Roadmap

See [SIMULATION_ARCHITECTURE_PLAN.md](SIMULATION_ARCHITECTURE_PLAN.md) for detailed architecture.

### Current Phase: Phase 1 - MuJoCo + Gym Setup

- [x] Setup dependencies with uv
- [x] Verify MuJoCo rendering on macOS
- [ ] Download SO-101 MJCF files
- [ ] Create SO101MuJocoEnv Gym environment
- [ ] Add camera rendering
- [ ] Test manual environment stepping

## 📚 References

- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [SO-ARM100 Simulation Files](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation)
- [MuJoCo mjpython Issue](https://github.com/google-deepmind/mujoco/issues/1923)
