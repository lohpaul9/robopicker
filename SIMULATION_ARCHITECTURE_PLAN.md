# SO-101 MuJoCo Simulation Recording - Architecture Plan

**Project Goal:** Record pick-and-place demonstrations in MuJoCo simulation with SO-101 robot, integrate with lerobot for training/eval.

---

## Architectural Decisions

### 1. MuJoCo Setup: Gymnasium Wrapper ✓

**Decision:** Wrap MuJoCo simulation in a custom Gym environment

**Rationale:**
- Standard interface that lerobot already understands
- Clean abstraction: simulation details hidden behind `reset()` and `step()`
- Easier to swap between simulation and real hardware later
- Matches existing lerobot patterns (see `gym_manipulator.py`)

**Implementation Details:**

```python
class SO101MuJocoEnv(gym.Env):
    """Custom Gym environment for SO-101 in MuJoCo"""

    def __init__(self,
                 mjcf_path: str,
                 render_mode: str = "human",
                 camera_config: dict = None):
        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None if render_mode == "headless" else mujoco.viewer.launch_passive(...)

        # Setup camera renderer for observations
        self.camera_renderer = mujoco.Renderer(self.model, height=480, width=640)

        # Define spaces
        self.observation_space = gym.spaces.Dict({
            "observation.state": Box(...),  # joint positions
            "observation.images.front": Box(0, 255, (480, 640, 3), dtype=np.uint8)
        })

        self.action_space = Box(...)  # End-effector delta (x, y, z, gripper)

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        # Set initial joint positions
        self.data.qpos[:] = self.initial_qpos
        return self._get_obs(), {}

    def step(self, action):
        # Convert EE delta to joint positions (via IK)
        joint_targets = self._ee_delta_to_joints(action)

        # Set controls and step simulation
        self.data.ctrl[:] = joint_targets
        mujoco.mj_step(self.model, self.data)

        obs = self._get_obs()
        reward = 0.0  # Can add task-specific reward later
        terminated = False
        truncated = False
        info = {}

        return obs, reward, terminated, truncated, info

    def _get_obs(self):
        # Capture camera image
        self.camera_renderer.update_scene(self.data, camera="front_cam")
        image = self.camera_renderer.render()

        # Get joint states
        joint_positions = self.data.qpos[:self.model.nu].copy()

        return {
            "observation.state": joint_positions,
            "observation.images.front": image
        }
```

**Key Files to Create:**
- `robopicker/envs/so101_mujoco_env.py` - Main Gym environment
- `robopicker/assets/so101.xml` - MuJoCo MJCF model (from SO-ARM100 repo)
- `robopicker/envs/so101_kinematics.py` - IK solver for EE control

**Dependencies:**
- `mujoco` - MuJoCo physics engine
- `gymnasium` - Gym interface
- `numpy` - Array operations

---

### 2. Teleoperation: Hybrid with Custom Teleop Interface ✓

**Decision:** Create custom `KeyboardTeleopSim` class that outputs end-effector delta actions

**Rationale:**
- End-effector control is more intuitive than joint control for pick-and-place
- Reuses lerobot's `Teleoperator` base class and factory pattern
- Integrates cleanly with `lerobot_record.py` recording script
- Similar to existing keyboard teleop in lerobot

**Implementation Details:**

```python
from lerobot.teleoperators.teleoperator import Teleoperator

class KeyboardTeleopSim(Teleoperator):
    """Keyboard teleoperation for simulated robot with EE control"""

    def __init__(self,
                 ee_step_size: float = 0.01,  # meters
                 gripper_step_size: float = 0.1):
        self.ee_step_size = ee_step_size
        self.gripper_step_size = gripper_step_size

        # Initialize keyboard listener
        self.keyboard_listener = pynput.keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )

        # Track key states
        self.keys_pressed = set()

        # Action features for lerobot recording
        self.action_features = {
            "dtype": "float32",
            "shape": (4,),  # [dx, dy, dz, gripper]
            "names": ["delta_x", "delta_y", "delta_z", "gripper_cmd"]
        }

    def connect(self):
        self.keyboard_listener.start()

    def disconnect(self):
        self.keyboard_listener.stop()

    def get_action(self) -> dict:
        """Returns end-effector delta action from keyboard input

        Keyboard mapping:
        - W/S: +/- X (forward/backward)
        - A/D: +/- Y (left/right)
        - Q/E: +/- Z (up/down)
        - Space: close gripper
        - Shift: open gripper
        """
        delta = np.zeros(3)

        if 'w' in self.keys_pressed:
            delta[0] += self.ee_step_size
        if 's' in self.keys_pressed:
            delta[0] -= self.ee_step_size
        if 'a' in self.keys_pressed:
            delta[1] += self.ee_step_size
        if 'd' in self.keys_pressed:
            delta[1] -= self.ee_step_size
        if 'q' in self.keys_pressed:
            delta[2] += self.ee_step_size
        if 'e' in self.keys_pressed:
            delta[2] -= self.ee_step_size

        gripper = 0.0  # No change
        if 'space' in self.keys_pressed:
            gripper = 1.0  # Close
        if 'shift' in self.keys_pressed:
            gripper = -1.0  # Open

        return {
            "delta_x": delta[0],
            "delta_y": delta[1],
            "delta_z": delta[2],
            "gripper_cmd": gripper
        }

    def _on_press(self, key):
        try:
            self.keys_pressed.add(key.char)
        except AttributeError:
            self.keys_pressed.add(key.name)

    def _on_release(self, key):
        try:
            self.keys_pressed.discard(key.char)
        except AttributeError:
            self.keys_pressed.discard(key.name)
```

**Integration with lerobot:**

Register in teleop factory:
```python
# In lerobot/teleoperators/__init__.py
from .keyboard_sim import KeyboardTeleopSim

TELEOPERATOR_REGISTRY = {
    ...
    "keyboard_sim": KeyboardTeleopSim,
}
```

**Key Files to Create:**
- `lerobot/src/lerobot/teleoperators/keyboard_sim/teleop_keyboard_sim.py`
- `lerobot/src/lerobot/teleoperators/keyboard_sim/__init__.py`

**Alternative Consideration:**
- Could also use gamepad for analog control (smoother motions)
- Stick with keyboard for MVP, add gamepad support later if needed

---

### 3. Recording Integration: SimRobot Wrapper ✓

**Decision:** Create `MuJocoSimRobot` class that wraps the Gym environment and implements lerobot's `Robot` interface

**Rationale:**
- Allows us to use `lerobot_record.py` script without modification
- Clean abstraction: recording script doesn't know it's simulation
- Reuses all existing recording infrastructure (video encoding, HF dataset format, etc.)
- Minimal code changes to lerobot core

**Implementation Details:**

```python
from lerobot.robots.robot import Robot

class MuJocoSimRobot(Robot):
    """Simulated robot wrapper for MuJoCo that implements Robot interface"""

    def __init__(self,
                 mjcf_path: str,
                 robot_type: str = "so101_sim",
                 fps: int = 30):
        # Create the Gym environment internally
        self.env = SO101MuJocoEnv(mjcf_path=mjcf_path, render_mode="human")

        self.robot_type = robot_type
        self.fps = fps
        self.is_connected = False

        # Define observation features for lerobot dataset
        self.observation_features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (6,),  # 6 joints
                "names": ["joint_0.pos", "joint_1.pos", ..., "gripper.pos"]
            },
            "observation.images.front": {
                "dtype": "video",
                "shape": (480, 640, 3),
                "names": ["height", "width", "channel"]
            }
        }

        # Define action features
        self.action_features = {
            "dtype": "float32",
            "shape": (4,),
            "names": ["delta_x", "delta_y", "delta_z", "gripper_cmd"]
        }

    def connect(self):
        """Initialize the simulation environment"""
        self.env.reset()
        self.is_connected = True

    def disconnect(self):
        """Close the simulation"""
        self.env.close()
        self.is_connected = False

    def get_observation(self) -> dict:
        """Get current observation from simulation

        Returns dict matching observation_features structure
        """
        # Get observation from gym env
        obs, _ = self.env.reset() if not hasattr(self, '_current_obs') else (self._current_obs, {})

        # Convert to lerobot format
        return {
            "observation.state": obs["observation.state"],
            "observation.images.front": obs["observation.images.front"],
            # Add individual joint positions for compatibility
            **{f"joint_{i}.pos": obs["observation.state"][i] for i in range(6)}
        }

    def send_action(self, action: dict) -> dict:
        """Execute action in simulation

        Args:
            action: Dict with keys matching action_features

        Returns:
            The actual action sent (same as input for sim)
        """
        # Convert dict to array for gym env
        action_array = np.array([
            action["delta_x"],
            action["delta_y"],
            action["delta_z"],
            action["gripper_cmd"]
        ])

        # Step simulation
        obs, reward, terminated, truncated, info = self.env.step(action_array)
        self._current_obs = obs

        return action  # In sim, we can execute exactly what was requested
```

**Integration with lerobot:**

Register in robot factory:
```python
# In lerobot/robots/__init__.py
from .mujoco_sim import MuJocoSimRobot

ROBOT_REGISTRY = {
    ...
    "mujoco_so101": MuJocoSimRobot,
}
```

**Configuration for recording:**
```python
@dataclass
class MuJocoSimRobotConfig(RobotConfig):
    type: str = "mujoco_so101"
    mjcf_path: str = "assets/so101.xml"
    fps: int = 30
```

**Usage with lerobot_record.py:**
```bash
lerobot-record \
    --robot.type=mujoco_so101 \
    --robot.mjcf_path=robopicker/assets/so101.xml \
    --teleop.type=keyboard_sim \
    --dataset.repo_id=lohpaul/so101_sim_pickplace \
    --dataset.num_episodes=50 \
    --dataset.single_task="Pick cube and place in box" \
    --dataset.fps=30 \
    --display_data=true
```

**Key Files to Create:**
- `robopicker/robots/mujoco_sim/robot.py` - Main SimRobot class
- `robopicker/robots/mujoco_sim/config.py` - Config dataclass
- `robopicker/robots/mujoco_sim/__init__.py`

**Critical Design Note:**
The SimRobot wraps the Gym environment, but presents the `Robot` interface to lerobot. This is the key that lets us use the recording script unchanged.

```
lerobot_record.py
    ↓ calls
MuJocoSimRobot (implements Robot interface)
    ↓ contains
SO101MuJocoEnv (implements Gym interface)
    ↓ uses
MuJoCo physics engine
```

---

### 4. Camera Setup: Single Front Camera (MVP) ✓

**Decision:** Start with one static front camera at 640x480 @ 30fps

**Rationale:**
- Simplest setup for MVP
- Reduces recording overhead (one video stream instead of multiple)
- Still provides sufficient visual information for basic pick-and-place
- Easy to expand to multi-camera later without architecture changes

**Camera Configuration:**

```xml
<!-- In so101.xml MuJoCo model -->
<mujoco>
    <worldbody>
        <camera name="front_cam"
                pos="0.5 0 0.4"
                euler="0 0.6 3.14159"
                fovy="45"/>
    </worldbody>
</mujoco>
```

**Camera positioning:**
- Position: 0.5m in front, 0.4m high
- Angle: Slightly downward to capture workspace
- FOV: 45 degrees (standard)

**Future Multi-Camera Expansion:**

When ready to add more cameras, just update:

1. MJCF model: Add camera elements
```xml
<camera name="wrist_cam" mode="targetbody" target="ee_link"/>
<camera name="side_cam" pos="0 0.5 0.3"/>
```

2. Gym env observation space:
```python
self.observation_space = gym.spaces.Dict({
    "observation.images.front": Box(...),
    "observation.images.wrist": Box(...),
    "observation.images.side": Box(...),
})
```

3. SimRobot observation features:
```python
self.observation_features = {
    "observation.images.front": {...},
    "observation.images.wrist": {...},
    "observation.images.side": {...},
}
```

No changes needed to recording or training code!

---

### 5. Raw Dataset Recording: Deferred ✓

**Decision:** Implement raw dataset recording AFTER the main pipeline works

**Rationale:**
- Main pipeline (record → train → eval) is the critical path
- Raw recording is for debugging/visualization - nice to have, not essential
- Reduces initial complexity
- Can add incrementally without breaking existing functionality

**When to implement:**
After successfully completing:
1. ✓ Record a simulation episode
2. ✓ Visualize recorded episode
3. ✓ Train a simple policy on simulated data
4. ✓ Evaluate policy in simulation

**What raw recording will provide:**
- Uncompressed PNG sequences (easier to inspect frame-by-frame)
- Metadata JSON with exact recording conditions
- Separate file per episode (easier debugging)
- Side-by-side with standard HF dataset format

**Reference Implementation:**
See commits:
- https://github.com/sherrychen1120/lerobot/commit/897339ac9f88dfe166f4ce6dc0e538dadfbc2737
- https://github.com/sherrychen1120/so101_bench/commit/326153977886a6b7eea6f1d1468d91f28c981e76

---

## Implementation Phases

### Phase 1: MuJoCo + Gym Setup (Weeks 1)
**Goal:** Get SO-101 rendering in MuJoCo, wrapped in Gym env

Tasks:
- [ ] Download MJCF files from SO-ARM100 repo
- [ ] Create basic `SO101MuJocoEnv` with joint control
- [ ] Add front camera rendering
- [ ] Test: Manual stepping of environment, view robot in MuJoCo viewer
- [ ] Verify observation/action spaces match expected format

**Success Criteria:**
```python
env = SO101MuJocoEnv("assets/so101.xml")
obs, _ = env.reset()
for _ in range(100):
    action = env.action_space.sample()
    obs, r, term, trunc, info = env.step(action)
# Robot visible in viewer, moving based on actions
```

---

### Phase 2: Teleoperation (Week 2)
**Goal:** Keyboard control of simulated robot with end-effector control

Tasks:
- [ ] Implement IK solver (or use existing lerobot kinematics)
- [ ] Create `KeyboardTeleopSim` class
- [ ] Integrate EE delta → joint position conversion in Gym env
- [ ] Test: Manual teleoperation, smooth EE movements
- [ ] Tune step sizes and control responsiveness

**Success Criteria:**
```python
teleop = KeyboardTeleopSim()
teleop.connect()
env = SO101MuJocoEnv("assets/so101.xml")
obs, _ = env.reset()

while True:
    action_dict = teleop.get_action()
    action_array = convert_dict_to_array(action_dict)
    obs, r, term, trunc, info = env.step(action_array)
# User can control robot smoothly with keyboard
```

---

### Phase 3: Recording Integration (Week 3)
**Goal:** Record episodes using lerobot_record.py

Tasks:
- [ ] Create `MuJocoSimRobot` class wrapping Gym env
- [ ] Register robot and teleop in lerobot factories
- [ ] Create config classes
- [ ] Test recording single episode
- [ ] Verify dataset format (HF dataset structure)
- [ ] Test playback with lerobot visualization tools

**Success Criteria:**
```bash
lerobot-record \
    --robot.type=mujoco_so101 \
    --robot.mjcf_path=assets/so101.xml \
    --teleop.type=keyboard_sim \
    --dataset.repo_id=test/sim_recording \
    --dataset.num_episodes=1 \
    --dataset.single_task="Test recording"

# Records successfully, creates HF dataset
# Can visualize with: lerobot-dataset-viz --repo-id=test/sim_recording
```

---

### Phase 4: Dataset Validation (Week 4)
**Goal:** Ensure recorded data is usable for training

Tasks:
- [ ] Record 10-20 episodes of simple pick-and-place
- [ ] Visualize episodes to check quality
- [ ] Verify action/observation dimensions
- [ ] Check for any data corruption or missing frames
- [ ] Compute dataset statistics (for normalization)

**Success Criteria:**
- All episodes play back correctly
- Actions are smooth (no discontinuities)
- Camera images are clear and properly synchronized
- Dataset metadata is complete

---

### Phase 5: Training Pipeline (Week 5)
**Goal:** Train a simple policy on simulated data

Tasks:
- [ ] Choose policy architecture (ACT, Diffusion Policy, etc.)
- [ ] Configure training script for simulated data
- [ ] Run training for small number of epochs
- [ ] Verify policy can load and output actions
- [ ] Check for dimension mismatches or errors

**Success Criteria:**
```bash
lerobot-train \
    --dataset.repo_id=lohpaul/so101_sim_pickplace \
    --policy.type=act \
    --training.num_epochs=100

# Training runs without errors
# Policy checkpoint saved
```

---

### Phase 6: Evaluation Pipeline (Week 6)
**Goal:** Evaluate trained policy in simulation

Tasks:
- [ ] Create evaluation script (may need custom script for sim)
- [ ] Load trained policy
- [ ] Run policy in simulation environment
- [ ] Collect success metrics
- [ ] Visualize policy behavior

**Success Criteria:**
```bash
lerobot-eval \
    --policy.path=outputs/act_so101/checkpoints/last.ckpt \
    --env.type=mujoco_so101 \
    --num_episodes=10

# Policy executes in simulation
# Success rate computed
```

---

## Key Files Structure

```
robopicker/
├── assets/
│   └── so101.xml                          # MuJoCo model
├── envs/
│   ├── __init__.py
│   ├── so101_mujoco_env.py               # Gym environment
│   └── so101_kinematics.py               # IK solver
├── robots/
│   └── mujoco_sim/
│       ├── __init__.py
│       ├── robot.py                       # MuJocoSimRobot class
│       └── config.py                      # Config dataclass
├── teleoperators/
│   └── keyboard_sim/
│       ├── __init__.py
│       └── teleop_keyboard_sim.py        # KeyboardTeleopSim class
└── scripts/
    ├── test_env.py                        # Standalone env test
    ├── test_teleop.py                     # Standalone teleop test
    └── eval_sim.py                        # Sim-specific eval script

lerobot/  (submodule - minimal changes)
├── src/lerobot/
│   ├── robots/__init__.py                # Register mujoco_so101
│   └── teleoperators/__init__.py         # Register keyboard_sim
```

---

## Critical Design Principles

### 1. Separation of Concerns
- **Gym Env**: Only handles MuJoCo physics and rendering
- **Teleop**: Only handles input → action mapping
- **SimRobot**: Only handles Robot interface translation
- **Recording**: Unchanged lerobot script

### 2. Minimal lerobot Modifications
- Only add new robot/teleop types to registries
- Don't modify core recording/training logic
- Makes updates easier, reduces merge conflicts

### 3. Sim-to-Real Transfer Preparation
- Use same action/observation format as real robot
- Record in same dataset format
- Same training pipeline
- Only swap `robot.type` when moving to hardware

### 4. Incremental Development
- Each phase builds on previous
- Test thoroughly before moving forward
- Don't implement "nice-to-haves" until core works

---

## Known Challenges & Solutions

### Challenge 1: IK Solver Performance
**Issue:** Real-time IK for EE control may be slow

**Solutions:**
- Use fast analytical IK if available for SO-101 kinematics
- Cache recent solutions as initial guess
- Fall back to numerical IK (scipy, PyKDL) if needed
- Consider simplified 3-DOF problem (ignore wrist orientation)

### Challenge 2: macOS OpenGL Backend
**Issue:** MuJoCo rendering issues on macOS

**Solutions:**
- Set `MUJOCO_GL=glfw` (should work on macOS)
- If issues persist, try `MUJOCO_GL=osmesa`
- Test on minimal example first before full integration

### Challenge 3: Action Space Mismatch
**Issue:** Teleop outputs EE deltas, but need joint positions for MuJoCo

**Solutions:**
- Implement full pipeline: delta_EE → IK → joint_pos → mujoco.ctrl
- Maintain current EE pose state
- Apply delta to get target EE pose
- Solve IK to get joint targets

### Challenge 4: Dataset Format Compatibility
**Issue:** Simulation data format might not match hardware expectations

**Solutions:**
- Define features explicitly in both sim and real robot configs
- Use lerobot's feature validation
- Test with existing lerobot datasets as reference
- Document any simulation-specific quirks

---

## Dependencies

### Core
- `mujoco>=3.0.0` - Physics simulation
- `gymnasium>=0.29.0` - Environment interface
- `numpy>=1.24.0` - Array operations

### Input Handling
- `pynput>=1.7.6` - Keyboard listener

### Kinematics
- `scipy>=1.10.0` - Numerical IK (if needed)
- OR use lerobot's existing `RobotKinematics` class

### Visualization
- `opencv-python>=4.8.0` - Image handling (may already be in lerobot)
- `rerun-sdk>=0.15.0` - 3D visualization (optional, for debugging)

### Install
```bash
cd robopicker
pip install -e .
pip install -e "lerobot[hilserl]"  # Includes all lerobot recording deps
```

---

## Testing Strategy

### Unit Tests
- `test_so101_env.py`: Gym environment reset/step
- `test_keyboard_teleop.py`: Action generation
- `test_sim_robot.py`: Robot interface compliance

### Integration Tests
- `test_recording.py`: End-to-end recording
- `test_playback.py`: Dataset visualization
- `test_training.py`: Policy training (smoke test)

### Manual Tests
- Teleop responsiveness (feel-based)
- Visual quality of camera images
- Smoothness of robot motions
- Dataset playback fidelity

---

## Success Metrics

### Phase 1-3 (Recording)
- ✅ Record 50 episodes without crashes
- ✅ All episodes playback correctly
- ✅ Average episode length > 10 seconds
- ✅ Camera FPS = 30 (no frame drops)

### Phase 4-6 (Training/Eval)
- ✅ Training completes without errors
- ✅ Policy loads and runs in simulation
- ✅ Policy success rate > random baseline
- ✅ End-to-end latency < 100ms

---

## Future Extensions (Post-MVP)

### Multi-Camera Support
- Add wrist camera for fine manipulation
- Add overhead camera for spatial context
- Sync all camera streams

### Raw Dataset Recording
- Implement `RawDatasetRecorder` alongside standard recording
- Save uncompressed images + metadata
- Tools for raw → standard conversion

### Domain Randomization
- Randomize object positions, colors
- Randomize lighting, camera angles
- Improve sim-to-real transfer

### Advanced Teleoperation
- Gamepad support (analog control)
- 3D mouse for 6-DOF control
- VR headset teleop

### Task Variations
- Multiple object types
- Different placement targets
- Obstacle avoidance

---

## Open Questions

1. **IK Solver Choice**: Use lerobot's `RobotKinematics` or implement custom for SO-101?
   - Decision: Try lerobot's first, implement custom if needed

2. **Gripper Simulation**: How to model parallel-jaw gripper in MuJoCo?
   - Decision: Check SO-ARM100 MJCF for existing gripper model

3. **Action Frequency**: Should we match real robot FPS (30?) or run faster in sim?
   - Decision: Match real robot (30 FPS) for consistency

4. **Observation Space**: Include velocity observations?
   - Decision: Start with position-only, add velocity if policies need it

5. **Dataset Location**: Local filesystem or push to HuggingFace Hub?
   - Decision: Start local, push to Hub once we have quality data

---

## References

- [SO-ARM100 Simulation Files](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation)
- [LeRobot Recording Script](lerobot/src/lerobot/scripts/lerobot_record.py)
- [LeRobot Robot Interface](lerobot/src/lerobot/robots/robot.py)
- [LeRobot Gym Manipulator](lerobot/src/lerobot/rl/gym_manipulator.py)
- [Raw Dataset Recording Commit](https://github.com/sherrychen1120/lerobot/commit/897339ac9f88dfe166f4ce6dc0e538dadfbc2737)

---

**Last Updated:** 2025-10-03
**Status:** Planning Phase
**Next Action:** Begin Phase 1 - MuJoCo + Gym Setup
