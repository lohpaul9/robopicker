# Understanding gym-hil Wrapper Flow and Action Dimensions

**Date:** 2025-10-04
**Context:** Understanding how keyboard teleoperation works with gym-hil environments for recording training datasets

---

## The Big Picture

```
Keyboard input → Wrapper stack → MuJoCo simulation → Dataset recording
```

The key insight: `gym_manipulator.py` sends "neutral" (zero) actions, but the **InputsControlWrapper** intercepts and **replaces** them with keyboard input when intervention mode is active.

---

## Action Dimensions Explained

### Action Space Transformations

```
Keyboard input → Wrapper processing → MuJoCo environment
     3D              →      7D       →    Robot control
```

### 1. Keyboard Produces 3D Deltas

**Location:** `gym-hil/gym_hil/wrappers/intervention_utils.py:222-239`

```python
delta_x, delta_y, delta_z = self.controller.get_deltas()
# Example: [0.01, 0.0, 0.0] means "move 0.01m forward in X"

gamepad_action = np.array([delta_x, delta_y, delta_z])  # Shape: (3,)

# With gripper:
gamepad_action = np.concatenate([gamepad_action, [1.0]])  # Shape: (4,)
# [dx, dy, dz, gripper_command]
# gripper: 0.0=close, 1.0=hold, 2.0=open
```

**4D Action:** `[x_delta, y_delta, z_delta, gripper_command]`

### 2. EEActionWrapper Converts to 7D

**Location:** `gym-hil/gym_hil/wrappers/hil_wrappers.py:85-103`

```python
def action(self, action):
    # Input: [dx, dy, dz, gripper] (4D)

    action_xyz = action[:3] * self._ee_step_size  # Scale the deltas
    # Example: [0.01, 0, 0] * [0.025, 0.025, 0.025] = [0.00025, 0, 0]

    actions_orn = np.zeros(3)  # No rotation control (yet)

    gripper_open_command = [action[-1] - 1.0]  # Normalize [0,2] → [-1,1]
    # 0.0 → -1.0 (close), 1.0 → 0.0 (hold), 2.0 → 1.0 (open)

    action = np.concatenate([action_xyz, actions_orn, gripper_open_command])
    # Output: [dx, dy, dz, 0, 0, 0, gripper] (7D)
    return action
```

**7D Action:** `[x_delta, y_delta, z_delta, roll_delta, pitch_delta, yaw_delta, gripper_delta]`

Currently rotation deltas (indices 3-5) are always zeros because keyboard doesn't control rotation.

---

## Wrapper Stack Walkthrough

### Wrapper Order (outermost to innermost)

When you create `gym_hil/SO101PickCubeKeyboard-v0`, the wrappers are applied in this order:

```
5. ResetDelayWrapper           (adds delay on reset)
4. PassiveViewerWrapper         (shows MuJoCo window)
3. InputsControlWrapper         ← KEYBOARD OVERRIDE HAPPENS HERE
2. EEActionWrapper              ← 4D → 7D conversion
1. GripperPenaltyWrapper        (penalizes wasteful gripper commands)
0. SO101PickCubeGymEnv (base)   (actual MuJoCo simulation)
```

### Step-by-step Execution Flow

When `env.step(action)` is called, it goes through each wrapper from top to bottom:

#### 5. ResetDelayWrapper
```python
# Only affects reset(), passes step() through unchanged
return self.env.step(action)
```

#### 4. PassiveViewerWrapper
```python
# Shows MuJoCo viewer window
# Passes through to next wrapper
return self.env.step(action)
```

#### 3. InputsControlWrapper ⭐ **THE OVERRIDE**

**Location:** `gym-hil/gym_hil/wrappers/hil_wrappers.py:216-271`

```python
def step(self, action):
    # action parameter = neutral action from gym_manipulator
    # Example: action = [0.0, 0.0, 0.0, 1.0] (no movement, hold gripper)

    # Read keyboard state
    is_intervention, gamepad_action, terminate_episode, success, rerecord = \
        self.get_gamepad_action()

    # gamepad_action = [0.01, 0.0, 0.0, 1.0] if arrow key pressed

    # ═══════════════════════════════════════
    # THE OVERRIDE HAPPENS HERE:
    # ═══════════════════════════════════════
    if is_intervention:  # Space bar toggles this to True
        action = gamepad_action  # REPLACES neutral action!

    # Now action = [0.01, 0.0, 0.0, 1.0] (keyboard input)

    obs, reward, terminated, truncated, info = self.env.step(action)

    # Save intervention metadata
    info["is_intervention"] = is_intervention
    info["action_intervention"] = action  # Saves keyboard action to info

    return obs, reward, terminated, truncated, info
```

**Key Points:**
- Space bar toggles `is_intervention` flag
- When active, neutral action is **completely replaced** with keyboard input
- The actual action used is saved in `info["action_intervention"]`

#### 2. EEActionWrapper

**Location:** `gym-hil/gym_hil/wrappers/hil_wrappers.py:85-103`

```python
def step(self, action):
    # action = [0.01, 0.0, 0.0, 1.0] from InputsControlWrapper

    # Convert to 7D before passing to base env
    action = self.action(action)
    # Now [0.00025, 0.0, 0.0, 0, 0, 0, 0.0]

    return self.env.step(action)
```

#### 1. GripperPenaltyWrapper

**Location:** `gym-hil/gym_hil/wrappers/hil_wrappers.py:40-50`

```python
def step(self, action):
    # action = [0.00025, 0.0, 0.0, 0, 0, 0, 0.0] from EEActionWrapper

    observation, reward, terminated, truncated, info = self.env.step(action)

    # Check if gripper command is wasteful
    # (trying to close when already closed, or open when already open)
    if (action[-1] < -0.5 and self.last_gripper_pos > 0.9) or \
       (action[-1] > 0.5 and self.last_gripper_pos < 0.1):
        info["discrete_penalty"] = -0.05  # Add penalty

    return observation, reward, terminated, truncated, info
```

#### 0. SO101PickCubeGymEnv (Base Environment)

**Location:** `gym-hil/gym_hil/envs/so101_pick_gym_env.py:116-137`

```python
def step(self, action):
    # action = [0.00025, 0.0, 0.0, 0, 0, 0, 0.0]

    # Send to MuJoCo robot controller
    self.apply_action(action)

    # Compute observations and rewards
    obs = self._compute_observation()
    rew = self._compute_reward()
    success = self._is_success()
    terminated = bool(success)

    return obs, rew, terminated, False, {"succeed": success}
```

---

## How gym_manipulator.py Fits In

### The Recording Loop

**Location:** `lerobot/src/lerobot/rl/gym_manipulator.py:644-659`

```python
while episode_idx < cfg.dataset.num_episodes_to_record:
    # 1. Create neutral action (no movement)
    neutral_action = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32)
    if use_gripper:
        neutral_action = torch.cat([neutral_action, torch.tensor([1.0])])
    # neutral_action = [0, 0, 0, 1]

    # 2. Step environment with neutral action
    transition = step_env_and_process_transition(
        env=env,
        action=neutral_action,  # Sends [0, 0, 0, 1]
        env_processor=env_processor,
        action_processor=action_processor,
    )
```

### The Step Function

**Location:** `lerobot/src/lerobot/rl/gym_manipulator.py:508-557`

```python
def step_env_and_process_transition(env, transition, action, ...):
    # action = [0, 0, 0, 1] (neutral)

    # Run through action processor (converts torch → numpy)
    processed_action = action_processor(transition)[TransitionKey.ACTION]
    # processed_action = np.array([0, 0, 0, 1])

    # STEP THE ENVIRONMENT
    obs, reward, terminated, truncated, info = env.step(processed_action)
    #                                           ↑
    #                        This triggers the wrapper stack!

    # Create new transition with results
    new_transition = create_transition(
        observation=obs,
        action=processed_action,
        reward=reward,
        done=terminated,
        truncated=truncated,
        info=info,
        complementary_data=complementary_data,
    )

    return new_transition
```

---

## The Complete Override Flow

### Without Intervention (Space NOT pressed)

```
gym_manipulator.py:     neutral_action = [0, 0, 0, 1]
                              ↓
action_processor:       torch → numpy conversion
                              ↓
env.step():             [0, 0, 0, 1]
                              ↓
┌────────────────────────────────────────┐
│ InputsControlWrapper:                  │
│   is_intervention = False              │
│   keeps action = [0, 0, 0, 1]         │ ← NO OVERRIDE
└────────────────────────────────────────┘
                              ↓
EEActionWrapper:        [0, 0, 0, 0, 0, 0, 0]
                              ↓
GripperPenaltyWrapper:  pass through
                              ↓
SO101PickCubeGymEnv:    robot doesn't move
```

### With Intervention (Space pressed, Arrow Up held)

```
gym_manipulator.py:     neutral_action = [0, 0, 0, 1]
                              ↓
action_processor:       torch → numpy conversion
                              ↓
env.step():             [0, 0, 0, 1]
                              ↓
┌────────────────────────────────────────┐
│ InputsControlWrapper:                  │
│   is_intervention = True               │
│   gamepad_action = [0.01, 0, 0, 1]    │ ← from keyboard
│   action = gamepad_action              │ ← OVERRIDE!
└────────────────────────────────────────┘
                              ↓
EEActionWrapper:        [0.00025, 0, 0, 0, 0, 0, 0]
                              ↓
GripperPenaltyWrapper:  pass through
                              ↓
SO101PickCubeGymEnv:    robot moves forward in X!
```

---

## Recording the Correct Actions

### How the Dataset Gets Keyboard Actions

**Location:** `lerobot/src/lerobot/rl/gym_manipulator.py:669-672`

```python
# Use teleop_action if available, otherwise use the action from the transition
action_to_record = transition[TransitionKey.COMPLEMENTARY_DATA].get(
    "teleop_action", transition[TransitionKey.ACTION]
)
```

The `info["action_intervention"]` from `InputsControlWrapper` gets propagated to `COMPLEMENTARY_DATA` and saved as `teleop_action`.

**Result:** The dataset records the **actual keyboard actions**, not the neutral actions!

---

## Keyboard Controls Reference

From `gym-hil/gym_hil/wrappers/intervention_utils.py:207-215`:

```
Arrow keys:     Move in X-Y plane (Up/Down = X, Left/Right = Y)
Shift:          Move down in Z axis
Shift_R:        Move up in Z axis
O key:          OPEN gripper
C key:          CLOSE gripper
Space:          Toggle intervention mode ON/OFF
Enter:          End episode with SUCCESS
Esc:            End episode with FAILURE
R key:          Mark episode for re-recording
```

---

## Interface Parity: Panda vs SO-101

Both robots follow the **exact same pattern**:

### Registration
```python
# Panda
"gym_hil/PandaPickCubeKeyboard-v0" → make_env(
    env_id="gym_hil/PandaPickCubeBase-v0",
    use_viewer=True,
    gripper_penalty=-0.05
)

# SO-101
"gym_hil/SO101PickCubeKeyboard-v0" → make_env(
    env_id="gym_hil/SO101PickCubeBase-v0",
    use_viewer=True,
    gripper_penalty=-0.05
)
```

### Wrapper Stack
✅ **IDENTICAL** for both:
1. GripperPenaltyWrapper
2. EEActionWrapper
3. InputsControlWrapper
4. PassiveViewerWrapper
5. ResetDelayWrapper

### Action Space
✅ **SAME** for both:
- 7D: `[x, y, z, roll, pitch, yaw, gripper]`
- All deltas in range `[-1, 1]`

### Observation Format
✅ **COMPATIBLE**:
- Panda: `{"pixels": {"front": ..., "wrist": ...}, "agent_pos": ...}`
- SO-101: `{"pixels": {"front": ...}, "agent_pos": ...}`

**Only difference:** Panda has wrist camera, SO-101 doesn't (yet)

---

## Summary

1. **Neutral actions are intentional** - they're a "default" that gets overridden
2. **Override happens in `InputsControlWrapper`** - Space bar toggles intervention mode
3. **Action dimensions transform: 3D → 4D → 7D** through the wrapper stack
4. **The dataset records keyboard actions**, not neutral actions
5. **SO-101 and Panda have identical interfaces** - your implementation is correct!

The wrapper pattern enables Human-in-the-Loop (HIL) teleoperation for data collection while maintaining a clean separation between the policy interface (neutral actions) and human control (keyboard/gamepad).
