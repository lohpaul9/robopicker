#!/usr/bin/env python3
"""
Test MuJoCo rendering on macOS.
Should open a viewer window with a simple robot arm that you can interact with.

Controls:
- Left mouse: rotate view
- Right mouse: pan view
- Scroll: zoom
- Double-click: select body
- Ctrl+Right-click: apply force
- Press ESC or close window to exit
"""

import mujoco
import mujoco.viewer
import time

# Create a simple 3-DOF robot arm
xml = """
<mujoco model="test_arm">
    <option timestep="0.01"/>

    <visual>
        <global offwidth="1920" offheight="1080"/>
        <quality shadowsize="4096"/>
    </visual>

    <asset>
        <texture name="grid" type="2d" builtin="checker" width="512" height="512"
                 rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
        <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
    </asset>

    <worldbody>
        <light directional="true" diffuse=".8 .8 .8" pos="0 0 10" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="2 2 0.1" material="grid"/>

        <!-- Base -->
        <body name="base" pos="0 0 0.1">
            <geom type="cylinder" size="0.08 0.05" rgba="0.3 0.3 0.3 1"/>
            <geom type="box" size="0.06 0.06 0.05" pos="0 0 0.05" rgba="0.5 0.5 0.5 1"/>

            <!-- Joint 1 (base rotation) -->
            <body name="link1" pos="0 0 0.1">
                <joint name="joint1" type="hinge" axis="0 0 1" range="-180 180" damping="0.1"/>
                <geom type="cylinder" size="0.04 0.15" rgba="0.8 0.2 0.2 1"/>

                <!-- Joint 2 (shoulder) -->
                <body name="link2" pos="0 0 0.15">
                    <joint name="joint2" type="hinge" axis="0 1 0" range="-120 120" damping="0.1"/>
                    <geom type="capsule" size="0.03" fromto="0 0 0 0 0 0.2" rgba="0.2 0.8 0.2 1"/>

                    <!-- Joint 3 (elbow) -->
                    <body name="link3" pos="0 0 0.2">
                        <joint name="joint3" type="hinge" axis="0 1 0" range="-150 150" damping="0.1"/>
                        <geom type="capsule" size="0.025" fromto="0 0 0 0 0 0.15" rgba="0.2 0.2 0.8 1"/>

                        <!-- End effector -->
                        <body name="ee" pos="0 0 0.15">
                            <geom type="sphere" size="0.04" rgba="1 0.8 0.2 1"/>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>

    <actuator>
        <motor name="motor1" joint="joint1" gear="1"/>
        <motor name="motor2" joint="joint2" gear="1"/>
        <motor name="motor3" joint="joint3" gear="1"/>
    </actuator>
</mujoco>
"""

def main():
    print("=" * 60)
    print("🤖 MuJoCo Rendering Test")
    print("=" * 60)
    print("\nCreating MuJoCo model...")

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    print("✅ Model loaded successfully")
    print(f"   - {model.nq} position DOFs")
    print(f"   - {model.nv} velocity DOFs")
    print(f"   - {model.nu} actuators")

    print("\n🎮 Controls:")
    print("   - Left mouse: rotate view")
    print("   - Right mouse: pan view")
    print("   - Scroll: zoom")
    print("   - Double-click: select body")
    print("   - ESC: exit")

    print("\n📺 Opening viewer window...")
    print("   (The arm will swing due to gravity)")
    print("   Close the window or press Ctrl+C to exit\n")

    # Set initial joint positions (bent elbow pose)
    data.qpos[0] = 0.0   # Base rotation
    data.qpos[1] = 0.5   # Shoulder angle
    data.qpos[2] = -1.0  # Elbow angle

    # Launch passive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Simulation loop
        step_count = 0
        while viewer.is_running():
            step_start = time.time()

            # Apply a small torque to joint 1 to make it interesting
            if step_count < 500:  # First 5 seconds
                data.ctrl[0] = 0.1 * (1 if step_count % 200 < 100 else -1)
            else:
                data.ctrl[0] = 0

            # Step simulation
            mujoco.mj_step(model, data)

            # Update viewer
            viewer.sync()

            # Maintain real-time speed
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            step_count += 1

    print("\n✅ Viewer closed successfully")
    print("🎉 MuJoCo rendering works on your macOS!")

if __name__ == "__main__":
    main()
