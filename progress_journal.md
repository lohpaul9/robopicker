```
      ___      
  \_ |^_^| _/  16 DAYS TO TRAIN A ROBOT
     |___|     
      |_|      Building an imitation learning pipeline
     [___]     
               
```

[Sherry](https://github.com/sherrychen1120) and [Paul](https://github.com/lohpaul9) team up to record our experience building a pipeline for training an SO-101 robot arm (pick-and-place) with simulation support, starting from building a simulation environment, an intuitive teleoperation interface, and training various policies on the collected dataset. This is our journey and thought processes along the way.

Find our code [here](https://github.com/lohpaul9/robopicker).

---
```
 ___
|O.O|  Day 7: Camera angles, randomized training data
|___|
```  

We tune the camera angles to ensure that the task is completely executable just by looking at the camera. This will be important for providing good observations to the policy. We choose:
- front view (allows observation of the cube's xy position in relation to the robot)
- top view (allows observation of the robot's height in relation to the cube)
- wrist view (allows observation of whether the gripper is closed correctly around the cube)

We also change data collection configs to ensure that it can take in carefully selected placements of the cube to ensure that we collect a diverse set of data.

We should be all ready to go!

---
```
 ___
|@_@|  Day 6: Our robot has a 'force field'
|___|
```

Things are looking ⬆️ - we now have a robot that can pick things up, and we can record episodes!

However, our robot seems to be applying force on nearby objects even when it's not in contact with them! It's a subtle 'field' just around the robot gripper. When picking up the the cube, it seems to dangle awkwardly as if it's not being held firmly.

After 4-5 hours of digging around, we figure it out by: the collision geometry is not being generated very precisely in mujoco. Turns out that for mesh items, mujoco uses a single convex hull by default, which creates a 'skin' effect. 

Pro tip: use mujoco's passive viewer's different rendering groups to inspect the collision geometry and other fine-grained details.

We fixed this by using a convex decomposition algorithm (CoACD) to decompose the mesh into multiple convex hulls, resulting in a much tighter fit.

After this change, our robot is able to pick up the cube with much better precision.

---
```
 ___
|^_^|  Day 5: We simplify our teleop again!
|___|
```

Remember day 3? Turns out only xyz cartesian control doesn't work perfectly either - we can't angle the arm to pick things up (who knew?). Back to the drawing board.

Looking at the considerations, it has to be (1) intuitive to a human user, and (2) flexible enough to reach all points in the workspace.

We first play with the idea of decoupling the wrist control from the rest of the arm, and letting the wrist only [point down](https://github.com/lohpaul9/lerobot/blob/main/src/lerobot/robots/so101_mujoco/minimal_orient_down.py) (so it can always pick things up). This works pretty well! But we're still stuck with 2 major problems: (1) Pointing down means we can't 'reach' for things, (2) pointing down is actually non-trivial to implement, and requires more hacky jacobian control, tuning, and logic to dampen this near singularities.

What if we go even simpler? Our final design (and one that's finally really usable by an operator) is to have:
(1) Cartesian end-effector control for the wrist position
(2) Direct joint position control for wrist flex and roll

The intuition is essentially giving the user control over where they want to move their wrist in xyz space, and then giving fine-grained control over the wrist flex and roll joints once at that position.

---
```
 ___
|>_<|  Day 4: Our code moves house again
|___|
```

We try plugging this in to lerobot, and realize that gym-hil doesn't play well with recording our pure imitation learning dataset. The problem: It is originally designed for human-in-the-loop interventions, and therefore the code is a lot more complex than the functionality we need (and we would need to make more changes). 

Some snooping reveals that [lerobot_record](https://github.com/huggingface/lerobot/blob/main/src/lerobot/scripts/lerobot_record.py) seems like a much simpler starting point. And most of our logic can be ported over to the Robot class interface.

We do the refactor, and the code now runs in lerobot! There are a few considerations to think about, such as how the recording loop frequency (30hz) interfaces with the mujoco control loop frequency (180hz) which interfaces with the physics loop frequency (540hz). (These were chosen to keep the simulation smooth and realistic.)

---
```
 ___
|o.o|  Day 3: Teleoperation and end-effectors
|___|
```

We wanted to see what it's like building in a simulation environment, and letting users control it. Let's start with the teleoperation interface!

We want to work as quickly as possible with what we have, so a keyboard teleoperator will have to do.

This is most intuitively done using end-effector control (instead of making the user reposition each joint). Things might not be so simple to do this o.0

We end up a rudimentary cartesian (xyz) control interface informed by a keyboard teleoperator, using simple [inverse kinematics](https://github.com/lohpaul9/gym-hil/commit/b28d7d96cecdff405621e6765a5e571e1366de27#diff-f8eba63758843bdf591b73f0b1cbad2bad93eb20280b071c53f30a35122a150e) and jacobian control to move the end-effector.

Sidenote: Why only cartesian? The SO-101 has a 5-DOF body and therefore cannot reach all arbritrary points!

---
```
 ___
|\o/|  Day 1-2: Navigating the codebase for mujoco and SO101
|___|
```

First things first - where should the code live? We dug around with lerobot and its downstream environments, and found working examples for mujoco environments in [gym-hil](https://huggingface.co/docs/lerobot/il_sim). Looks like we can record episodes for imitation learning - let's start with that! We write up a Gym class for our SO-101 robot and implement its interfaces.

Next up, we load the robot from its [assets](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101), and create a mujoco [scene](https://github.com/lohpaul9/gym-hil/commit/b28d7d96cecdff405621e6765a5e571e1366de27#diff-c08e49fac8b3208f4aab757c09737d5334e95c44110516a5683b73af07a6d8db) with our cube. 
