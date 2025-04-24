# 🦾 Kinova Gen3 RL & Sim2Real Toolkit

## Overview

This repository provides a modular framework for training reinforcement learning (RL) agents on the **Kinova Gen3** robot using **Isaac Lab**, and deploying trained models either in **Isaac Sim**, or on the **real robot** via ROS2.

Built as a standalone Isaac Lab extension, it allows isolated development.

> [!IMPORTANT]
> The Sim2Real pipeline runs entirely with **ROS2** and does **not require Isaac Lab or Isaac Sim** to be installed. And for convenience, **pre-trained models are provided** so you can get started immediately without training.


| Section | What you’ll find |
|---------|------------------|
| **🛠️ Installation** | How to install Isaac Lab, clone this repo, and install the Python package. |
| **🚀 Training & Basic Testing** | Commands to train a reach-task policy with `rsl_rl` or `rl_games` and replay it in Isaac Lab. |
| **🧪 Sim-to-Sim Deployment (Isaac Sim)** *(WIP)* | Plan for running the exported `.usd` scene and `policy.pt` directly in Isaac Sim with zero Isaac Lab dependencies. |
| **🤖 Sim-to-Real Deployment (ROS 2)** | Step-by-step instructions to test on fake hardware, then execute the exact same policy on the physical Kinova Gen3. |
| **🌟 Acknowledgements** | Credits to Isaac Lab, Kinova, community contributors, and INIT Lab. |


## 🛠️ Installation

1. Install Isaac Lab by following the official [installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html) (conda recommended).  
2. Clone this repo **outside** the `IsaacLab` directory.  
3. Install from the repository:

```bash
python -m pip install -e source/gen3
```

## 🚀 Training & Basic Testing

You can train a policy on the Kinova Gen3 **Reach Task** using either `rsl_rl` or `rl_games` library:

```bash
python scripts/rsl_rl/train.py --task Gen3-Reach-v0
```

After training, a quick way to validate the behavior is to use `play.py`:

```bash
python scripts/rsl_rl/play.py --task Gen3-Reach-v0
```

This helps confirm that the learned policy performs as expected in **Isaac Lab** before attempting transfer.

<video width="600" controls>
  <source src="medias/training.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## 🧪 Sim2Sim Deployment (Isaac Sim) [WIP]

To decouple from Isaac Lab, a **standalone policy runner script** will be provided to load and run the trained model directly in **Isaac Sim**, using only the USD environment and model weights.

🎯 Goals:
- Run policy inference using Isaac Sim without Isaac Lab dependencies  
- Use the same `.usd` and neural policy exported during training

This is the **first transfer step** before attempting control in physical or ROS-based environments.

## 🤖 Sim2Real Deployment (ROS2)

The Sim2Real pipeline focuses on deploying trained reinforcement learning policies directly onto the **real Kinova Gen3 robot** using a minimal **ROS2-based interface**, with no dependency on Isaac Lab or Isaac Sim at runtime.

### Fake-hardware test

You can first try to simulate the Kinova Gen3 in ROS2 using fake hardware mode:

```bash
ros2 launch kortex_bringup gen3.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true
```

To test movement commands, send a simple joint trajectory:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  points: [
    { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 1 } },
  ]
}" -1
```

Run the Reach Task with the trained policy to go to a certain predefined position:

```bash
python3 scripts/sim2real/run_task_reach.py
```

### Real robot test

The next step is to connect the same interface to the **real Kinova Gen3** and execute the learned Reach Task in real-world conditions — using the exact same model and runtime logic validated in simulation.

For that you'll run::

```bash
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=192.168.1.10
```

And in the same way as simulation, in another terminal:

```bash
python3 scripts/sim2real/run_task_reach.py
```

Now your robot should be alternating between three positions.

## 🌟 Acknowledgements

* Isaac Lab team & contributors
  * [Isaac Lab repository](https://github.com/isaac-sim/IsaacLab)
  * [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/main/index.html)
* Johnson Sun
  * [GitHub profile](https://github.com/j3soon)
  * [UR10 Reacher RL sim2real Isaac Gym env repository](https://github.com/j3soon/OmniIsaacGymEnvs-UR10Reacher) 
* Kinova Robotics team & the ros2_kortex contributors
  * [ros2_kortex repository](https://github.com/Kinovarobotics/ros2_kortex)
* INIT Lab
  * [Website](https://initrobots.ca/)