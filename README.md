# 🦾 Kinova Gen3 RL & Sim2Real Toolkit

## Overview

This repository provides a modular framework for training reinforcement learning (RL) agents on the **Kinova Gen3** robot using **Isaac Lab**, and deploying trained models either in **Isaac Sim**, **URSim**, or on the **real robot** via ROS.

Built as a standalone Isaac Lab extension, it allows isolated development.

**✨ Features**
- 🧠 Task-specific RL environments for Kinova Gen3  
- 🎯 Reach task implemented as a starting point  
- 🧪 Sim2Sim Deployment (→ Isaac Sim) (WIP)
- 🤖 Sim2Real Deployment (→ ROS) (WIP)

---

## 🛠️ Installation

1. Install Isaac Lab by following the official [installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html) (conda recommended).  
2. Clone this repo **outside** the `IsaacLab` directory.  
3. Install in editable mode:

```bash
python -m pip install -e source/gen3
```

---

## 🚀 Training & Basic Testing

You can train a policy on the Kinova Gen3 **Reach Task** using your preferred RL library:

```bash
python scripts/rsl_rl/train.py --task Gen3-Reach-v0
```

After training, a quick way to validate the behavior is to use `play.py`:

```bash
python scripts/rsl_rl/play.py --task Gen3-Reach-v0 --checkpoint <path_to_checkpoint>
```

This helps confirm that the learned policy performs as expected in **Isaac Lab** before attempting transfer.

<video width="600" controls>
  <source src="medias/training.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

---

## 🧪 Sim2Sim Deployment (→ Isaac Sim) [WIP]

To decouple from Isaac Lab, a **standalone policy runner script** will be provided to load and run the trained model directly in **Isaac Sim**, using only the USD environment and model weights.

🎯 Goals:
- Run policy inference using Isaac Sim without Isaac Lab dependencies  
- Use the same `.usd` and neural policy exported during training

This is the **first transfer step** before attempting control in physical or ROS-based environments.

---

## 🤖 Sim2Real Deployment (→ ROS) [WIP]

The Sim2Real pipeline focuses on deploying trained reinforcement learning policies directly onto the **real Kinova Gen3 robot** using a minimal **ROS2-based interface**, with no dependency on Isaac Lab or Isaac Sim at runtime.

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
python3 scripts/sim2real/run_task.py
```

The next step is to connect the same interface to the **real Kinova Gen3** and execute the learned Reach Task in real-world conditions — using the exact same model and runtime logic validated in simulation. **(WIP)**

---

## 🧹 Code Formatting

```bash
pip install pre-commit
pre-commit run --all-files
```

---

## 🐛 Troubleshooting

**Missing IntelliSense / Pylance Indexing:**

```json
"python.analysis.extraPaths": [
    "<path-to-repo>/source/gen3"
]
```

**Pylance Crashing?** Exclude unused extensions in `.vscode/settings.json`:

```json
"<path-to-isaac-sim>/extscache/omni.anim.*",
"<path-to-isaac-sim>/extscache/omni.services.*"
```

---

## 🌟 Acknowledgements

NEED TO DO

- IsaacLab dev
- Johnsun
- repo de johnsun
- Kinova dev
- Labo Init