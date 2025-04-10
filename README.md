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

---

## 🧪 Sim2Sim Deployment (→ Isaac Sim) [WIP]

To decouple from Isaac Lab, a **standalone policy runner script** will be provided to load and run the trained model directly in **Isaac Sim**, using only the USD environment and model weights.

🎯 Goals:
- Run policy inference using Isaac Sim without Isaac Lab dependencies  
- Use the same `.usd` and neural policy exported during training

This is the **first transfer step** before attempting control in physical or ROS-based environments.

---

## 🤖 Sim2Real Deployment (→ ROS) [WIP]

The Sim2Real pipeline will begin with **standalone ROS control of the end-effector** using the **Reach Task** trained policy, without relying on Isaac Sim or Isaac Lab.

🔧 Initial phase:
- Create a lightweight ROS node that loads the trained model  
- Infer target end-effector positions in real-time  
- Send Cartesian or joint commands to the physical Kinova Gen3  
- Use basic teleop (e.g., keyboard arrows) as fallback/manual control

> This approach ensures the runtime system is clean, efficient, and hardware-ready, keeping only the trained model as a dependency.

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