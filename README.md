# 🦾 Isaac Sim 4.2.0

This branch contains **scene code** compatible with **Isaac Sim 4.2.0** and integrated with **ROS1 Noetic**.  
Use this for legacy or environment-specific projects that require Isaac Sim 4.2.0.


## 🔧 Purpose

This branch includes:

-  Franka Panda robot setup via URDF
-  TF-based tracking (via ROS1)
-  Basic pick and place logic
-  ROS 1 Noetic bridge integration


## 💻 System Requirements

> Ensure your system passes the official [Isaac Sim Compatibility Checker](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/installation/requirements.html#isaac-sim-compatibility-checker)

### 🖥️ Operating System

-  Ubuntu 20.04 / 22.04  
  >  This project was developed and tested on **Ubuntu 20.04**  
-  Windows is **not supported** for Isaac Sim containers

### ⚙️ Hardware Requirements

| Component | Minimum | Recommended | Ideal |
|----------|---------|-------------|-------|
| CPU      | Intel i7 (7th Gen) / Ryzen 5 | i7 (9th Gen) / Ryzen 7 | i9 / Ryzen 9 |
| Cores    | 4        | 8           | 16+ |
| RAM      | 32 GB    | 64 GB       | 64 GB+ |
| Storage  | 50 GB SSD | 500 GB SSD | 1 TB NVMe |
| GPU      | RTX 3070 | RTX 4080    | RTX Ada 6000 |
| VRAM     | 8 GB     | 16 GB       | 48 GB+ |

> ⚠️ GPUs **must** have RT Cores (e.g., RTX series). A100/H100 are **not supported**.

---

## 🔧 Driver Requirements

| Platform | Recommended Driver Version |
|----------|-----------------------------|
| Linux    | `535.129.03`                |
| Windows  | `537.58` / `537.70`         |


## 🛠️ Installation

1. Install [ROS1 Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
2. Download and install the [Omniverse Launcher (Linux)](https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage)
3. Through the Omniverse Launcher, install:
   -  Isaac Sim 4.2.0
   -  Nucleus Server
   -  Cache Server
4. Optional (but recommended): Install [Visual Studio Code](https://code.visualstudio.com/download)

> 🔗 Follow the [Isaac Sim Workstation Installation Guide](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/installation/install_workstation.html) for detailed setup.


## 🚀 How to Run

```bash
# Clone the repository
git clone https://github.com/Rsjaigk/nvidia_omniverse_franka.git
cd nvidia_omniverse_franka

# Switch to this branch
git checkout isaac_sim_4.2.0
```

## 👇 Setup Scene in Isaac Sim

Copy the franka.py script into the Isaac Sim examples:

```bash
cp franka.py path/to/isaac-sim/exts/standalone_examples/api/
```

Launch the simulation:

```bash
cd path/to/isaac-sim
./python.sh standalone_examples/api/franka.py
```
> **Note** Replace path/to/isaac-sim with the actual path to your Isaac Sim installation.

## 📂 Related Branches

For full features:  
→ Switch to `main` branch  
For Isaac Sim 4.5.0 support:  
→ Use `isaac_sim_4.5.0` branch

