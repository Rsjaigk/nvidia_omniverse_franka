# 🤖 Isaac Sim 4.5.0

This branch contains **scene code** compatible with **Isaac Sim 4.5.0** and integrated with **ROS1 Noetic**.  
Use this version for projects that benefit from the improvements in Isaac Sim 4.5.0 while maintaining ROS1 compatibility.

## 🔧 Purpose

This branch includes:

- Franka Emika Panda robot integration using URDF
- ROS 1 Noetic-based TF publishing and control
- Basic pick-and-place logic
- Updated Isaac Sim 4.5.0 scene compatibility

## 💻 System Requirements

> ⚠️ **Use the [Isaac Sim Compatibility Checker](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html#isaac-sim-latest-release)** to verify your system.

### 🖥️ Operating System

- **Ubuntu 20.04 / 22.04**  
  > 🧪 This project was developed and tested on **Ubuntu 20.04**  
- Windows is supported for GUI, but **Docker support is Linux-only**

### ⚙️ Hardware Requirements

| Component | Minimum | Recommended | Ideal |
|----------|---------|-------------|-------|
| CPU      | Intel i7 (7th Gen) / Ryzen 5 | i7 (9th Gen) / Ryzen 7 | i9 / Threadripper |
| Cores    | 4        | 8           | 16+ |
| RAM      | 32 GB    | 64 GB       | 64 GB+ |
| Storage  | 50 GB SSD | 500 GB SSD | 1 TB NVMe |
| GPU      | RTX 3070 | RTX 4080    | RTX Ada 6000 |
| VRAM     | 8 GB     | 16 GB       | 48 GB+

> ⚠️ GPUs **must have RT Cores**. **A100/H100 are not supported**

## 🧰 Driver Requirements

| Platform | Recommended Driver Version |
|----------|-----------------------------|
| Linux    | `535.129.03` or newer |
| Windows  | `537.58` or `537.70` |

> For kernel 6.8.0+ (Ubuntu 22.04.5), use `535.216.01` or newer

## 🛠️ Installation

### Step 1: Install Dependencies

- [ROS1 Noetic (Ubuntu 20.04)](https://wiki.ros.org/noetic/Installation/Ubuntu)
- [Visual Studio Code](https://code.visualstudio.com/download)

### Step 2: Install Isaac Sim 4.5.0 (Standalone)

- Download [Isaac Sim 4.5.0](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone%404.5.0-rc.36%2Brelease.19112.f59b3005.gl.linux-x86_64.release.zip)

```bash
# Download the Isaac Sim package (Linux)
mkdir ~/isaacsim
cd ~/Downloads
unzip "isaac-sim-standalone@4.5.0-rc.36+release.19112.f59b3005.gl.linux-x86_64.release.zip" -d ~/isaacsim
cd ~/isaacsim
./post_install.sh
./isaac-sim.selector.sh
```

> 🔗 Follow the [Isaac Sim Workstation Installation Guide](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_workstation.html) for detailed setup.

## 🚀 How to Run

```bash
# Clone the repository
git clone https://github.com/Rsjaigk/nvidia_omniverse_franka.git
cd nvidia_omniverse_franka

# Switch to this branch
git checkout isaac_sim_4.5.0
```

## 👇 Setup Scene in Isaac Sim

Copy the franka.py script into the Isaac Sim examples:

```bash
cp isaac_franka_4.5.0.py path/to/isaac-sim/exts/standalone_examples/tutorials/
```

Launch the simulation:

```bash
cd path/to/isaac-sim
./python.sh standalone_examples/tutorials/isaac_franka_4.5.0.py
```
> **Note** Replace path/to/isaac-sim with the actual path to your Isaac Sim installation.

## 📂 Related Branches

For full features:  
→ Switch to `main` branch  
For Isaac Sim 4.2.0 support:  
→ Use `isaac_sim_4.2.0` branch
