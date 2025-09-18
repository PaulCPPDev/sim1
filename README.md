# sim1 – Robotics Simulation Engine

**sim1** is a lightweight robotics simulation engine written in [Rust](https://www.rust-lang.org/) using the [Bevy](https://bevyengine.org/) game engine.  
It allows loading robot descriptions from [URDF](http://wiki.ros.org/urdf) files and visualizing/manipulating them in a simulated environment.  

---

## ✨ Features
- Load robots from **URDF files** (`--urdf=path/to/file.urdf`)  
- Visualize robot links and joints in a **2D/3D Bevy-powered scene**  
- Define end effectors and kinematic chains  
- Extensible architecture for physics, controls, and planning  
- Cross-platform (Linux, macOS, Windows) — runs on Vulkan/Metal/DirectX backends  

---

## 📦 Installation

### Prerequisites
- [Rust](https://www.rust-lang.org/tools/install) (via `rustup`)  
- System dependencies (Linux/Ubuntu example):
  ```bash
  sudo apt update
  sudo apt install build-essential pkg-config \
       libx11-dev libasound2-dev libudev-dev \
       libwayland-dev libxkbcommon-dev
  ```
- For VirtualBox or headless environments:
  ```bash
  sudo apt install mesa-vulkan-drivers vulkan-tools
  ```

### Build
Clone and build:

```bash
git clone https://github.com/yourusername/sim1.git
cd sim1
cargo build --release
```

---

## 🚀 Usage

Run the simulator with a URDF model:

```bash
cargo run -- --urdf="urdf/rr_arm.urdf"
```

Optional flags:
- `--urdf=<path>` : path to the robot URDF file  
- `--end-effector=<link>` : specify the end-effector link name  

---

## 🖥️ Running in a VM

If you’re running on **VirtualBox**, hardware acceleration is limited to OpenGL 4.1, which is not sufficient for Bevy’s compute shaders.  
Use the Vulkan software backend (llvmpipe) instead:

```bash
# (Optional) Nudge selection toward llvmpipe explicitly
export MESA_VK_DEVICE_SELECT=llvmpipe

# Tell wgpu/Bevy to use Vulkan
export WGPU_BACKEND=vulkan

# Run your program
cargo run -- --urdf="urdf/rr_arm.urdf"
```

For best performance, run on bare metal Linux or with proper GPU passthrough (QEMU/KVM recommended).  

---

## 📂 Project Structure
```
sim1/
├── src/
│   └── main.rs         # entry point, Bevy app setup
├── urdf/               # sample URDF files
├── Cargo.toml          # project dependencies
└── README.md           # you are here
```

---

## 🛠️ Roadmap
- ✅ URDF parsing  
- ✅ Bevy-based visualization  
- ⏳ Physics engine integration  
- ⏳ Interactive control & trajectory execution  
- ⏳ ROS2 bridge  

---

## 🤝 Contributing
Contributions, issues, and feature requests are welcome!  
Feel free to open a PR or start a discussion.  

---

## 📜 License
MIT License — see [LICENSE](LICENSE) for details.  
