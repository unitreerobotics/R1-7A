<div align="center">

[简体中文](README.md) | **English**

# R1-7A

**A 7-DoF robotic arm teleoperation framework powered by PICO / XRoboToolkit**

Single-arm / dual-arm teleoperation · Multi-camera capture · MCAP recording and replay

[Quick Setup](#-environment-setup) · [Teleoperation](#-teleoperation) · [Data Collection](#-data-collection) · [Detailed Guide (Chinese)](detail.md)

</div>

| Dual-arm Teleoperation · 01 | Dual-arm Teleoperation · 02 |
|:---:|:---:|
| ![Dual-arm teleoperation demo 1](mp4/demo-1.gif) | ![Dual-arm teleoperation demo 2](mp4/demo-2.gif) |

## ✳️ Overview

The R1-7A teleoperation framework uses two PICO controllers to operate 7-DoF robotic arms and Dex1 grippers. It supports single-arm and dual-arm operation, along with camera and joint data collection.

- 🎮 **Teleoperation**: relative controller pose tracking with independent control of each arm and gripper.
- 📦 **Data Collection**: MCAP recordings of control targets, measured joint states, and multiple camera streams.
- 🔄 **Trajectory Replay**: replay on real hardware or inspect recordings in MuJoCo.

`Hardware Preparation` → `Environment Setup` → `Device Calibration` → `Teleoperation & Recording` → `Trajectory Replay`

## 🔧 Hardware Preparation

<p align="center">
  <img src="mp4/微信图片_20261022175307_8_2.jpg" alt="R1-7A robotic arms, Dex1 grippers, and cameras" width="800">
</p>

<p align="center">
  <img src="mp4/硬件连线.png" alt="R1-7A hardware wiring diagram: PC, USB-to-RS485 module, arm power module, and gripper connections; labels in Chinese" width="800">
</p>

**[4-Channel USB-to-RS485 Module](https://support.unitree.com/home/en/Motor_SDK_Dev_Guide/4-channel_485_to_USB_module)**

| Device | Description |
|---|---|
| R1-7A 7-DoF robotic arm | One arm for single-arm operation or two for dual-arm operation; each uses its own serial port and calibration |
| Dex1 gripper | Independent left/right control; can be disabled when not installed |
| PICO headset and two controllers | Runs the XRoboToolkit client |
| Linux PC | Ubuntu 22.04, x86_64, with Git and Conda installed |
| 4-channel USB-to-RS485 module | Connects the PC, arms, and grippers through separate channels |
| Cameras (optional) | RealSense or V4L2 / UVC cameras for image collection |

> Before operating real hardware, secure the arm bases, support the arms, and keep the emergency stop accessible and the homing paths clear. Zero-gain probing does not hold the arms in place. Dual-arm collision planning is not provided.

## 📥 Environment Setup

### 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/unitreerobotics/R1-7A.git
cd R1-7A
```

### 2. Install Dependencies

Install all project dependencies and build the required components with one command:

```bash
bash setup.sh
```

### 3. Configure PICO

1. Install and start [XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) on the PC.
2. Install the [XRoboToolkit client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client) on PICO and connect to the PC's local network IP address.
3. Keep the app in the foreground, enable controller data transmission, and move the controllers to confirm that data updates continuously.

For installation options, dependency versions, and troubleshooting, see the [detailed installation guide (Chinese)](detail.md#-环境安装).

## ⚙️ Device Configuration and Calibration

Before the first run, configure the serial ports, the calibration files for both arms, and the gripper endpoints. Save the machine-specific settings in `.env`. See the [configuration and calibration guide (Chinese)](detail.md#-设备配置与标定) for a complete example.

Reference wiring for the four-channel FTDI adapter:

| Right Gripper | Right Arm | Left Gripper | Left Arm |
|:---:|:---:|:---:|:---:|
| if00 | if01 | if02 | if03 |

Verify the adapter serial number and physical wiring, and use the correct calibration file for each arm. Existing calibration files can be reused if they match the physical devices and their current encoder references. The setup script does not capture zero positions automatically.

**Load the environment from the repository root in every terminal:**

```bash
conda activate r1_teleop
source .env
source scripts/real_env.sh
```

`r1_teleop` is the Conda environment name. The `r1arm` names in commands are the existing code paths.

## 🛠️ Teleoperation

### 1. Start the Gripper Services

After loading the environment in terminal 1, run the following command and keep the terminal open:

```bash
bash dual/start_grippers.sh
```

Confirm that both sides report `Detected motors` and `Gripper Server started`.

### 2. Start Dual-arm Teleoperation

Load the same environment in terminal 2, then run:

```bash
python dual/teleop.py --motion-mode --no-cameras
```

Check the serial ports, calibration files, and startup messages before typing `MOVE`. Both arms move smoothly to home by default. Once homing is complete, hold the corresponding controller's grip button to start tracking.

For right-arm-only operation, use these commands in terminals 1 and 2 instead:

```bash
# Terminal 1: right gripper
bash scripts/start_gripper.sh

# Terminal 2: right-arm teleoperation
python scripts/teleop_r1arm_real.py --motion-mode
```

If no grippers are installed, skip the gripper services and add `--no-enable-dex1` to the teleoperation command.

### 3. Controller Buttons

| Input | Behavior |
|---|---|
| Left / right grip | Hold to track the corresponding controller's relative pose |
| Left / right trigger | Controls the corresponding gripper's closing ratio |
| A (right controller) | Return to home; both arms home together in dual-arm mode |
| X (left controller) | Start recording / stop and save |
| B (right controller) | Pause / resume recording and tracking during an active recording |
| Ctrl+C | Stop and exit; continue supporting the arms after exit |

The single-arm teleoperation entry point does not record data; use the single-arm recording entry point below to enable X/B recording controls. The dual-arm teleoperation entry point also supports recording; no episode is saved until X is pressed.

## 📦 Data Collection

Exit any existing teleoperation program and keep the gripper services running. The recording programs include teleoperation.

```bash
# Dual-arm recording with three cameras by default
python dual/record.py --motion-mode --data-root data/dual_real

# Right-arm recording with two cameras by default
python scripts/record_r1arm_real.py --motion-mode \
  --output-dir data/right_real --require-all-cameras
```

Press X to start / save and B to pause / resume. Output files are named `episode_0000.mcap`, etc. The target control rate is 90 Hz and the target image observation rate is 30 Hz.

For camera configuration and verification commands, see the [image collection guide (Chinese)](detail.md#2-启用图像). To record without images:

```bash
python dual/record.py --motion-mode --no-cameras --data-root data/dual_real
```

## 🔄 Trajectory Replay

Exit teleoperation and recording programs first. A read-only check does not drive the motors:

```bash
python dual/replay.py --episode data/dual_real/episode_0000.mcap
```

After checking the data and device configuration, keep the gripper services running and replay on real hardware:

```bash
python dual/replay.py --episode data/dual_real/episode_0000.mcap --motion-mode
```

Replay does not require PICO. For single-arm replay, slower playback, and image previews, see the [detailed replay guide (Chinese)](detail.md#-轨迹回放).

## 🧪 Simulation

Run dual-arm simulation with mock input, without connecting to real arms:

```bash
python scripts/teleop_r1arm_sim.py --mock-xr --use-dual-arm
```

Remove `--mock-xr` to test real PICO input.

## 🧑‍✈️ Key Parameters

| Parameter | Default |
|---|---|
| Target control rate | 90 Hz |
| Motor kp / kd | 60 / 10 |
| Maximum joint speed | 1 rad/s |
| Controller translation scale | 1.0 |
| Home | `0,1.2,0,-1.2,0,0,0` rad |

Use `--help` on each entry point for the full parameter list. Device configuration, calibration, data formats, and troubleshooting are covered in [detail.md (Chinese)](detail.md).

## 🧠 Training

## 🚀 Deployment

## 🙏 Acknowledgments

Thanks to [XRoboToolkit](https://github.com/XR-Robotics/XRoboToolkit-PC-Service), [Placo](https://github.com/Rhoban/placo), [MuJoCo](https://github.com/google-deepmind/mujoco), and the related Unitree SDK projects.

The page structure is inspired by [UniArmL1](https://github.com/unitreerobotics/UniArmL1). Third-party dependencies retain their original licenses. This repository does not currently include a root-level `LICENSE` file.
