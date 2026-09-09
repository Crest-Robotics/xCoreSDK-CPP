# xCore SDK Robot Control Interface

> **Note:** This document was translated from Chinese to English by Claude Code.

xCore SDK is a programming interface library provided by Rokae Robotics for customer secondary development. Through this interface library, customers can perform a range of control operations on the robot.

## Online Documentation

Visit Rokae's official online documentation to learn about other SDK content and the related API descriptions and usage, and to learn about Rokae's other technical products.

https://docs.rokae.com/docs/SDK/cpp

## Control Modes

xCore SDK provides both non-real-time and real-time control of the robot.

### Non-real-time Control

* Motion: mainly sends motion commands with target points in joint space or Cartesian space, using the controller's internal trajectory planning
  * Joint space motion (MoveAbsJ, MoveJ), Cartesian space motion (MoveL, MoveC, MoveCF, MoveSP); set motion speed and blending zone for commands
  * Supports external-axis (track) coordination and configuring track parameters
  * Supports reachability verification, setting acceleration, enabling singularity avoidance, etc.
* Force control commands
* Robot communication
  * Digital and analog I/O, register read/write
  * XMS and XMC model end-effector RS-485 communication
* RL project transfer, query, and execution; tool/workpiece configuration
* Drag teaching and path playback (collaborative robots only)
* Other operations
  * Basic robot pose and status reading
  * Jog (manual robot jogging)
  * Configure collision detection, get collision status
  * Set soft limits, clear alarms, query controller logs, etc.
* Controller upgrade and backup

### Real-time Control

Real-time mode supports control at up to 1 kHz, suitable for algorithm validation and new application development.

* Collaborative robots support 5 control modes:
  * Joint space position control, supports ServoJ
  * Cartesian space position control
  * Joint space impedance control
  * Cartesian space impedance control
  * Direct torque control
* Six-axis industrial robots support 2 position control modes:
  * Joint space position control
  * Cartesian space position control

## Compatibility

### Robot Controller

xCore controller version 3.2.1 or later

### Build Environment

| OS Platform                     |Compiler| Platform            |Language|
|--------------------------|---|---------------|----|
| Ubuntu 18.04/20.04/22.04 |build-essential| x86_64<br/>aarch64 |C++|
| Windows 10/11            |MSVC 14.1+| x86_64        |C++|

## Build

The C++ version of xCore SDK uses CMake to build the project; CMake version 3.12 or later is required.

### Obtaining the Prebuilt Library

This repository contains only headers, examples, and the CMake project — it does **not** include prebuilt libraries (to avoid using up Git LFS quota).

1. Clone this repository
2. Open the [Release page](https://github.com/Crest-Robotics/xCoreSDK-CPP/releases/tag/v0.7.1) matching your SDK version (URL format: `…/releases/tag/v{VERSION}`; the version is in `CMakeLists.txt`; if the library is missing, `cmake` will also print a direct link)
3. Download the library package matching your platform
4. Extract it at the repository root so the files land in the `lib/` directory

See [lib/README.md](lib/README.md) for detailed instructions and the package-name reference.

### Prerequisites

#### Ubuntu

* Install g++ and cmake: `sudo apt install cmake g++`
* On Ubuntu 18.04, the default CMake version is 3.10; you can install the latest CMake using the steps below:

~~~
sudo apt remove --purge --auto-remove cmake
sudo apt update && \
sudo apt install -y software-properties-common lsb-release && \
sudo apt clean all
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
sudo apt update
sudo apt install kitware-archive-keyring
sudo rm /etc/apt/trusted.gpg.d/kitware.gpg
~~~

If `sudo apt update` produces a `NO_PUBKEY` error, run:

~~~
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 6AF7F09730B3F0A4
~~~

Then continue with:

~~~
sudo apt update
sudo apt install cmake
~~~

#### Windows

* Download and install Microsoft Visual Studio, version 2017 or later, with the *Desktop development with C++* workload selected.

### Build Targets

*Note:* Build target names vary depending on the executable name.

* Build
    * *all* (the default if no target is provided)
    * *clean*
    * *sdk_example* - example program
    * *install* - installs executables to *CMAKE_INSTALL_PREFIX*
    * *doc* - generates API documentation, requires Doxygen

### CMake Options

* `CMAKE_INSTALL_PREFIX` - install path
* `CMAKE_BUILD_TYPE` - build type
* `XCORE_LINK_SHARED_LIBS` - whether to link the shared library
* `XCORE_USE_XMATE_MODEL` - whether to use the xMate model library for kinematics and dynamics calculations. Currently supports Linux x86_64 and Windows 64-bit.

## Usage

### Hardware Setup

xCore SDK connects to the robot via Ethernet (TCP/IP). If only using non-real-time control, network requirements are low — wired or wireless connections both work, as long as the workstation PC and the robot are on the same LAN.
If using real-time mode, a direct wired connection to the robot is recommended to ensure network stability.

### Robot Setup

* xCore SDK does not require enabling related features via Robot Assist before use.
* xCoreSDK requires authorized features; if you encounter a "feature not authorized" error, please contact customer support.

### Using the Interface

See *example*. For a more complete description of the interface, error codes, common issues, and usage scenarios, refer to the [online documentation](https://docs.rokae.com/docs/SDK/cpp).

# License

> Copyright (C) 2026 ROKAE (Beijing) Technology Co., LTD.
