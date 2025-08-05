
# ArduPilot Gazebo Plugin

This is a fork of the official [ArduPilot Gazebo plugin](https://github.com/khancyr/ardupilot_gazebo). It has been updated to:
- Support the latest [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install) version. 
- Integrate better with recent ArduPilot changes for our usecase
- Include new features tailored for advanced use cases.

---

## Features added

- Support for additional sensors.
- More worlds.
- Multi-drone support*
- 3D gimbal and camera integration.

---

## 📦 Prerequisites

### Supported Platforms

- **Ubuntu 22.04** (recommended)
- **macOS** (Intel/M1) with limitations (e.g., OpenGL required).
    

### Required Tools

- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install)
- [ArduPilot SITL](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
- [OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)
---

## 🔧 Installation

### 1. Install Gazebo Harmonic (Ubuntu)

#### Option A – Recommended (Manual APT)

```bash
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

#### Option B – Using rosdep

```bash
export GZ_VERSION=harmonic

sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'

rosdep update
rosdep resolve gz-harmonic
# Navigate to your ROS workspace if using catkin/colcon
rosdep install --from-paths src --ignore-src -y
```

---

## ⚙️ Build Instructions

```bash
# Make sure GZ_VERSION is set
export GZ_VERSION=harmonic

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

---

## 🌐 Environment Setup

Add the following lines to your `~/.bashrc` or `~/.zshrc`:

> ✅ If you're using this with the Nebula project, run `make set_envs` is a better option. if not do these

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
```

Reload:
```bash
source ~/.bashrc   # or ~/.zshrc
```


---

## 🚁 Quick Start: Simulate a Quad

### 1. Run Gazebo Simulation

```bash
gz sim -v4 -r iris_runway.sdf
```

### 2. Start ArduPilot SITL

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

### 3. Arm and Takeoff in MAVProxy

```bash
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
```

---

## 🧪 Additional Features

### 📷 Camera Streaming via GStreamer

Enable the camera stream:

```bash
gz topic -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming -m gz.msgs.Boolean -p "data: 1"
```

#### View Stream (OpenCV or GStreamer)

**Option A – GStreamer**

```bash
gst-launch-1.0 -v udpsrc port=5600 \
    caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
    ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

**Option B – Python with OpenCV**

```python
import cv2

pipeline = (
    "udpsrc port=5600 ! "
    "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! "
    "rtph264depay ! "
    "h264parse ! "
    "avdec_h264 ! "
    "videoconvert ! "
    "appsink drop=1"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open stream! Check sender or pipeline.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame.")
        break

    cv2.imshow("Stream", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

### 🎮 Control 3D Gimbal via RC Channels

Run ArduPilot with Gimbal parameters:

```bash
cd ardupilot
sim_vehicle.py -D -v ArduCopter -f JSON --add-param-file=$HOME/ardupilot_gazebo/config/gazebo-iris-gimbal.parm --console --map
```

---

## ⏩ Run Faster Than Real-Time (Optional)

To simulate at high speed, `iris_runway.sdf` includes:

```xml
<physics name="1ms" type="ignore">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>-1.0</real_time_factor>
</physics>
```

---

## 🛠️ Troubleshooting

- Gazebo crash or blank screen? Check for OpenGL support and driver compatibility.
    
- GStreamer stream doesn't show? Ensure UDP port is open, and plugin is configured.
    
- Refer to the official [ArduPilot SITL + Gazebo Guide](https://ardupilot.org/dev/docs/sitl-with-gazebo.html#sitl-with-gazebo)
