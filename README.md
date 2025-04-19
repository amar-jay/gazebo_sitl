# ArduPilot Gazebo Plugin

This is a fork of the Ardupilot Gazebo plugin. We made a couple of changes to the original plugin to incoperate recent changes in Gazebo and Ardupilot. We also added some new features
for our own use cases.

---

This is the official ArduPilot plugin for [Gazebo](https://gazebosim.org/home).
It replaces the previous
[`ardupilot_gazebo`](https://github.com/khancyr/ardupilot_gazebo)
plugin and provides support for the recent releases of the Gazebo simulator
[(Gazebo Harmonic)](https://gazebosim.org/docs/harmonic/install).

It also adds the following features:

- More flexible data exchange between SITL and Gazebo using JSON.
- Additional sensors supported.
- True simulation lockstepping. It is now possible to use GDB to stop
  the Gazebo time for debugging.
- Improved 3D rendering using the `ogre2` rendering engine.

The project comprises a Gazebo plugin to connect to ArduPilot SITL
(Software In The Loop) and some example models and worlds.

## Prerequisites

Gazebo Harmonic is supported on Ubuntu 22.04 (Jammy).
Harmonic is recommended.
If you are running Ubuntu as a virtual machine you will need at least
Ubuntu 20.04 in order to have the OpenGL support required for the
`ogre2` render engine. Gazebo and ArduPilot SITL will also run on macOS
(Big Sur, Monterey and Venturua; Intel and M1 devices).

Follow the instructions for a binary install of
[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install)
and verify that Gazebo is running correctly.

Set up an [ArduPilot development environment](https://ardupilot.org/dev/index.html).
In the following it is assumed that you are able to run ArduPilot SITL using
the [MAVProxy GCS](https://ardupilot.org/mavproxy/index.html).

## Installation

Install additional dependencies:

### Ubuntu

#### Harmonic (apt)

Manual - Gazebo Harmonic Dependencies:

```bash
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

#### Rosdep

Use rosdep with
[osrf's rosdep rules](https://github.com/osrf/osrf-rosdep?tab=readme-ov-file#1-use-rosdep-to-resolve-gazebo-libraries)
to manage all dependencies. This is driven off of the environment variable `GZ_VERSION`.

```bash
export GZ_VERSION=harmonic
sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'
rosdep update
rosdep resolve gz-harmonic
# Navigate to your ROS workspace before the next command.
rosdep install --from-paths src --ignore-src -y
```

Ensure the `GZ_VERSION` environment variable is set to `harmonic`.

Clone the repo and build:

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

## Configure

Set the Gazebo environment variables in your `.bashrc` or `.zshrc` or in
the terminal used to run Gazebo.

#### Terminal

Assuming that you have cloned the repository to `$HOME/ardupilot_gazebo`:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Reload your terminal with `source ~/.bashrc` (or `source ~/.zshrc` on macOS).

## Usage

### 1. Iris quad-copter

#### Run Gazebo

```bash
gz sim -v4 -r iris_runway.sdf
```

The `-v4` parameter is not mandatory, it shows additional information and is
useful for troubleshooting.

#### Run ArduPilot SITL

To run an ArduPilot simulation with Gazebo, the frame should have `gazebo-`
in it and have `JSON` as model. Other commandline parameters are the same
as usual on SITL.

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

#### Arm and takeoff

```bash
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
```

#### Increase the simulation speed

The `zephyr_runway.sdf` world has a `<physics>` element configured to run
faster than real time:

```xml
<physics name="1ms" type="ignore">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>-1.0</real_time_factor>
</physics>
```

### 3. Streaming camera video

Images from camera sensors may be streamed with GStreamer using
the `GstCameraPlugin` sensor plugin. The example gimbal models include the
plugin element:

```xml
<plugin name="GstCameraPlugin"
    filename="GstCameraPlugin">
  <udp_host>127.0.0.1</udp_host>
  <udp_port>5600</udp_port>
  <use_basic_pipeline>true</use_basic_pipeline>
  <use_cuda>false</use_cuda>
</plugin>
```

The `<image_topic>` and `<enable_topic>` parameters are deduced from the
topic name for the camera sensor, but may be overriden if required.

The `gimbal.sdf` world includes a 3 degrees of freedom gimbal with a
zoomable camera. To start streaming:

```bash
gz topic -t
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming -m gz.msgs.Boolean -p "data: 1"
```

### 4. Viewing camera stream using opencv

Display the streamed video:

```bash
gst-launch-1.0 -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

or use OpenCV in Python:

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
        print("Failed to read frame. Is the stream active?")
        break

    cv2.imshow("Stream", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### 4. Using 3d Gimbal

The Iris model is equipped with a 3d gimbal and camera that can be controlled directly in MAVProxy using the RC overrides.

```bash
cd ardupilot

sim_vehicle.py -D -v ArduCopter -f JSON --add-param-file=$HOME/ardupilot_gazebo/config/gazebo-iris-gimbal.parm --console --map
```

Control action for gimbal over RC channel:

| Action | Channel | RC Low     | RC High    |
| ------ | ------- | ---------- | ---------- |
| Roll   | RC6     | Roll Left  | Roll Right |
| Pitch  | RC7     | Pitch Down | Pitch Up   |
| Yaw    | RC8     | Yaw Left   | Yaw Right  |

Example usage:

`rc 6 1100` - Gimbal rolls left

`rc 7 1900` - Gimbal pitch upwards

`rc 8 1500` - Gimbal yaw neutral

## Troubleshooting

For issues concerning installing and running Gazebo on your platform please
consult the Gazebo documentation for [troubleshooting frequent issues](https://gazebosim.org/docs/harmonic/troubleshooting#ubuntu).
