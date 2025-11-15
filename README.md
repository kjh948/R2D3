# R2D3
R2D2 facelift

- OS
  - pi4, buster
  - ROS1 Noetic

- GPIO connections
  - gpio26 - encoderA
  - gpio19 - encoderB
  - gpio13 - zero hole
  - gpio06 - WS2812 LED data

- Dome
  - dome motor is connected to `mh.getMotor(3)`
  - uses encoder A/B and zero-hole (index) sensor for position control
  - at startup, homing sets the zero reference using the zero-hole sensor

- LED
  - 2 WS2812 LEDs at the head
  - controlled via `library/led_controller.py`


## ROS API (Noetic)
This project provides a simple ROS package `r2d3_control` (under `src/`) that wraps the local hardware controllers and exposes ROS-friendly interfaces.

Package location
- `/home/pi/workspace/r2d3/src` (package name: `r2d3_control`)

Nodes
- `dome_node`
  - Services:
    - `/dome/home` (std_srvs/Trigger)
      - Triggers dome homing sequence (uses zero-hole sensor). Returns success and message.
  - Topics:
    - `/dome/move_to_count` (std_msgs/Int32)
      - Publish a target encoder count to move the dome to (non-blocking).

- `led_node`
  - Topics:
    - `/led/set_color` (std_msgs/ColorRGBA)
      - Sets all LEDs to the given color. ColorRGBA uses floats 0.0–1.0.
  - Services (std_srvs/Trigger):
    - `/led/pattern_warning` — run red blink warning pattern
    - `/led/pattern_success` — run green pulse success pattern
    - `/led/pattern_thinking` — run non-blocking thinking pattern
    - `/led/stop_pattern` — stop any running LED pattern

- `motor_node`
  - Topics:
    - `/cmd_vel` (geometry_msgs/Twist)
      - Standard Twist messages: use `linear.x` for forward velocity (m/s) and `angular.z` for yaw rate (rad/s). These are forwarded to the open-loop wheel controller in `library/wheel.py`.

Examples
- Home the dome:

```bash
rosservice call /dome/home
```

- Move dome to encoder count 100:

```bash
rostopic pub /dome/move_to_count std_msgs/Int32 "data: 100" --once
```

- Set LEDs to blue:

```bash
rostopic pub /led/set_color std_msgs/ColorRGBA "{r: 0.0, g: 0.0, b: 1.0, a: 1.0}" --once
```

- Trigger a warning pattern:

```bash
rosservice call /led/pattern_warning
```

- Drive the robot (open-loop):

```bash
rostopic pub /cmd_vel geometry_msgs/Twist "{ linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x:0.0, y:0.0, z:0.2} }" --once
```


## Build and Run

### Prerequisites
- ROS Noetic installed and sourced via `~/ros1.sh`
- Python dependencies: `pip install rpi-ws281x` (or follow rpi_ws281x setup)
- Emakefun libraries in Python path (included in `library/`)

### Build
From the workspace root:

```bash
source ~/ros1.sh
cd /home/pi/workspace/r2d3
catkin_make
```

### Run Nodes
Source the ROS environment and devel setup, then launch:

```bash
source ~/ros1.sh
cd /home/pi/workspace/r2d3
source devel/setup.bash
roslaunch r2d3_control r2d3_control.launch
```

**Note:** The launch file runs `led_node` with `sudo` to access GPIO/I2C hardware. You will be prompted for a password when the node starts, unless you configure passwordless sudo for the helper script:

```bash
echo "$USER ALL=(ALL) NOPASSWD: /tmp/run_led_node_sudo.sh" | sudo tee -a /etc/sudoers.d/r2d3_sudo
```

### Verify Nodes
In another terminal, verify that the nodes are running and services are registered:

```bash
source ~/ros1.sh
rosservice list
rostopic list
```

You should see:
- **Services:** `/dome/home`, `/led/pattern_warning`, `/led/pattern_success`, `/led/pattern_thinking`, `/led/stop_pattern`
- **Topics:** `/cmd_vel`, `/led/set_color`, `/dome/move_to_count`, `/rosout`, `/rosout_agg`


## Notes & Safety
- **Wheel motor control is open-loop** (no wheel encoders); test at low speeds and be ready to cut power.
- **Dome homing uses GPIO and motors** — ensure wiring is correct and a safe environment before running homing.
- **Hardware permissions** — LED access requires GPIO and I2C permissions; the launch file runs `led_node` with `sudo` to enable this.
- **Python path** — A helper script (`/tmp/run_led_node_sudo.sh`) wraps the led_node to ensure ROS environment variables are available when running under sudo.


## Audio Node

The `audio_node` provides emotion-based audio playback from MP3 files stored in `src/resources/<emotion>/` folders.

### Supported Emotions
- Annoyed, Chat, Fun, Happy, Ooh, Question, Scared, Scream, Yell

### Service
- `/audio/play_emotion` (r2d3_control/PlayAudio)
  - **Request:** `string emotion` (emotion folder name)
  - **Response:** `bool success`, `string message`
  - Plays a random MP3 file from the emotion folder

### Examples

Play a happy emotion audio:
```bash
rosservice call /audio/play_emotion "emotion: 'Happy'"
```

Play scared emotion:
```bash
rosservice call /audio/play_emotion "emotion: 'Scared'"
```

### Requirements
- Audio playback: `aplay` (default ALSA player on Raspberry Pi) or `mpg123`
- MP3 files stored in resource folders (included)

### Audio File Structure
```
src/resources/
  Annoyed/
    *.mp3
  Chat/
    *.mp3
  ... (other emotions)
```

Each emotion folder contains multiple MP3 files; the service randomly selects one when called.
