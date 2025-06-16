# Open-Loop Velocity Control with Constant speed and Acceleration in ROS 2

This ROS 2 project implements a simple open-loop controller for a mobile robot that:

- Accelerates from **0 m/s to 2 m/s**
- Cruises at **2 m/s**
- Decelerates back to **0 m/s**
- Travels a total of **10 meters**
- Logs and plots speed and distance in real time

It demonstrates basic kinematic motion planning without feedback (i.e., open-loop control), ideal for simulation or as an educational example.

---

## Project Overview

The motion profile is divided into three phases:

1. **Acceleration Phase**  
   Using `v(t) = a * t` to smoothly ramp up to `2 m/s`.

2. **Constant Speed Phase**  
   Maintains `2 m/s` over a calculated segment of the path.

3. **Deceleration Phase**  
   Gradually reduces speed back to `0 m/s`.

The script also collects data for:
- Speed vs. Time
- Distance vs. Time

And saves it as:
- A CSV log
- A PNG graph

---

## Directory Structure

```
 turtlebot3_openloop_controller
 ├── tb_control/
    ├── tb_control/
    │   └── tb_openLoop_const_vel.py
    │   └── open_loop_acceleration.pytb_openLoop_with_acc_and_decc.py
    ├── README.md
    ├── distance_speed_time_data.csv     # Generated after run
    └── motion_profile.png               # Generated after run
  ```

---

## ⚙️ Requirements

- ROS 2 (Humble or newer)
- Python 3.8+
- `geometry_msgs`
- `rclpy`
- `matplotlib` (for plotting)

Install matplotlib if not already:

```bash
pip install matplotlib
```

---

## 🚀 How to Run

### 1. Clone into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/your_username/tb_control.git
```

### 2. Make the script executable

```bash
chmod +x ~/ros2_ws/src/tb_control/scripts/open_loop_acceleration.py
```

### 3. Build and source

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 4. Run the node

```bash
ros2 run tb_control open_loop_acceleration.py
```

---

## Outputs

Upon completion, the script will:

- Save a CSV file: `distance_speed_time_data.csv`
- Generate a PNG graph: `motion_profile.png`  
  *(with subplots for distance vs time and speed vs time)*

---

## Parameters

You can customize the motion by editing these variables in `open_loop_acceleration.py`:

```python
self.max_speed = 2.0          # Target speed (m/s)
self.total_distance = 10.0    # Total travel distance (m)
self.acceleration = 1.0       # Acceleration and deceleration (m/s²)
```

---

## Example Output

- Distance vs. Time: Smooth increase to 10m
- Speed vs. Time: Triangle profile — ramp up, flat, ramp down

---

## Tested With

- ROS 2 Humble (Ubuntu 22.04)
- TurtleBot3 Waffle Pi simulation
- Python 3.10

---

## License

This project is licensed under the Apache 2.o License.

---

## Acknowledgments

- Developed by **Munyaradzi Pepukai Antony**
- Built using [ROS 2](https://docs.ros.org) and Python
- Plots generated using Matplotlib

---

## Contact

Have suggestions, feedback, or questions?
- Email: munyaradziantony@gmail.com
