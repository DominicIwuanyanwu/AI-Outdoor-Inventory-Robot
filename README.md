# AI-Outdoor-Inventory-Robot
AI-powered mobile inventory robot using Raspberry Pi, ONNX computer vision, OpenCV, barcode decoding, ROS 2, Arduino motor control, and Gazebo/Nav2 simulation. Final year project.
# AI Outdoor Inventory Robot

Final year project — a Raspberry Pi-based mobile robot that detects barcodes using computer vision, decodes them, and logs inventory to CSV.

## What it does
- **Barcode Detection:** Pi Camera → ONNX model detects barcode regions → OpenCV extracts ROI → pyzbar decodes value
- **Inventory Logging:** Successful scans update `inventory.csv` with item counts and timestamps
- **Robot Control:** Raspberry Pi sends serial commands to an Arduino/Elegoo chassis for movement
- **Obstacle Avoidance:** HC-SR04 ultrasonic sensor stops the robot when objects are too close
- **Simulation:** ROS 2 / Gazebo / Nav2 demonstrates autonomous navigation and SLAM mapping

## Repo Structure
| Folder | Contents |
|--------|----------|
| `notebooks/` | `BarcodeTrainedModel.ipynb` — YOLO/ONNX model training pipeline |
| `arduino/` | Motor controller & sensor firmware |
| `physical_robot/` | Python scanning scripts + ROS 2 workspace for the real robot |
| `simulation/` | ROS 2 / Nav2 workspace for Gazebo simulation |
| `models/` | Placeholder for `best.onnx` (not included due to size) |
| `docs/` | Images, diagrams, evidence |

## Hardware
- Raspberry Pi 5
- Pi Camera Module
- Arduino / Elegoo robot chassis + L298N motor driver
- HC-SR04 Ultrasonic Sensor
- Power supply + jumper wires

## Software Stack
Python, ROS 2 Humble, OpenCV, ONNX Runtime, pyzbar, Gazebo, SLAM Toolbox, Nav2

## Quick Start (Physical Robot)
1. Clone repo
2. Install Python deps: `pip install -r requirements.txt`
3. Place your trained `best.onnx` in `/models`
4. Flash `arduino/robot_firmware/` to your Arduino board
5. Build ROS 2 workspace:
   ```bash
   cd physical_robot/ros2_ws
   colcon build
   source install/setup.bash

   
---

## 2. `physical_robot/README.md`

```markdown
# Physical Robot

Real hardware implementation. Raspberry Pi handles perception and decision-making; Arduino handles motor control.

## Nodes
| Node | File | Purpose |
|------|------|---------|
| Camera Scan | `inventory_project/scan_store_onnx.py` or similar | Captures frames, runs ONNX inference, decodes barcodes, updates CSV |
| Obstacle Avoid | `inventory_project/obstacle_avoid_node.py` | Reads ultrasonic distance, publishes stop/slow commands |
| Serial Bridge | `inventory_project/serial_bridge_node.py` | Converts ROS 2 `/cmd_vel` to serial strings for Arduino |

## How it runs
1. Arduino flashed with firmware from `../arduino/robot_firmware/`
2. Pi camera enabled (`sudo raspi-config`)
3. Start ROS 2 nodes:
   ```bash
   ros2 run inventory_robot camera_scan
   ros2 run inventory_robot serial_bridge
   ros2 run inventory_robot obstacle_avoid

   
---

## 3. `simulation/README.md`

```markdown
# Simulation Environment

ROS 2 + Gazebo + Nav2 for autonomous navigation demonstration.

## What's inside
- `inventory_sim/` — Gazebo world, robot URDF/Xacro, LiDAR plugin
- SLAM Toolbox for mapping
- Nav2 for path planning and goal navigation

## Launch
```bash
ros2 launch inventory_sim simulation.launch.py
ros2 launch inventory_sim nav2.launch.py


---

## 4. `arduino/README.md`

```markdown
# Arduino Firmware

Low-level motor control and sensor reading for the Elegoo/Arduino chassis.

## Files
- `robot_firmware/motor_controller_serial.ino` — main sketch

## What it does
- Listens for serial commands from Raspberry Pi
- Drives L298N motor driver based on left/right speed values
- Reads HC-SR04 ultrasonic sensor
- Sends distance readings back to Pi

## Pinout (adjust to your wiring)
| Component | Arduino Pin |
|-----------|-------------|
| Motor A IN1 | 2 |
| Motor A IN2 | 3 |
| Motor A EN | 5 (PWM) |
| Motor B IN1 | 4 |
| Motor B IN2 | 7 |
| Motor B EN | 6 (PWM) |
| HC-SR04 Trig | 8 |
| HC-SR04 Echo | 9 |

## Serial Baud Rate
9600 (match this in the Pi's serial bridge node)

# Model Training

`BarcodeTrainedModel.ipynb` — Jupyter notebook for training the barcode detection model.

## Pipeline
1. Dataset preparation (barcode images from `../docs/images/`)
2. YOLO training
3. Export to ONNX format (`best.onnx`)
4. Validation metrics

## Output
Place the exported `best.onnx` in `../models/` before running the physical robot.
