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
