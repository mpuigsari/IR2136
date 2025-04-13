# Autonomous Underwater Inspection with Unity, ROS, and YOLOv8

> A project developed for **IR2136 - Aerial and Underwater Robots**, integrating Unity simulation with real-time computer vision and ROS-based control.

---

## 🌊 Project Overview
This project showcases a simulated autonomous underwater inspection system using:

- A **Unity-based submarine simulator**
- A **ROS Noetic Docker container**
- **YOLOv8 segmentation** for pipeline and fissure detection
- Real-time communication via **rosbridge**
- **Manual teleoperation** and visualization with `rqt`

This system is tailored to detect underwater pipelines, fissures, and shadows, enabling the robot pilot to be notified upon fissure detection — simulating real-world inspection scenarios.

---

## 🧰 Technologies & Tools
- [ROS Noetic](http://wiki.ros.org/noetic)
- [Docker + docker-compose](https://docs.docker.com/compose/)
- [Unity Simulator](https://github.com/org-arl/UWRoboticsSimulator)
- [YOLOv8 Segmentation](https://github.com/ultralytics/ultralytics)
- [ultralytics_ros](https://github.com/Alpaca-zip/ultralytics_ros) (custom model)
- [Roboflow Instance Segmentation Project](https://universe.roboflow.com/ir2136/underwater-pipeline-segmentation)

---

## 🧱 System Architecture
```

+---------------------------+       +---------------------------+       +----------------------------+
|        Unity              | <---> |         Docker            | <---->|           Pilot            |
|---------------------------|       |---------------------------|       |----------------------------|
| - Publishes camera feed   |       | - ROSBridge node          |       | - Receives /yolo_image     |
|   (/images/front/...)     |       | - YOLOv8 node             |       |   for visualization        |
| - Accepts control commands|       | - Controller node         |       | - Sends keyboard teleop    |
|   from ROSBridge          |       +---------------------------+       |   commands to controller   |
+---------------------------+                                           +----------------------------+

```

---

## 🧪 Dataset & Model Training
Custom dataset creation and training pipeline:

- Annotated using **Roboflow polygon tool** with 3 classes:
  - `pipeline`
  - `pipeline-fissure`
  - `pipeline-shadow`
- Exported in **COCO** format and converted to YOLOv8 segmentation format
- Trained with `yolov8s-seg` on RTX 3080 Ti
- Achieved **mAP50-95 = 0.77**

> View dataset here: [Roboflow project](https://universe.roboflow.com/ir2136/underwater-pipeline-segmentation)


![Segmentation Overview](/media/segmentation.gif)

---

## 🚀 How to Run the Project
### Step 1: Start Unity Simulator
- Open the Unity project (UWRoboticsSimulator)
- Ensure WebSocket is set to `ws://localhost:9090`
- Start simulation — camera publishes on `/images/front/compressed`

### Step 2: Start Docker (ROS + YOLO)
```bash
xhost +local:root  # allow GUI
cd ros_docker_ws
docker compose up -d
```
Then, inside the container:
```bash
roslaunch ultralytics_ros tracker.launch \
  yolo_model:=pipelinev2_yolov8s-seg.pt \
  conf_thres:=0.5 \
  input_topic:=/images/front/compressed \
  classes:="[0,1,2]"
```

### Step 3: Visualize + Control
Open a second terminal:
```bash
docker exec -it ros_yolo bash
rqt &
rosrun controllerpkg key_publisher.py
```
![Segmentation Overview](/media/setup.gif)

---

## 🔄 Data Flow Summary
- Unity publishes compressed camera frames to ROS
- YOLOv8 model in `ultralytics_ros` performs real-time segmentation
- Results are published to `/yolo_result` and `/yolo_image`
- `rqt` visualizes detections, and the robot can be controlled via keyboard

---

## 🛠️ Future Work
- Integrate autonomous navigation using detection feedback
- Convert YOLO model to ONNX/TensorRT for faster inference
- Improve segmentation accuracy with more data and tuning


---

## 📎 References
- [ultralytics_ros GitHub](https://github.com/Alpaca-zip/ultralytics_ros)
- [UWRoboticsSimulator GitHub](https://github.com/org-arl/UWRoboticsSimulator)
- [Roboflow Dataset](https://universe.roboflow.com/ir2136/underwater-pipeline-segmentation)

---

## 👤 Author
**Max Puig**  
Bachelor in Robotics Intelligence — Universitat Jaume I (2021–2025)

---

> This project is part of my academic portfolio and showcases integration of simulation, real-time perception, and robot teleoperation using modern robotics software tools.
