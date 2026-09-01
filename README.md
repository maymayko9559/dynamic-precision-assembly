# Dynamic Precision Assembly System

Computer Vision 기반 동적 작업환경 대응형 자율 정밀 조립 시스템

## System

RealSense Camera
→ OpenCV Detection
→ Object Tracking
→ Velocity Estimation
→ Kalman Filter
→ Motion Prediction
→ Robot Control
→ Precision Assembly

## ROS2 Packages

- `vision_system` - OpenCV-based object and target detection
- `object_tracking` - Object tracking, velocity estimation, Kalman filtering, and motion prediction
- `robot_control` - Doosan robot motion and precision assembly control
- `assembly_interfaces` - Custom ROS2 messages and services
