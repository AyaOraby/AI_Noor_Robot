# NOOR Robot – AI Perception System

By:
Aya Oraby 
Yasmeen Abosaif 

**Course:** Robotics Project – CIE 595 Spring
**Team:** AI Team – Zewail City of Science and Technology

---

## Table of Contents

1. [Introduction](#introduction)
2. [System Features](#system-features)
3. [Installation Steps](#installation-steps)
4. [Running the System](#running-the-system)
5. [Package Descriptions and Launch Instructions](#package-descriptions-and-launch-instructions)
6. [Hardware Notes](#hardware-notes)
7. [Conclusion](#conclusion)

---

## Introduction

The NOOR robot is a smart social robot built to interact naturally with humans in Arabic. This perception system equips the robot with the ability to "see" and "understand" people and its surroundings using computer vision and AI tools, enabling:

* Real-time object detection
* Facial detection and recognition
* Emotion and gender classification
* ROS-based inter-module communication

---

## System Features

This system integrates several AI-based perception modules with ROS to enable:

* Object detection using YOLOv3 (`darknet_ros`)
* Face detection and tracking
* Emotion and gender inference using DeepFace
* Face identity recognition
* Realsense or USB camera integration
* Output visualizations using `rqt_image_view`

---

## Installation Steps

### 1. Prerequisites

* OS: Ubuntu 20.04
* ROS Version: Noetic
* Hardware: Jetson Nano or Raspberry Pi 4
* Camera: Intel RealSense or USB webcam



### 2. Install Dependencies

```bash
sudo apt update
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-vision-msgs
pip install deepface opencv-python dlib
```

### 3. Download Pretrained Models

* YOLOv3 weights: [YOLOv3 Weights](https://pjreddie.com/media/files/yolov3.weights)
* DeepFace models will auto-download on first run
* packages link   https://drive.google.com/drive/folders/1GbMzqU6o78yWGliWEAFnynD8MdWFbGTf
  

---

## Running the System

### Launch All Modules

```bash
# 1. Start camera
roslaunch realsense2_camera rs_camera.launch

# 2. Start YOLO object detection
roslaunch darknet_ros darknet_ros.launch

# 3. Start face/emotion/gender recognition
roslaunch noor_ai AI_launch.launch
```

---

## Package Descriptions and Launch Instructions

### 1. `realsense2_camera` – Camera Driver

**Introduction**:
This ROS package interfaces the Intel RealSense camera with ROS, publishing image and depth topics required by other perception nodes.

**Launch Command**:

```bash
roslaunch realsense2_camera rs_camera.launch
```

**Topics Published**:

* `/camera/color/image_raw`
* `/camera/depth/image_rect_raw`

---

### 2. `darknet_ros` – YOLO Object Detection

**Introduction**:
Integrates YOLOv3 (You Only Look Once) for real-time object detection with ROS. Detects objects in the camera stream and publishes bounding boxes.

**Launch Command**:

```bash
roslaunch darknet_ros darknet_ros.launch
```

**Topics Published**:

* `/darknet_ros/bounding_boxes`
* `/darknet_ros/detection_image`

---

#### 3. `ai_detection` – Face detection and tracking module

**Introduction:** Uses OpenCV and SSD or Haar cascades to detect faces in camera frames and publish ROIs for other packages.
**Launch Command:**

```bash
roslaunch ai_detection detection.launch
```

---

### 4.  `Face Recognition` – Recognize idetity, gender and emotion of person detected
`emotion_detection_ros`, `identity_detection_ros`, `gender_detection_ros`

**Introduction**:
These custom ROS packages use the DeepFace library to analyze cropped face images and infer emotion, identity, and gender.

**Launch Command (All Combined in AI\_launch.launch)**:

```bash
roslaunch noor_ai AI_launch.launch
```

**Topics Published**:

* `/noor_perception/deepface_analysis`
* `/deepface/person_detected`
* `/deepface/gender`
* 

---

## Hardware Notes

### Issue:

Jetson Nano may shut down when launching camera node due to insufficient power.

### Solution:

* Offload DeepFace inference to Raspberry Pi 4
* Use externally powered USB hub for camera

---

## Conclusion

This AI module equips the NOOR robot with essential real-time perception abilities for Arabic HRI using DeepFace and YOLO, running under the ROS1 Noetic framework. Despite hardware limitations, the system was optimized for Jetson Nano/Raspberry Pi environments and successfully integrated with the chatbot and navigation systems.
