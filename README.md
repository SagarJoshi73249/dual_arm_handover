# Dual Arm Handover System

This repository contains a skeleton implementation of a dual-arm robotic handover system driven by a diffusion policy. The system architecture emphasizes tight integration between sensing, planning, and actuation for low-latency, contact-rich manipulation tasks.

---

## Overview

The system enables one robotic arm to pick an object and transfer it to a second arm, which places it in a target bin. It is designed with modular ROS2 nodes for control and real-time policy execution outside the ROS message pipeline for latency minimization.

---

## Features

- Dual-arm control with synchronized MoveIt FK/IK services
- Direct RGB camera access for low-latency vision input
- Tactile sensing via ROS2 topic subscriptions (per fingertip)
- Gripper actuation
- Skeleton class for diffusion policy inference
- Unified observation dictionary per timestep (RGB + FK + tactile + gripper)

---