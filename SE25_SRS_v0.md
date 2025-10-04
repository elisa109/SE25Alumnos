# Software Requirements Specification (SRS)

## Table of Contents
1. [Introduction](#1-introduction)  
   1.1 [Purpose](#11-purpose)  
   1.2 [Scope](#12-scope)
   1.3 [Glossary](#13-glossary)
   1.4 [References](#14-references)  
   1.5 [Overview](#15-overview)  

3. [Overall Description](#2-overall-description)  
   2.1 [Product Perspective](#21-product-perspective)  
   2.2 [Product Functions](#22-product-functions)  
   2.3 [User Characteristics](#23-user-characteristics)  
   2.4 [Constraints](#24-constraints)  
   2.5 [Assumptions and Dependencies](#25-assumptions-and-dependencies)  

4. [System Requirements](#3-system-requirements)  
   - Functional Requirements (REQ-F-XXX)  
   - Non-Functional Requirements (REQ-N-XXX)  

5. [Requirements Traceability Matrix (RTM)](#4-requirements-traceability-matrix)  

---

## 1. Introduction

### 1.1 Purpose
The purpose of this Software Requirements Specification (SRS) is to formally describe the requirements of the **SE25 – Robot Simulation and Control System**. This system aims to provide a realistic and interactive simulation of a 6-axis robotic arm, enabling both manual and automated control in a virtual 3D environment.  

The SRS document serves several purposes:
- **For developers (students):** It provides a clear specification to guide system design, implementation, and testing.  
- **For the instructor (customer):** It establishes a baseline to evaluate the team’s progress and ensure alignment with the educational objectives of the course.  
- **For end-users (hypothetical researchers):** It communicates what the system is expected to do and the constraints under which it must operate.  

This document will serve as a reference throughout the entire software life cycle, ensuring traceability from requirements definition to implementation and validation.

### 1.2 Scope
The **System of Interest (SoI)** is a distributed robotic simulation and control platform. It is composed of two main subsystems:  
1. **Simulation Environment:** A physics-based 3D environment capable of rendering a robotic arm and simulating its movement under user-defined commands.  
2. **Control Workstation:** A human–machine interface (HMI) that enables operators to issue manual control commands, define automated trajectories, and monitor the robot’s state.  

The communication between these components will be managed using **ROS 2 Jazzy Jalisco**, ensuring real-time data exchange and modularity.  

The primary objectives of the system are:
- Provide a robust 3D environment for robotic arm simulation.  
- Allow users to manually manipulate the robot’s joints or execute predefined motion trajectories.  
- Detect collisions with obstacles and respond with immediate safety measures.  
- Visualize the robot’s state and motion in real-time through a user-friendly GUI.  

### 1.3 Definitions, Acronyms, Abbreviations
- **SoI (System of Interest):** The robotic simulation and control software being developed.  
- **OpsCon (Operational Concept):** A user-oriented description of how the system is expected to operate.  
- **URDF (Unified Robot Description Format):** A standard XML format used to describe the structure of a robot model.  
- **ROS 2 (Robot Operating System 2):** Middleware framework for distributed robotics applications.  
- **HMI (Human-Machine Interface):** The graphical interface through which the user interacts with the system.  
- **REQ-F:** Functional Requirement.  
- **REQ-N:** Non-Functional Requirement.  
- **RTM (Requirements Traceability Matrix):** A matrix mapping requirements to scenarios and stakeholders.  


### 1.4 References
- SE25 Project Description Document (SE25-01 v1.0 Draft, September 2025).  
- ISO/IEC/IEEE 12207:2017, *Systems and software engineering — Software life cycle processes*.  
- ISO/IEC/IEEE 29148:2018, *Systems and software engineering — Requirements engineering*.  
- IEEE Computer Society, *SWEBOK Guide v4.0*, 2025. 

### 1.5 Overview
This document is structured as follows:
- **Section 1 (Introduction):** Defines the purpose, scope, terminology, and references.  
- **Section 2 (Overall Description):** Provides a high-level description of the system, including perspective, functions, users, constraints, and assumptions.  
- **Section 3 (System Requirements):** Lists and details functional and non-functional requirements with unique IDs and acceptance criteria.  
- **Section 4 (Traceability Matrix):** Maps requirements to operational scenarios and stakeholders to ensure coverage and alignment.  


## 2. Overall Description

### 2.1 Product Perspective
The system will act as a standalone 3D simulation platform. The SE25 system is designed as a **distributed real-time robotic simulation and control platform**. It consists of two interconnected subsystems:  
1. **Simulation Environment:**  
   - Built with a physics engine capable of simulating realistic robotic arm kinematics and dynamics.  
   - Provides a 3D rendering of the robot and its environment.  
   - Responsible for collision detection and state publishing.  
2. **Control Workstation:**  
   - Provides a GUI for user interaction.  
   - Sends manual and automated commands to the simulation.  
   - Receives and displays the robot’s current state in real time.  

The communication between these subsystems is implemented using **ROS 2 Jazzy Jalisco**, enabling a modular publish–subscribe architecture. For example:  
- `/robot_state` topic: simulation publishes joint positions and end-effector pose.  
- `/joint_commands` topic: control interface sends joint commands to the simulation.  

Thus, the system integrates **simulation accuracy** with **user interactivity** while adhering to real-time performance constraints.
<img width="2335" height="1240" alt="image" src="https://github.com/user-attachments/assets/3f89484b-7c49-4cee-8de7-dac740b4d044" />



### 2.2 Product Functions
- 3D rendering of environment and robots  
- Camera navigation  
- Basic simulation controls  

### 2.3 User Characteristics
- Developers, researchers, and testers with basic robotics knowledge  

### 2.4 Constraints
- Should run on standard consumer hardware  
- Should be cross-platform (Windows/Linux/Mac)  

### 2.5 Assumptions and Dependencies
- Relies on a physics engine for simulation accuracy  

---

## 3. System Requirements

### 3.1 Functional Requirements

#### REQ-F-001: 3D Simulation Environment
**ID:** REQ-F-001  
**Type:** Functional  
**Title:** 3D Simulation Environment  
**Description:**  
The system shall provide a 3D simulation environment that includes:  
- A ground plane with configurable textures  
- A global coordinate system (X, Y, Z axes)  
- Adjustable lighting sources (ambient, directional, point lights)  
- Camera navigation (pan, zoom, rotate)  

**Rationale:** Provides a realistic test environment without physical prototypes.  
**Dependencies:** REQ-N-001 (performance)  
**Acceptance Criteria:**  
- The environment loads with a ground plane visible.  
- Coordinate axes are displayed.  
- Lighting can be adjusted by the user.  

---

### 3.2 Non-Functional Requirements

#### REQ-N-001: Performance
**ID:** REQ-N-001  
**Type:** Non-Functional  
**Title:** Performance Requirement  
**Description:**  
The system shall render at a minimum of 30 FPS under standard simulation conditions.  

**Rationale:** Ensures smooth and usable simulation.  
**Dependencies:** REQ-F-001  
**Acceptance Criteria:** Simulation runs at or above 30 FPS on reference hardware.  

---

## 4. Requirements Traceability Matrix (RTM)

| Requirement ID | Operational Scenario(s) | Stakeholder(s) |
|----------------|--------------------------|----------------|
| REQ-F-001      | SC-01: Robot movement in simulation | Developer, Researcher |
| REQ-N-001      | SC-02: Real-time visualization       | Tester, End User |
