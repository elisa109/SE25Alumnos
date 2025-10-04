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
- `/RobotState` topic: simulation publishes joint positions and end-effector pose.  
- `/ControlCommands` topic: control interface sends joint commands to the simulation.  

Thus, the system integrates **simulation accuracy** with **user interactivity** while adhering to real-time performance constraints.
<img width="2335" height="1240" alt="image" src="https://github.com/user-attachments/assets/3f89484b-7c49-4cee-8de7-dac740b4d044" />



### 2.2 Product Functions
At a high level, the SE25 system will provide the following functions:
- **System Initialization:** Load the robotic arm model (URDF) into the simulation environment.  
- **Manual Control:** Allow users to jog individual joints of the robotic arm via the GUI.  
- **Automated Path Execution:** Accept a sequence of target poses and execute them as trajectories in the simulation.  
- **Collision Detection:** Identify collisions between the robotic arm and environment obstacles.  
- **Emergency Response:** Halt motion immediately upon collision or detected fault.  
- **Visualization:** Display real-time robot pose and environment state to the user.   

### 2.3 User Characteristics
The system is intended for three primary categories of stakeholders:  
- **Students (Developers):**  
  - Background: Undergraduate software engineering students.  
  - Need: A clear, structured project that allows them to apply concepts from ISO 12207, real-time programming, and software engineering best practices.  
- **Instructor (Customer):**  
  - Role: Defines the project requirements and evaluates deliverables.  
  - Need: A project that fulfills course objectives, ensures traceability of requirements, and demonstrates applied knowledge.  
- **End-User (Hypothetical Researcher):**  
  - Background: Robotics researcher.  
  - Need: A reliable, flexible, and user-friendly tool for testing control algorithms without requiring physical robots.  

### 2.4 Constraints
The development and execution of the system are subject to the following constraints:  
- **Platform Constraint:** The system must run on **Ubuntu 24.04 LTS**.  
- **Programming Languages:** The implementation shall use **C/C++**.  
- **Concurrency:** Multithreading must rely on **POSIX threads (pthreads)**.  
- **Middleware:** All communication between components must use **ROS 2 Jazzy Jalisco**.  
- **Version Control:** All source code shall be version controlled using **Git&GitHub**.  
- **Performance:** The system shall achieve a minimum of 30 FPS rendering rate. 


---

## 3. System Requirements

### 3.1 Functional Requirements
### **Requisitos Funcionales**

---

#### **Requirement ID: REQ-F-001**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | 3D Simulation Environment |
| **Description** | The system shall provide a 3D simulation environment for a 6-axis robotic arm. |
| **Type** | Functional |
| **Source** | Instructor (Customer) |
| **Rationale** |A 3D simulation environment allows users to visualize and test robotic arm operations without the risk of damaging physical hardware. It provides a safe and cost-effective platform for prototyping, debugging motion algorithms, and training personnel. Additionally, it ensures that system behaviors can be evaluated under controlled scenarios, improving design reliability and reducing development time |
| **Priority** |Must have |
| **Verification Method**|1. Launch the simulation application.
2. Verify that a 3D environment is displayed correctly.
3. Confirm that the 6-axis robotic arm is fully rendered.
4. Operate each joint and validate correct kinematic behavior.
5. Test collision detection and ensure visual feedback responds accurately.
6. Confirm simulation performance is smooth under typical scenarios |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-F-002**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Carga de modelo URDF |
| **Description** | El sistema deberá permitir a un usuario cargar un modelo URDF (Unified Robot Description Format) de un brazo robótico en la simulación. |
| **Type** | Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-F-003**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Interfaz Gráfica de Usuario (GUI) |
| **Description** | El sistema deberá proporcionar una interfaz gráfica de usuario (GUI) para controlar el brazo robótico. |
| **Type** | Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-F-004**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Visualización de posición en tiempo real |
| **Description** | La GUI deberá mostrar la posición y orientación en tiempo real del brazo robótico. |
| **Type** | Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-F-005**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Control articular individual |
| **Description** | El usuario deberá ser capaz de controlar cada articulación del brazo robótico de forma individual. |
| **Type** | Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-F-006**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Definición y ejecución de secuencias de poses |
| **Description** | El usuario deberá ser capaz de definir y ejecutar una secuencia de poses objetivo para el efector final del robot. |
| **Type** | Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-F-007**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Detección de colisiones |
| **Description** | El sistema deberá ser capaz de detectar colisiones entre el brazo robótico y obstáculos predefinidos en la simulación. |
| **Type** | Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-F-008**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Parada por colisión |
| **Description** | Tras la detección de una colisión, el sistema deberá detener inmediatamente el movimiento del robot. |
| **Type** | Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---
### **Requisitos No Funcionales**

---

#### **Requisito ID: REQ-N-001**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Latencia de comunicación |
| **Description** | La comunicación entre la interfaz de operador y la simulación deberá tener una latencia inferior a 50 milisegundos. |
| **Type** | No Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-N-002**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Tasa de fotogramas (Framerate) |
| **Description** | El entorno de simulación deberá mantener una tasa de fotogramas de al menos 30 frames por segundo (FPS). |
| **Type** | No Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-N-003**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Plataforma de desarrollo |
| **Description** | El sistema deberá ser desarrollado usando C/C++ sobre un sistema operativo Linux. |
| **Type** | No Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-N-004**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Framework de comunicación |
| **Description** | El sistema deberá utilizar ROS 2 para toda la comunicación entre componentes. |
| **Type** | No Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-N-005**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Interfaces para programación concurrente |
| **Description** | El sistema deberá utilizar interfaces POSIX para tareas de programación concurrente. |
| **Type** | No Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-N-006**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Control de versiones |
| **Description** | Todo el código fuente deberá ser controlado en versiones utilizando Git. |
| **Type** | No Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |

---

#### **Requisito ID: REQ-N-007**
| Atributo | Detalles |
| :--- | :--- |
| **Requirement Name** | Documentación del sistema |
| **Description** | El sistema deberá ser documentado de acuerdo con el plan de documentación del proyecto. |
| **Type** | No Funcional |
| **Source** | Enunciado del Proyecto |
| **Rationale** | |
| **Priority** | |
| **Verification Method**| |
| **Dependencies** | |
| **Status** | Propuesto |
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

## 4. Requirements Traceability Matrix (RTM)

| Requirement ID | Operational Scenario(s) | Stakeholder(s) |
|----------------|--------------------------|----------------|
| REQ-F-001      | SC-01: Robot movement in simulation | Developer, Researcher |
| REQ-N-001      | SC-02: Real-time visualization       | Tester, End User |
