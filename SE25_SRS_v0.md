# Software Requirements Specification (SRS)

## Table of Contents
1. [Introduction](#1-introduction)  
   1.1 [Purpose](#11-purpose)  
   1.2 [Scope](#12-scope)  
   1.3 [Glossary](#13-glossary)  
   1.4 [References](#14-references)  
   1.5 [Overview](#15-overview)

2. [Overall Description](#2-overall-description)  
   2.1 [Product Perspective](#21-product-perspective)  
   2.2 [Product Functions](#22-product-functions)  
   2.3 [User Characteristics](#23-user-characteristics)  
   2.4 [Constraints](#24-constraints)  

3. [System Requirements](#3-system-requirements)  
    3.1 [Functional Requirements (REQ-F-XXX)](#31-functional-requirements)  
    3.2 [Non-Functional Requirements (REQ-N-XXX)](#32-non-functional-requirements)   

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

### 1.3 Glossary
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

#### **Requirement ID: REQ-F-001**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | 3D Simulation Environment |
| **Description** | The system shall provide a 3D simulation environment for a 6-axis robotic arm. |
| **Type** | Functional |
| **Source** | Instructor (Customer) |
| **Rationale** |A 3D simulation environment allows users to visualize and test robotic arm operations without the risk of damaging physical hardware. It provides a safe and cost-effective platform for prototyping, debugging motion algorithms, and training personnel. Additionally, it ensures that system behaviors can be evaluated under controlled scenarios, improving design reliability and reducing development time |
| **Priority** |Must have |
| **Verification Method**|1. Launch the simulation application. <br>2. Verify that a 3D environment is displayed correctly. <br>3. Confirm that the 6-axis robotic arm is fully rendered. <br>4. Operate each joint and validate correct kinematic behavior. <br>5. Test collision detection and ensure visual feedback responds accurately. <br>6. Confirm simulation performance is smooth under typical scenarios.|
| **Dependencies**|None|
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-002**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | URDF Model Loading |
| **Description** | The system shall allow a user to load a URDF (Unified Robot Description Format) model of a robotic arm into the simulation.  |
| **Type** | Functional |
| **Source** | Instructor (Customer) |
| **Rationale** | Allowing URDF model loading ensures that the simulation system is flexible and can accommodate various robotic arms without requiring hard-coded models. This capability enables users to experiment with different robotic designs, verify kinematics, and validate control strategies before deploying them on real hardware, thereby increasing versatility and reducing risk.  |
| **Priority** |Must have |
| **Verification Method**| 1. Open the model loading feature feature. <br>2. Select multiple URDF files representing different robotic arms. <br>3. Verify that each robot’s visual appearance, joints, and links are correctly displayed. <br>4. Check that joint movements correspond to defined kinematics. <br>5. Validate collision models respond correctly during simulation. <br>6. Ensure no errors or crashes occur during model loading. <br>7. Inspect logs to confirm proper parsing of XML structure. |
| **Dependencies** |REQ-F-001 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-003**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Interfaz Gráfica de Usuario (GUI) |
| **Description** | The system shall provide a graphical user interface (GUI) for controlling the robotic arm. |
| **Type** | Functional |
| **Source** | Instructor (Customer) |
| **Rationale** | A graphical user interface simplifies interaction with the robotic arm by providing intuitive controls for movement, configuration, and monitoring. It reduces the learning curve for new users, increases operational efficiency, and minimizes the risk of errors that could occur with manual command entry. A well-designed GUI also improves accessibility and supports rapid prototyping and testing.  |
| **Priority** |Must have |
| **Verification Method**|1. Launch the GUI and confirm all interface elements are visible and correctly labeled. <br>2. Operate each control (sliders, buttons, input fields) and verify robotic arm movement corresponds accurately. <br>3. Test saving and loading of configurations. <br>4. Verify emergency stop/reset functionality works. <br>5. Check that the GUI provides feedback for invalid commands or system errors. |
| **Dependencies** |REQ-F-001|
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-004**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Real-time Pose Display |
| **Description** | The GUI shall display the real-time position and orientation of the robotic arm. |
| **Type** | Functional |
| **Source** | Project Statement |
| **Rationale** | Displaying real-time telemetry is crucial for providing the operator with immediate feedback on the robot's status. This allows for precise control, facilitates trajectory debugging, and helps in instantly detecting anomalous behaviors, significantly improving the system's safety and operational efficiency. |
| **Priority** | Must have |
| **Verification Method**| 1. Launch the simulation and the GUI. <br>2. Move a joint of the robotic arm using the GUI controls. <br>3. Verify that the position and orientation values (Cartesian and angular coordinates) in the GUI update in real-time. <br>4. Check that the displayed values correspond to the robot's actual position in the 3D environment. <br>5. Move the end-effector to a known point and confirm that the GUI accurately reflects those coordinates. <br>6. Ensure the data update rate is smooth and does not exhibit a noticeable lag. |
| **Dependencies** | REQ-F-003 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-005**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Individual Joint Control |
| **Description** | The user shall be able to control each joint of the robotic arm individually. |
| **Type** | Functional |
| **Source** | Project Statement |
| **Rationale** | Individual control of each joint is a fundamental feature for robot operation and debugging. It allows the user to make fine adjustments to the pose, verify the range of motion for each axis, and recover the arm from singular or complex positions. Furthermore, it is an indispensable control method for calibration and maintenance tasks. |
| **Priority** | Must have |
| **Verification Method**| 1. Select a specific joint (e.g., Joint 2) in the GUI. <br>2. Use the associated control (e.g., a slider) to command movement in one direction. <br>3. Visually verify in the 3D environment that only the selected joint moves in the expected direction. <br>4. Check that the joint's movement stops when it reaches its defined software limits. <br>5. Repeat steps 1 through 4 for each of the other joints of the robotic arm. <br>6. Confirm that the telemetry displayed in the GUI (REQ-F-004) accurately reflects the current angle of the moved joint. |
| **Dependencies** | REQ-F-003, REQ-F-004 |
| **Status** | Proposed |
---

#### **Requirement ID: REQ-F-006**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Pose Sequence Definition and Execution |
| **Description** | The user shall be able to define and execute a sequence of target poses for the robot's end-effector. |
| **Type** | Functional |
| **Source** | Project Statement |
| **Rationale** | This functionality is essential for automating tasks and transitioning from manual control to programming complex operations. Allowing the definition and execution of sequences enables the robot to perform useful and repetitive tasks (such as pick-and-place or path following) consistently, laying the groundwork for saving and validating complete robot programs. |
| **Priority** | Must have |
| **Verification Method**| 1. Use the GUI to define a sequence of at least three distinct target poses (position and orientation) for the end-effector. <br>2. Save the created pose sequence. <br>3. Press the 'Execute Sequence' control in the GUI. <br>4. Observe in the 3D environment and verify that the robot's end-effector reaches each of the defined poses in the correct order. <br>5. Check that the trajectory between poses is smooth and controlled. <br>6. Verify that the GUI displays feedback on the execution status (e.g., 'Sequence in progress,' 'Sequence completed'). |
| **Dependencies** | REQ-F-003, REQ-F-005|
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-007**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Collision Detection |
| **Description** | The system shall be capable of detecting collisions between the robotic arm and predefined obstacles within the simulation environment. |
| **Type** | Funcional |
| **Source** | Project Statement |
| **Rationale** | Collision detection ensures safe and reliable operation of the robotic arm during simulation by preventing physical interference between components or obstacles. This contributes to reducing risks, costs, and debugging time during development.|
| **Priority** | Must have |
| **Verification Method**| 1. Launch the simulation application. <br>2. Initialize the robotic arm and load predefined obstacles. <br>3. Move the arm to positions that intersect with obstacles. <br>4. Verify that collision detection is triggered accurately and visual feedback is displayed. |
| **Dependencies** |REQ-F-001, REQ-F-004, REQ-F-006 |
| **Status** | Proposed |

---

#### **Requirement ID:REQ-F-008**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Collision Stop |
| **Description** | After a collision is detected, the system shall immediately stop the movement of the robotic arm. |
| **Type** | Funcional |
| **Source** | Project Statement |
| **Rationale** | Implementing an automatic stop after collision detection ensures the safety of the robotic arm and surrounding environment. It prevents hardware damage, protects users, and maintains system integrity. This functionality is essential for testing safety protocols and validating the system’s response to unexpected contact events. |
| **Priority** | Must have |
| **Verification Method**| 1. Launch the simulation environment. <br>2. Initiate robotic arm movement toward a predefined obstacle. <br>3. Trigger a collision event and verify that all arm movements stop immediately. <br>4. Confirm that no further motion commands are executed after the stop signal. <br>5. Observe the system’s feedback to ensure correct detection and response.|
| **Dependencies** | REQ-F-007 |
| **Status** | Proposed |

---
### 3.2 Non-Functional Requirements

---

#### **Requirement ID: REQ-N-001**
| Attribute | Details |
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

#### **Requirement ID: REQ-N-002**
| Attribute | Details |
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

#### **Requirement ID: REQ-N-003**
| Attribute | Details |
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

#### **Requirement ID: REQ-N-004**
| Attribute | Details |
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

#### **Requirement ID: REQ-N-005**
| Attribute | Details |
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

#### **Requirement ID: REQ-N-006**
| Attribute | Details |
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

#### **Requirement ID: REQ-N-007**
| Attribute | Details |
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


---

## 4. Requirements Traceability Matrix (RTM)

| Requirement ID | Operational Scenario(s) | Stakeholder(s) |
| :--- | :--- | :--- |
| **REQ-F-001** | Scenario 1: System Initialization and Robot/Environment Loading | End-User (Hypothetical) |
| **REQ-F-002** | Scenario 1: System Initialization and Robot/Environment Loading | End-User (Hypothetical) |
| **REQ-F-003** | Scenario 2: Manual Control of the Robotic Arm | End-User (Hypothetical) |
| **REQ-F-004** | Scenario 2: Manual Control of the Robotic Arm | End-User (Hypothetical) |
| **REQ-F-005** | Scenario 2: Manual Control of the Robotic Arm | End-User (Hypothetical) |
| **REQ-F-006** | Scenario 3: Automated Path Execution | End-User (Hypothetical) |
| **REQ-F-007** | Scenario 4: Fault Detection and Response | Instructor (Customer) |
| **REQ-F-008** | Scenario 4: Fault Detection and Response | Instructor (Customer) |
| **REQ-N-001** | Scenario 2, Scenario 3 | End-User (Hypothetical) |
| **REQ-N-002** | Scenario 1, 2, 3, 4 | End-User (Hypothetical) |
| **REQ-N-003** | N/A (Course Constraint) | Instructor (Customer) |
| **REQ-N-004** | N/A (Course Constraint) | Instructor (Customer) |
| **REQ-N-005** | N/A (Course Constraint) | Instructor (Customer) |
| **REQ-N-006** | N/A (Development Process) | Students (Developers) |
| **REQ-N-007** | N/A (Course Deliverable) | Instructor (Customer) |
