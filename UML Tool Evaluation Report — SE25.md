# UML Tool Evaluation Report — SE25

---

## Table of Contents
- [1. Introduction](#1-introduction)
  - [1.1 Scope & Objectives](#11-scope--objectives)
  - [1.2 Tools Under Evaluation](#12-tools-under-evaluation)
  - [1.3 Evaluation Methodology](#13-evaluation-methodology)
- [2. Detailed Evaluation](#2-detailed-evaluation)  
  - [2.1 Eclipse Papyrus](#21-eclipse-papyrus)  
    -[2.1.1 Eclipse Papyrus Overview](#211-eclipse-papyrus-overview)  
    -[2.1.2 Eclipse Papyrus Diagrams](#212-eclipse-papyrus-diagrams)  
  - [2.2 Modelio](#22-modelio)   
    -[2.2.1 Modelio Overview](#221-modelio-overview)   
    -[2.2.2 Modelio Diagrams](#222-modelio-diagrams)   
  - [2.3 IBM Rhapsody](#23-ibm-rhapsody)   
    -[2.3.1 IBM Rhapsody Overview](#231-ibm-rhapsody-overview)      
    -[2.3.2 IBM Rhapsody Diagrams](#232-ibm-rhapsody-diagrams)    
- [3. Comparison Matrix](#3-comparison-matrix)
- [4. Recommended Tool](#4-recommended-tool) 


## 1. Introduction 

### 1.1 Scope & Objectives
**Purpose:** Select a single standardized UML modeling tool for the SE25 project as the team transitions from architecture to detailed design (Weeks 3–4).  

**Artifacts covered:** Class, Sequence, and State Machine diagrams.  
**Deliverable:** This evaluation report and example diagrams exported from the selected tool.  



## 1.2 Tools Under Evaluation
1. **Eclipse Papyrus** — Open-source, standards-compliant UML tool integrated into the Eclipse IDE.  
2. **Modelio** — Open-source UML modeling environment supporting a wide range of diagrams and models.  
3. **IBM Rhapsody** — Professional Model-Based Systems Engineering (MBSE) tool. Academic licenses are available.


## 1.3 Evaluation Methodology
Each tool will be assessed according to the following:

  1. **Ease of Installation & Setup** (on Ubuntu 24.04 LTS)  
  2. **Learning Curve & Documentation** (user-friendliness and learning resources)  
  3. **UML Diagram Support & Standard Compliance** (Class, Sequence, State Machine)  
  4. **Integration with Project Workflow:**  
  5. **Performance & Stability**  
  6. **Licensing Terms**  

Each criterion will be scored on a **1–5 scale**



--- 

## 2. Detailed Evaluation


### 2.1 Eclipse Papyrus  

- #### 2.1.1 Eclipse Papyrus Overview   
  - **Overview:** Eclipse Papyrus is an open-source modeling tool based on the Eclipse platform. It supports full UML 2.x and SysML modeling, and allows the use of custom profiles. It’s suitable for academic and engineering projects, especially when working with complex systems. The tool is extensible through plug-ins and follows OMG standards.
  - **Installation (Ubuntu 24.04):** Papyrus is installed as a plug-in inside Eclipse (via Help → Install New Software). It requires Java JDK and some initial configuration. The installation process is manageable with the help of the official documentation. Once set up, it works well on Ubuntu 24.04 LTS.
  - **User Experience:** The interface is quite detailed and offers many features, but it can be difficult to use at first. Users need to understand how Eclipse handles modeling files (.uml, .notation, .di). Once familiar with the environment, it becomes easier to manage diagrams and models.
  - **UML Support:** Papyrus supports all major UML 2.x diagrams, including:
    - Class Diagrams
    - Sequence Diagrams
    - State Machine Diagrams
    - Use Case, Component, Deployment, Activity Diagrams
  - **Performance & Stability:** The tool runs stably on standard hardware. Models may slow down the interface, but during our usage, we didn’t encounter any crashes or major issues.
  - **Licensing:** Papyrus is fully open source under the Eclipse Public License (EPL). All features are available without restrictions.
  - **Integration with Workflow:**  
    - **C++ / Ubuntu integration:** Papyrus does not generate C++ code directly, but it can be used within Eclipse-based development environments. It’s useful for design and documentation purposes.
    - **Git file management:** The modeling files are XML-based. While they can be versioned with Git, the file diffs are not easily readable.
    - **Image export:** Diagrams can be exported as PNG, SVG, or PDF. The quality is good for documentation and wiki pages.


- #### 2.1.2 Eclipse Papyrus Diagrams





### 2.2 Modelio
 - #### 2.2.1 Modelio Overview  
    - **Overview:** _Modelio is a versatile open-source modeling tool that supports UML, BPMN, ArchiMate, and TOGAF standards. It offers enterprise-grade features such as requirements management, code generation (Java, C++, etc.), and documentation export. Its modular architecture and support for scripting make it highly extensible for complex modeling needs._  
    - **Installation (Ubuntu 24.04):** _Modelio installs easily on Ubuntu 24.04 LTS via Snap (sudo snap install modelio) with no additional configuration required. It runs smoothly out of the box and supports automatic updates, making it suitable for enterprise environments with minimal setup overhead._  
    - **User Experience:** _While Modelio’s interface may feel technical at first, it is well-documented and supported by a strong community. Tutorials and guides are available for both beginners and advanced users, making the learning curve manageable with some initial effort._  
    - **UML Support:** _Modelio provides full support for UML2 diagrams, including Class, Sequence, and State Machine diagrams. It enforces UML standards and allows for detailed modeling with validation rules, making it suitable for both academic and professional software design._  
    - **Performance & Stability:** _The application performs reliably on typical development machines and is not resource-intensive. It remains stable during extended modeling sessions and handles large models without noticeable lag or crashes._   
    - **Licensing:** _Modelio is fully open source under GPLv3 for its core, with additional modules licensed under Apache 2.0. There are no functional limitations in the free version, making it a strong candidate for teams seeking a cost-effective modeling solution._   
    - **Integration with Workflow:**  
      - **C++ / Ubuntu integration:** _Modelio supports code generation for C++ and runs natively on Ubuntu 24.04 LTS via Snap, making it compatible with typical Linux-based development environments. While it doesn't integrate directly with build systems like CMake or IDEs such as VS Code, its generated code can be easily incorporated into existing workflows. This makes it suitable for teams using C++ on Ubuntu, provided that Modelio is used primarily for design and documentation rather than full-cycle development._  
      - **Git file management:** _Modelio stores models in XML-based formats, which are lightweight and version-control friendly. Unlike binary-heavy tools, its files can be tracked efficiently in Git repositories without causing bloat. This allows for clean diffs, branching, and collaborative versioning, making Modelio a good fit for teams managing models alongside source code in GitHub or other Git platforms._  
      - **Image export:** _Modelio supports exporting diagrams to standard image formats including PNG, SVG, and PDF. The export quality is suitable for technical documentation, presentations, and Markdown-based wikis. This ensures that visual assets from Modelio can be seamlessly integrated into GitHub README files, project documentation, and collaborative design reviews._      
  



- #### 2.2.2 Modelio Diagrams  



        



### 2.3 IBM Rhapsody

- #### 2.3.1 IBM Rhapsody Overview   
  - **Overview:** _(Summary of features and enterprise capabilities.)_  
  - **Installation (Ubuntu 24.04):** _(Complexity, dependencies, setup notes.)_  
  - **User Experience:** _(Interface clarity, complexity, learning curve.)_  
  - **UML Support:** _(UML compliance, diagram richness, modeling depth.)_  
  - **Performance & Stability:** _(System load, stability, responsiveness)_   
  - **Licensing:** _(Academic license terms, renewal requirements)_   
  - **Integration with Workflow:**  
    - **C++ / Ubuntu integration:** _(Build integrations, code gen, workflow compatibility)_  
    - **Git file management:** _(Binary size, repo impact, versioning)_  
    - **Image export:** _(Quality, supported formats)_     

- #### 2.3.2 IBM Rhapsody Diagrams  

  
  
  
  

--- 

## 3. Comparison Matrix


| Criterion | Papyrus | Modelio | IBM Rhapsody |
|---|:--:|:--:|:--:|
| Ease of Installation & Setup | _ | 5 | _ |
| Learning Curve & Documentation | _ | 4 | _ |
| UML Diagram Support & Compliance | _ | 5 | _ |
| Integration: C++/Ubuntu | _ | 4 | _ |
| Integration: Git friendliness | _ | 4 | _ |
| Integration: Export PNG/SVG | _ | 5 | _ |
| Performance & Stability | _ | 5 | _ |
| Licensing | _ | 5 | _ |
| **Total (sum)** | **_** | **37** | **_** |





--- 

## 4. Recommended Tool

**Selected Tool:** **_[Name]_** 

**Concise Justification:**  
- _(Criterion 1)_  
- _(Criterion 2)_  
- _(Criterion 3)_  

