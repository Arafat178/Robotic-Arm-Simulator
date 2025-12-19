# Constraint-Based Chess-Playing Robotic Arm Simulation  
### A SolidWorks API and Python-Based Approach

## Abstract
This project presents a **constraint-based robotic arm simulation** capable of
performing chess piece pick-and-place operations.
The system integrates **inverse kinematics**, **parametric CAD control**, and
**mate-based grasp modeling** using the SolidWorks API and Python.
Unlike conventional robotic simulations that rely on external frameworks such as ROS,
this work demonstrates that **CAD environments can be effectively utilized as
motion visualization and control platforms** for educational and research purposes.

---

## 1. Introduction
Robotic manipulation tasks typically require kinematic modeling, motion planning,
and grasp control.
Most educational and research implementations rely on robotics middleware and
physics-based simulators.
However, such frameworks can obscure fundamental principles for early-stage learners.

This project explores an alternative approach in which **SolidWorks is employed
as a parametric motion environment**, while all kinematic computation and sequencing
are performed externally using Python.
A chess-playing task is selected as a structured and repeatable benchmark for
pick-and-place manipulation.

---

## 2. System Overview

The system consists of:
- a 4-degree-of-freedom articulated robotic arm modeled in SolidWorks
- a Python-based control layer
- constraint-based grasp modeling using CAD mates

A high-level overview of the control pipeline is shown below:

1. Chess square identification
2. Coordinate mapping to Cartesian space
3. Inverse kinematics computation
4. Joint-level motion execution via SolidWorks parameters
5. Grasp simulation using mate activation
6. Collision-safe motion sequencing

---

## 3. Kinematic Modeling

### 3.1 Robot Configuration
The robotic arm is modeled as a 4-DOF serial manipulator consisting of:
- a revolute base joint
- two planar revolute joints
- a wrist joint maintaining vertical end-effector orientation

### 3.2 Inverse Kinematics
Inverse kinematics are derived geometrically by reducing the 3D problem to a planar
two-link mechanism after base rotation.
The end-effector is constrained to remain vertically oriented throughout the motion,
simplifying grasp alignment and placement.

All joint angles are computed in Python and transmitted to SolidWorks by updating
corresponding angular parameters.

---

## 4. Chessboard Mapping

Each chessboard square is mapped to a Cartesian coordinate based on:
- known square dimensions
- fixed board placement relative to the robot base
- square center alignment

This deterministic mapping enables direct conversion from chess notation
(e.g., `a4`, `f2`) to robotic target positions.

---

## 5. Motion Planning and Sequencing

### 5.1 Joint Activation Strategy
To achieve smooth and realistic motion, different joint activation sequences are
used for approach and retraction phases:
- **Approach sequence:** base → wrist → elbow → shoulder
- **Retraction sequence:** shoulder → elbow → wrist → base

This strategy reduces unnecessary sweeping motion and improves visual clarity.

### 5.2 Collision Avoidance
Collision avoidance is enforced using a **Z-first motion strategy**.
All lateral (X–Y) motion is restricted to a predefined safe height above the board,
while vertical motion is performed only directly above the target square.

---

## 6. Grasp Modeling Using Mates

Grasping is simulated using **concentric and coincident mates** between the
end-effector and the chess piece.
Rather than physically modeling contact forces, grasp engagement and release are
achieved by activating and suppressing these mates programmatically.

This approach represents an **ideal rigid grasp assumption**, suitable for
visualization and control logic validation.

---

## 7. Implementation

### 7.1 Software Architecture

src/
├── main.py # System entry point


### 7.2 Tools and Libraries
- Python
- SolidWorks API (via win32com)
- NumPy (for interpolation)

---

## 8. Experimental Demonstration

The system is demonstrated through a complete chess move execution
(e.g., `a6 → f2`), including:
- approach
- grasp
- lift
- transport
- placement
- retraction

Motion behavior is observed directly within the SolidWorks assembly environment.

---

## 9. Limitations

- No force, torque, or compliance modeling
- No sensor or vision feedback
- No physics-based collision detection
- Idealized rigid grasp assumption

These limitations are acceptable within the scope of this conceptual and
educational study.

---

## 10. Future Work

Potential extensions of this work include:
- forward kinematics validation
- basic collision envelope modeling
- integration with vision systems
- migration to ROS for real hardware deployment
- incorporation of compliant gripper models

---

## 11. Conclusion
This project demonstrates that **SolidWorks can be effectively repurposed as a
robotic motion visualization and constraint environment** when combined with
external kinematic computation.
The proposed approach provides a clear and accessible platform for understanding
robotic manipulation fundamentals without reliance on complex middleware.

---

## Author
**Arafat**  
Mechanical Engineering Student

---

## License
This project is intended for academic and educational use.
