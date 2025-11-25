# 1D Gantry Tracking & Laser Activation System

This project implements a complete control pipeline for a **1-Dimensional (1D) gantry system** capable of:

- Recognizing and tracking visual targets using **image processing**
- Aligning a gantry controlled by a stepper motor to the detected target position
- Activating a **laser** when aligned
- Communicating with a **Raspberry Pi** running a **Simulink** control program that drives a stepper motor and laser hardware

The repository is hosted at:  
**https://github.com/gw680-bath/ME32088_1D_Gantry**

The system integrates **computer vision**, **motion control**, and **embedded hardware** into a unified automation workflow.

---

## 🚀 Features

### 🔍 Image Processing & Target Tracking
- Detect targets using computer vision  
- Track targets in real time  
- Convert pixel coordinates → physical gantry coordinates  
- Provide alignment instructions to the motor controller  

### 🛠️ Gantry Control
- Stepper motor control via Raspberry Pi + Simulink  
- Direction, Step (Pulse), and Enable line management  
- Position estimation through step-counting  
- Homing and calibration routines  

### 💥 Laser Activation
- Automatically triggers laser when gantry is aligned  
- Configurable alignment thresholds  

### 🔗 Raspberry Pi & Simulink Integration
- Python → Raspberry Pi communication layer  
- Sends target position and control commands  
- Receives motor status, step count, and alignment confirmation  
- Raspberry Pi runs a Simulink model for deterministic, hardware-level motor and laser control  

### 🔧 Modular Script Design
- Separate scripts for image processing, control logic, hardware communication, and Simulink integration  

---

## 📁 Project Structure

The project folder is named **Python_1D_Gantry**.

A typical structure may look like:

[yet to be decided...]

## Create & Activate a virtual environment

python -m venv 1D_Gantry.venv

## Windows
1D_Gantry.venv\Scripts\activate

## Mac/Linux
source 1D_Gantry.venv/bin/activate


### Install Dependencies
pip install -r requirements.txt