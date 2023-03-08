# BatchSynthesisPlatform


## Introduction
<p align="center">
  <img src="img\BatchSynthesisPlatform_architecture.png" width="70%" height="70%" />
</p>

This repository contains source code of batch synthesis hardware for nanoparticle synthesis. It follows behind robotic settings. This system is controlled by [MasterPlatform](https://github.com/KIST-CSRC/MasterPlatform)

## Robotics settings

- Vial storage (Arduino Uno + Servo motor)
- Syringe pump (Tecan cavro Centris)
- Stirrer (IKA, RET Control Visc)
- Robotic arm (Doosan robotics, M0609) + Gripper (OnRobot, RG2)
- Linear actuator (Science Town)
- Stock solution bottle (Daihan scientific company)

## Video

### Vial storage

### Stirrer

### Linear actuator

### Syringe pump


## Installation

**Using conda**
```bash
conda env create -f requirements_conda.txt
```
**Using pip**
```bash
pip install -r requirements_pip.txt
```

## Script architecture
```
BatchSynthesisPlatform
├── BaseUtils
│   └── Preprocess.py
│   └── TCP_Node.py
├── Chemical_Storage
│   └── Vial_Storage.py
├── doosan-robot
│   └── Robot_Arm (followed by Doosan-robotics)
│   └── RobotServer.py
├── img
├── Linear_Actuator_client
│   └── Linear_Actator_Class.py
├── Linear_Actuator_server
│   └── AXL.dll
│   └── configuration.mot
│   └── EzBasicAxl.dll
│   └── LA_Server_Connection.cpp
│   └── Linear_Actuator.cpp
│   └── manual.txt
│   └── Linear_Actuator_server.sln
│   └── Source.cpp
│   └── TCPServer.cpp
│   └── TCPServer.h
├── Log
│   └── Logging_Class.py
├── Stirrer
│   └── serial_labware
│       └── serial_labware.py
│   └── hotplate.py
│   └── IKA_RET_Control_Visc.py
├── Syringe_Pump
│   └── Syringe_Pump_Package
│       └── syringe.py
│       └── tecanapi.py
│       └── transport.py
│   └── Syringe_Class.py
└── BatchSynthesisServer.py
```

## Reference
Please cite us if you are using our model in your research work: <br />

"# BatchSynthesisPlatform" 
