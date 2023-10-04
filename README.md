# Batch Synthesis Module


## Introduction
<p align="center">
  <img src="img\BatchSynthesisPlatform_architecture.png" width="70%" height="70%" />
</p>

This repository contains source code of batch synthesis hardware for nanoparticle synthesis. It follows behind robotic settings. This system is controlled by [MasterPlatform](https://github.com/KIST-CSRC/MasterPlatform](https://github.com/KIST-CSRC/BespokeSynthesisPlatform)

## Device settings

- Vial storage (Arduino Uno + Servo motor)
- Syringe pump (Tecan cavro Centris)
- Stirrer (IKA, RET Control Visc)
- Robotic arm (Doosan robotics, M0609) + Gripper (OnRobot, RG2)
- XYZ actuator (Science Town)
- Stock solution (Daihan scientific company)
- Ice bucket (Daihan scientific company)

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
BatchSynthesisModule
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
│   └── LA_Server_Connection.cpp
│   └── Linear_Actuator.cpp
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

"# Batch Synthesis Module" 
