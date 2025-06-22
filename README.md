# Auto-Multilift_simulation

## Introduction

This branch provides parallel implementations of [Auto-Multilift](https://github.com/RCL-NUS/Auto-Multilift/), introducing two specific approaches—ROS topic and ROS service—to enable collaborative Model Predictive Control (MPC) solving and communication among multiple agents (multi-drones). These methods are designed to achieve faster parallel computation through enhanced cooperation.

> **Note:** This implementation is currently under active development and may contain unresolved issues.

## Prerequisites

### Auto-Multilift Dependencies

Before running the source code, please ensure that the following packages are installed with the specified versions:

- [CasADi](https://web.casadi.org/): 3.5.5  
- [ACADOS](https://github.com/acados/acados) (**stable version v0.3.5**; see [documentation](https://docs.acados.org/)).  
  **Important:** Do **not** use the latest version of ACADOS.
- [NumPy](https://numpy.org/): 1.23.0  
- [PyTorch](https://pytorch.org/): 1.12.0+cu116  
- [Matplotlib](https://matplotlib.org/): 3.3.0  
- [Python](https://www.python.org/): 3.9.12  
- [SciPy](https://scipy.org/): 1.8.1  
- [Pandas](https://pandas.pydata.org/): 1.4.2  
- [scikit-learn](https://scikit-learn.org/stable/whats_new/v1.0.html): 1.0.2  

After installing the above packages, you can validate your environment by running the following commands in the numerical simulation environment:
```
cd src/px4-offboard/px4_offboard/
python3 parallel_distributed_autotuning_evaluation_acados.py
```

### Building the ROS2 Workspace

Follow the [official PX4 ROS2 user guide](https://docs.px4.io/main/en/ros2/user_guide.html#installation-setup) to set up the ROS2 workspace.  
**Note:** During setup, ensure you install PX4 version **v1.14.3** using the following command:
```
git clone -b v1.14.3 https://github.com/PX4/PX4-Autopilot.git --recursive
```

### IsaacSim Installation

Use the [installation script](https://github.com/Temasek-Dynamics/SimulatorSetup) provided by the Temasek-Dynamics Laboratory to set up IsaacSim and PegasusSimulator.

After installation, edit the following configuration file:
```
cd path/to/SimulatorSetup/submodules/PegasusSimulator/extensions/pegasus.simulator/config/configs.yaml
```
Update the configuration as follows:
```
px4_default_airframe: none_iris
px4_dir: path/to/PX4-Autopilot   # v1.14.3, as installed above
```

## Run this project

After completing the installation and setup steps above, you can launch the simulation using the provided `.sh` scripts.

## References

- [Auto-Multilift](https://github.com/RCL-NUS/Auto-Multilift/)
- [PX4 Control Diagram](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html)
- [px4_offboard (Offboard Demo Source)](https://github.com/Jaeyoung-Lim/px4-offboard)
- [ROS 2 Offboard Control Example](https://docs.px4.io/main/en/ros2/offboard_control.html)