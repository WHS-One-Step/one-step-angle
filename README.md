# One-Step-Angle:
This repository contains the official source code pertaining to the calculation of the knee-flexion angle for team One-Step.

## Directories:
- bootstrap: Contains bootstrapping Linux scripts to quickly install dependencies related to the Operating System and Python Virtual Environment.
- core: Contains the core components of the project, mainly the Tracker class that performs calculations on data provided by two IMUs.

## Prerequisites:
* A Linux Operating System that utilizes the **APT** package manager, as that is what the scripts are configured to use to install the appropriate libphidget bindings.
* Two PhidgetSpatial 3/3/3 IMUs, with one on the thigh and one on the shank.

* The Python executable language installed with an appropriate version, e.g 3.10.
```bash
# In case Python is not preinstalled on the Raspberry Pi:
sudo apt install python3
```

## Installation:
Execute the following commands in the Linux terminal to install the repository, dependencies for the Raspberry Pi, and create a Python Virtual Environment:
```bash
# Clone the repository:
git clone https://www.github.com/whs-one-step/one-step-angle && cd ./one-step-angle/

chmod +x ./bootstrap/* # Enable the executable bit
./bootstrap/bootstrap-raspberry.sh # Install dependencies for the Raspberry Pi
./bootstrap/bootstrap-environment.sh # Create the Python Virtual Environment and install dependencies

# Execution:
venv/bin/python main.py
```

## Purpose:
The purpose of this repository was to calculate the knee-flexion angle that would later be mapped into a pulse modulation value and then encoded into binary. The angle is calculated by taking the dot product of two acceleration vectors and performing appropriate trigonometry.
- Acceleration is used as orientation as it is the perceived tilt of the device relative to gravity, and gyroscopic vectors are especially prone to drift. The math can be viewed here:
```python
# Variables (Assignment):
# Dot:
dot_product = np.dot(self.thigh_orientation, self.shin_orientation)

# Range:
dot_product = min(1.0, max(-1.0, dot_product))

# Offset:
angle = degrees(acos(dot_product)) + self.calibration_offset

# Range:
angle = min(180.0, max(0.0, angle))
```

## Framework:
There are no frameworks used in this repository. A framework is defined as a set of tools to help develop a product in a specific way. This project was made with total creative-liberty, and structured in a way as to provide modularity and reuseable code.
- Phidget is considered as a library, as it provided bindings to communicate with the IMUs, however it didn't force us to structure our project in a specific way.

# Language:
The primary language used in this repository is Python, however this repository is currently outdated and does not use C-optimizations, such as the math optimizations in the [one-step-optimizations](https://www.github.com/whs-one-step/one-step-optimizations) repository.
Python permitted extreme development productivity, streamlining development experiences and advancements in the project. Constant improved prototypes were able to produced due to the simplicity of Python as a language, however optimizations were needed to reduce latency and increase performance.
