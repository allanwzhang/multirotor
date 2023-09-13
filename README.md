# multirotor and Ardupilot integration

Simulation of multi-rotor unmanned aerial vehicles in python.

This package provides an object-oriented interface for modeling and simulating motors, propellers, and airframe of a UAV.

## Installation

Clone repository and install for development. This will allow you to change the code of the package so the changes show up when you `import multirotor` in other projects.

```bash
git clone https://github.com/allanwzhang/multirotor.git
cd multirotor
pip install -e .

# to also install dependencies for building package/documentation
pip install -e .[DEV]
```

Make sure Ardupilot is installed. If not follow these instructions: [18.04](https://github.com/allanwzhang/python-gazebo/blob/main/installArdupilot18.md), [20.04](https://github.com/allanwzhang/python-gazebo/blob/main/installArdupilot20.md)

## Usage
### Start Ardupilot
From the ardupilot directory
```bash
cd ArduCopter
sim_vehicle.py -v ArduCopter -f octa --model JSON --console
# may need to run conda deactivate first
```
