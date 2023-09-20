# multirotor and Ardupilot integration

Simulation of multi-rotor unmanned aerial vehicles in Python. This package provides an object-oriented interface for modeling and simulating motors, propellers, and airframe of a UAV. Furthermore, an Ardupilot integration implementation is provided through their [SITL JSON backend](https://github.com/ArduPilot/ardupilot/tree/6cb4e6b31da44c0e7531c7f0d60e2880ced11392/libraries/SITL/examples/JSON).

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
Note: Make sure X forwarding application is on (if running on windows os)
### Start Python
From multirotor directory, open up ArdupilotPython.ipynb file and run blocks to start the connection.
### Control Parameters
After all three parts are running, load Octocopter parameters from the ArduPilot console. There should be an parameters button where you can then choose to load a file. Choose simOcto.parm in the multirotor directory. You will only need to do this once (parameters should save automatically).
### Starting the drone
In the console where Ardupilot is running, enter these commands after everything loads and connects. Look for these two commands to appear in the ArduPilot console (IMU0 is using GPS, IMU1 is using GPS), then run these commands:
```bash
mode guided
arm throttle
takeoff 10
```
### Using a ground control system
To run more advanced flights, you can download a Ground Control system.  
IQ_GNC Tutorial: [QGroundControl](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_qgc.md)  
Instead of running commands from terminal, you can set flights from the ground control system instead.
### Logging
You can run the blocks in the ArduPilotPython.ipynb file to see the logs of the drone flight.
