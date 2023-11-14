from pprint import pprint as print

import matplotlib.pyplot as plt
import numpy as np
from tqdm.auto import tqdm, trange
from multirotor.helpers import DataLog
from multirotor.vehicle import MotorParams, VehicleParams, PropellerParams, SimulationParams, BatteryParams
from multirotor.simulation import Multirotor
from multirotor.coords import inertial_to_body
from multirotor.env import SpeedsMultirotorEnv
from multirotor.visualize import plot_datalog

import socket
import time
import json
import struct
from multirotor.coords import direction_cosine_matrix, body_to_inertial 

# Tarot T18 params
bp = BatteryParams(max_voltage=25.2)
mp = MotorParams(
    moment_of_inertia=5e-5,
    # resistance=0.27,
    resistance=0.081,
    k_emf=0.0265,
    # k_motor=0.0932,
    speed_voltage_scaling= 0.0325,
    # speed_voltage_scaling= 0.025,
    max_current=38.
)
pp = PropellerParams(
    moment_of_inertia=1.85e-6,
    use_thrust_constant=True,
    k_thrust=9.8419e-05, # 18-inch propeller
    # k_thrust=5.28847e-05, # 15 inch propeller
    k_drag=1.8503e-06, # 18-inch propeller
    # k_drag=1.34545e-06, # 15-inch propeller
    motor=mp #try motor = none
)
vp = VehicleParams(
    propellers=[pp] * 8,
    battery=bp,
    angles=np.array([0.5, 1.5, 0.25, 1.75, 0.75, 1.25, 1, 0])*np.pi,
    distances=np.ones(8) * 0.635,
    clockwise=[1,1,-1,-1,-1,-1,1,1],
    mass=10.66,
    inertia_matrix=np.asarray([
        [0.2206, 0, 0],
        [0, 0.2206, 0.],
        [0, 0, 0.4238]
    ])
)
sp = SimulationParams(dt=0.0025, g=9.81)

def changeToJSONString(curr_time, state, accel):
    phys_time = curr_time
    pos = state[0:3].tolist()
    velo = state[3:6]
    euler = state[6:9].tolist()
    gyro = state[9:12].tolist()
    accel = accel.tolist()

    # change reference frame / coordinates to make AP requirements
    pos[2] = -1*pos[2]
    pos[0], pos[1] = pos[1], pos[0]

    dcm = direction_cosine_matrix(*euler)
    v_inertial = body_to_inertial(velo, dcm).tolist()
    v_inertial[2] = -1*v_inertial[2]
    v_inertial[0], v_inertial[1] = v_inertial[1], v_inertial[0]

    gyro[0], gyro[1] = gyro[1], gyro[0]
    gyro[2] = -1*gyro[2]
    euler[0], euler[1] = euler[1], euler[0]
    euler[2] = -1*euler[2]

    # Build JSON format
    IMU_fmt = {
        "gyro" : gyro,
        "accel_body" : accel
    }
    JSON_fmt = {
        "timestamp" : phys_time,
        "imu" : IMU_fmt,
        "position" : pos,
        "attitude" : euler,
        "velocity" : v_inertial
    }

    JSON_string = "\n" + json.dumps(JSON_fmt,separators=(',', ':')) + "\n"

    return JSON_string

def send_velocity(vx, vy, vz, client_socket, server_address):
    message = str(vx)+","+str(vy)+","+str(vz)
    message = message.encode('utf-8')
    client_socket.sendto(message, server_address)

def ap_sim(env, sock, steps=600000, disturbance=None):
    ap_log = DataLog(env.vehicle, other_vars=("propeller_speed",))
    command_logger = {}

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = ('127.0.0.1', 1234)
    disturb_force, disturb_torque = 0., 0

    curr_time = 0  # Get the current time in seconds
    RATE_HZ = 400 # should be 400 (rate of sending data to AP)
    TIME_STEP = 1/RATE_HZ
    last_frame = -1
    frame_count = 0

    for i in range(0, steps):
        try:   
            # Retrieve data from AP
            data, addr = sock.recvfrom(100)
            parse_format = 'HHI16H'
            magic = 18458
            if len(data) != struct.calcsize(parse_format):
                print("got packet of len %u, expected %u" % (len(data), struct.calcsize(parse_format)))
                continue
            unpacked_data = struct.unpack(parse_format, data)
            if magic != unpacked_data[0]:
                print("Incorrect protocol magic %u should be %u" % (unpacked_data[0], magic))
                continue

            frame_rate = unpacked_data[1]
            frame_count = unpacked_data[2]
            pwm = np.array(unpacked_data[3:11]) # Array of motor commands recieved

            string_action = " ".join(str(element) for element in pwm.tolist())

            if frame_count < last_frame:
                print('Reset controller')
            elif frame_count == last_frame:
                continue
            if frame_count != last_frame + 1 and last_frame != 0:
                print("Missed %u frames" % (frame_count - last_frame))
                continue
            last_frame = frame_count
            
            curr_time += TIME_STEP

            action = (pwm-1000)*0.575 # Translate PWM ESC to motor speeds in rpm
    
            string_action = " ".join(str(element) for element in action.tolist())

            if disturbance is not None and action[0] != 0 and env.vehicle.position[2] > 9.5:
                disturb_force = disturbance(i, env.vehicle)
                send_velocity(curr_time*10, 0, 0, client_socket, server_address)

            # Find new state of the vehicle given these commands
            state, *_ = env.step(
                action, disturb_forces=disturb_force, disturb_torques=disturb_torque
            )

            # Calculate accel and put data into required JSON format
            accel = env.vehicle.dxdt_speeds(0, state, action, disturb_forces=disturb_force, disturb_torques=disturb_torque)[3:6]
            accel[2] = -9.8
            # if disturbance is not None:
            #     accel[2] += disturb_force[2] # Change accel z frame to body
            JSON_string = changeToJSONString(curr_time, state, accel)

            # Send JSON of new state back to AP
            sock.sendto(bytes(JSON_string,"ascii"), addr)
        except json.JSONDecodeError:
            print("Invalid JSON data received")
        except socket.timeout:
            continue
        except KeyboardInterrupt:
            sock.close()
        except Exception as e:
            if not isinstance(e, OSError):
                raise e
            sock.close()
            break
    
    ap_log.done_logging()   
    
    return ap_log, command_logger

def wind(t, m):
    return np.array([50, 0, 0])

def start_program(vp, sp, UDP_IP="127.0.0.1", UDP_PORT=9002, wind=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.1)  # Set a timeout value of 1 second on the socket object

    env = SpeedsMultirotorEnv(vehicle=Multirotor(vp, sp)) # step() takes speed signals
    env.reset(np.zeros(12))
    ap_log, action_log = ap_sim(env, sock, steps=600000, disturbance=wind) # Can add disturbance if needed (like wind)

    sock.close()

# Tarot T18 params
bp = BatteryParams(max_voltage=25.2)
mp = MotorParams(
    moment_of_inertia=5e-5,
    # resistance=0.27,
    resistance=0.081,
    k_emf=0.0265,
    # k_motor=0.0932,
    speed_voltage_scaling= 0.0325,
    # speed_voltage_scaling= 0.025,
    max_current=38.
)
pp = PropellerParams(
    moment_of_inertia=1.85e-6,
    use_thrust_constant=True,
    k_thrust=9.8419e-05, # 18-inch propeller
    # k_thrust=5.28847e-05, # 15 inch propeller
    k_drag=1.8503e-06, # 18-inch propeller
    # k_drag=1.34545e-06, # 15-inch propeller
    motor=mp #try motor = none
)
vp = VehicleParams(
    propellers=[pp] * 8,
    battery=bp,
    angles=np.array([0.5, 1.5, 0.25, 1.75, 0.75, 1.25, 1, 0])*np.pi,
    distances=np.ones(8) * 0.635,
    clockwise=[1,1,-1,-1,-1,-1,1,1],
    mass=10.66,
    inertia_matrix=np.asarray([
        [0.2206, 0, 0],
        [0, 0.2206, 0.],
        [0, 0, 0.4238]
    ])
)
sp = SimulationParams(dt=0.0025, g=9.81)

start_program(vp, sp, wind=wind)