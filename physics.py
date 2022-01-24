from typing import Iterable

import numpy as np
from numba import njit, jit
from scipy.optimize import fsolve



@njit
def thrustEqn(vi, *prop_params):
    
    # Unpack parameters
    R,A,rho,a,b,c,eta,theta0,theta1,u,v,w,Omega = prop_params
    
    # Calculate local airflow velocity at propeller with vi, V'
    Vprime = np.sqrt(u**2 + v**2 + (w - vi)**2)
    
    # Calculate Thrust averaged over one revolution of propeller using vi
    Thrust = 1/4 * rho * a * b * c * R * \
        ( (w - vi) * Omega * R + 2/3 * (Omega * R)**2 * (theta0 + 3/4 * theta1) + \
          (u**2 + v**2) * (theta0 + 1/2 * theta1) )
    
    # Calculate residual for equation: Thrust = mass flow rate * delta Velocity
    residual = eta * 2 * vi * rho * A * Vprime - Thrust
    return residual



def thrust(speed, airstream_velocity, R, A, rho, a, b, c, eta, theta0, theta1) -> float:
    u, v, w = airstream_velocity
    # Convert commanded RPM to rad/s
    Omega = 2 * np.pi / 60 * speed
    
    #Collect propeller config, state, and input parameters
    prop_params = (R,A,rho,a,b,c,eta,theta0,theta1,u,v,w,Omega)
    
    # Numerically solve for propeller induced velocity, vi
    # using nonlinear root finder, fsolve, and prop_params
    # TODO: numba jit gives error for this function ('Untyped global name fsolve')
    vi = fsolve(thrustEqn, 0.1, args=prop_params)
    
    # Plug vi back into Thrust equation to solve for T
    Vprime = np.sqrt(u**2 + v**2 + (w - vi)**2)
    Thrust = eta * 2 * vi * rho * A * Vprime
    return Thrust



@njit
def torque(position_vector: np.ndarray, thrust: float) -> np.ndarray:
    thrust = np.asarray([0, 0, -thrust])
    return np.cross(position_vector, thrust)



@njit
def apply_forces_torques(forces: np.ndarray, torques: np.ndarray, x: np.ndarray,
    g: float, mass: float, inertia_matrix: np.matrix, inertia_matrix_inverse: np.matrix):
    # Store state variables in a readable format
    ub = x[0]
    vb = x[1]
    wb = x[2]
    p = x[3]
    q = x[4]
    r = x[5]
    phi = x[6]
    theta = x[7]
    psi = x[8]
    xI = x[9]
    yI = x[10]
    hI = x[11]
    
    # Pre-calculate trig values
    cphi = np.cos(phi);   sphi = np.sin(phi)
    cthe = np.cos(theta); sthe = np.sin(theta)
    cpsi = np.cos(psi);   spsi = np.sin(psi)

    fx, fy, fz = forces
    tx, ty, tz = torques
    I = inertia_matrix
    I_inv = inertia_matrix_inverse
    
    # Calculate the derivative of the state matrix using EOM
    xdot = np.zeros_like(x)
    
    xdot[0] = -g * sthe + r * vb - q * wb  # = udot
    xdot[1] = g * sphi * cthe - r * ub + p * wb # = vdot
    xdot[2] = 1/mass * (fz) + g * cphi * cthe + q * ub - p * vb # = wdot


    # xdot[3] = 1/Ixx * (tx + (Iyy - Izz) * q * r)  # = pdot
    # xdot[4] = 1/Iyy * (ty + (Izz - Ixx) * p * r)  # = qdot
    # xdot[5] = 1/Izz * (tz + (Ixx - Iyy) * p * q)  # = rdot
    xdot[3:6] = I_inv @ (torques - np.cross(x[3:6], I @ x[3:6]))

    xdot[6] = p + (q*sphi + r*cphi) * sthe / cthe  # = phidot
    xdot[7] = q * cphi - r * sphi  # = thetadot
    xdot[8] = (q * sphi + r * cphi) / cthe  # = psidot
    
    xdot[9] = cthe*cpsi*ub + (-cphi * spsi + sphi*sthe*cpsi) * vb + \
        (sphi*spsi+cphi*sthe*cpsi) * wb  # = xIdot
        
    xdot[10] = cthe*spsi * ub + (cphi*cpsi+sphi*sthe*spsi) * vb + \
        (-sphi*cpsi+cphi*sthe*spsi) * wb # = yIdot
        
    xdot[11] = (-sthe * ub + sphi*cthe * vb + cphi*cthe * wb) # = zIdot
    
    return xdot
