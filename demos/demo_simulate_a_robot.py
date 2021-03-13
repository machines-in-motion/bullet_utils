#!/usr/bin/env python

"""simulate_a_robot

Example for using the PinBulletWrapper for a quadruped robot.

license: BSD 3-Clause License
copyrights: Copyright (C) 2018-2021, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.

"""

import os
from os import path
import numpy as np
import time
import pybullet
import pinocchio

from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo8wrapper import Solo8Robot
try:
    # use standard Python importlib if available (Python>3.7)
    import importlib.resources as importlib_resources
except ImportError:
    import importlib_resources

if __name__ == "__main__":
    np.set_printoptions(precision=2, suppress=True)
    env = BulletEnvWithGround(pybullet.GUI)
    robot = env.add_robot(Solo8Robot)
    q, dq = robot.get_state()

    # Update the simulation state to the new initial configuration.
    robot.reset_state(q, dq)

    # Run the simulator for 2000 steps = 2 seconds.
    for i in range(2000):
        # Get the current state (position and velocity)
        q, dq = robot.get_state()
        active_contact_frames, contact_forces = robot.get_force()

        # Alternative, if you want to use properties from the pinocchio robot
        # like the jacobian or similar, you can also get the state and update
        # the pinocchio internals with one call:
        #
        #  q, dq = robot.get_state_update_pinocchio()

        if i % 100 == 0:
            print("Forces:", active_contact_frames, contact_forces)

        # Compute the command torques at the joints. The torque
        # vector only takes the actuated joints (excluding the base)
        tau = np.zeros(robot.pinocchio_robot.nv - 6)

        # Send the commands to the robot.
        robot.send_joint_command(tau)

        # Step the simulator and sleep.
        env.step(sleep=True)

    # Print the final active force frames and the forces
    force_frames, forces = robot.get_force()

    print("Active force_frames:", force_frames)
    print("Corresponding forces:", forces)
