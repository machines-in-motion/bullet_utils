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

from bullet_utils.wrapper import PinBulletWrapper

class RobotSimulator(PinBulletWrapper):
    def __init__(self, physics_client=None):

        # Get the pointer tower a physics client.
        if physics_client is None:
            self.physics_client = PinBulletWrapper.initialize_physics_client()
        else:
            self.physics_client = physics_client

        # Load the ground.
        # may raise PackageNotFoundError
        self.ground_urdf = path.join(
            get_package_share_directory("pinocchio_bullet"),
            "urdf",
            "ground.urdf",
        )
        self.ground_id = pybullet.loadURDF(self.ground_urdf)

        # Load the robot
        self.start_pose = [0.0, 0.0, 0.4]
        self.start_orientation = pybullet.getQuaternionFromEuler([0, 0, 0])
        self.robot_urdf = path.join(
            get_package_share_directory("pinocchio_bullet"),
            "urdf",
            "solo12.urdf",
        )
        self.robot_meshes_path = path.dirname(
            get_package_share_directory("pinocchio_bullet"))
        assert path.exists(self.robot_urdf)
        assert path.exists(self.robot_meshes_path)
        print("robot_urdf: ", self.robot_urdf)
        print("robot_meshes_path: ", self.robot_meshes_path)
        self.robot_id = pybullet.loadURDF(
            self.robot_urdf,
            self.start_pose,
            self.start_orientation,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=False,
        )

        # Create the robot wrapper in pinocchio.
        self.pinocchio_robot = pinocchio.robot_wrapper.RobotWrapper.BuildFromURDF(
            self.robot_urdf,
            package_dirs=[self.robot_meshes_path, "."],
            root_joint=pinocchio.JointModelFreeFlyer(),
            verbose=True
        )

        # Query all the joints.
        self.num_joints = pybullet.getNumJoints(self.robot_id)

        for ji in range(self.num_joints):
            pybullet.changeDynamics(
                self.robot_id,
                ji,
                linearDamping=0.04,
                angularDamping=0.04,
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.base_link_name = "base_link"
        controlled_joints = []
        for leg in ["FL", "FR", "HL", "HR"]:
            controlled_joints += [leg + "_HAA", leg + "_HFE", leg + "_KFE"]
        self.joint_names = controlled_joints

        # Creates the wrapper by calling the super.__init__.
        super(RobotSimulator, self).__init__(
            self.robot_id,
            self.pinocchio_robot,
            controlled_joints,
            ["FL_ANKLE", "FR_ANKLE", "HL_ANKLE", "HR_ANKLE"],
        )


if __name__ == "__main__":
    np.set_printoptions(precision=2, suppress=True)

    # Setup pybullet for the quadruped and a wrapper to pinocchio.
    robot = RobotSimulator()

    # Get the current state and modify the joints to have the legs
    # bend inwards.
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
        pybullet.stepSimulation()
        time.sleep(0.001)

    # Print the final active force frames and the forces
    force_frames, forces = robot.get_force()

    print("Active force_frames:", force_frames)
    print("Corresponding forces:", forces)
