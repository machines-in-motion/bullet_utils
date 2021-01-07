"""wrapper

Pybullet interface using pinocchio's convention.

License: BSD 3-Clause License
Copyright (C) 2018-2021, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""
try:
    # use standard Python importlib if available (Python>3.7)
    import importlib.resources as importlib_resources
except ImportError:
    import importlib_resources
import pybullet
import pinocchio
import numpy as np
import time
from pinocchio.utils import zero


class BulletEnv(object):
    def __init__(self, server=pybullet.GUI, dt=0.001):
        self.dt = dt
        self.objects = []
        self.robots = []

        self.physics_client = pybullet.connect(server)
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
        
    def add_robot(self, RobotWrapper, pos=None, orn=None, useFixedBase=False):
        robot = RobotWrapper(pos, orn, useFixedBase)
        self.robots.append(robot)
        return robot
        
    def add_object_from_urdf(self, urdf_path, 
                             pos=[0,0,0], orn=[0,0,0,1],
                             useFixedBase=True):
        # Load the object.
        object_id = pybullet.loadURDF(urdf_path, useFixedBase=useFixedBase)
        pybullet.resetBasePositionAndOrientation(object_id, pos, orn)
        self.objects.append(object_id)
        return object_id

    def start_recording(self, file_name):
        self.file_name = file_name
        pybullet.startStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)

    def stop_recording(self):
        pybullet.stopStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)
   
    def step(self, sleep=True):
        if sleep:
            time.sleep(self.dt)
        pybullet.stepSimulation()

    def print_physics_engine_params(self):
        params = pybullet.getPhysicsEngineParameters(self.physicsClient)
        print("physics_engine_params:")
        for key in params:
            print("    - ", key, ": ", params[key])


class BulletEnvWithGround(BulletEnv):
    def __init__(self, server=pybullet.GUI, dt=0.001):
        super().__init__(server, dt)
        with importlib_resources.path(__package__, "env.py") as p:
            package_dir = p.parent.absolute()
        plane_urdf = str(package_dir/"resources"/"plane_with_restitution.urdf")
        self.add_object_from_urdf(plane_urdf)
