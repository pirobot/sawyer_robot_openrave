#!/usr/bin/env python

import os
import openravepy
import numpy as np
import time

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Error)
env = openravepy.Environment()
plugin = openravepy.RaveCreateModule(env, "urdf")
env.SetViewer('qtcoin')

with env:
	name = plugin.SendCommand('load sawyer_fred.urdf sawyer_base_fred.srdf')
	robot = env.GetRobot(name)

robot.SetActiveManipulator('arm')

ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
        robot, iktype=openravepy.IkParameterization.Type.Transform6D
)

if not ikmodel.load():
        ikmodel.autogenerate()

m = robot.GetActiveManipulator()
robot.SetActiveDOFs(m.GetArmIndices())

while True:
        joint_values = np.random.uniform(low=-1.57, high=1.57, size=(7,)) # Pick a random set of joint values
        
        print "Joint values:", joint_values

        robot.SetDOFValues(joint_values, [0, 1, 2, 3, 4, 5, 6]) # Set the joint values

        Tee = ikmodel.manip.GetEndEffectorTransform() # Get the resulting end effector transformation

        # Compute all IK solutions for current end effector pose
        try:
                start = time.time()
                sols = ikmodel.manip.FindIKSolutions(Tee, openravepy.IkFilterOptions.CheckEnvCollisions)
                elapsed = time.time() - start
                n_sols = len(sols)
                print "Number of solutions:", n_sols, 'in', elapsed, 'seconds,', elapsed / n_sols, 's per solution' 
        except:
                print "Can't find solution!"


        for sol in sols[::10]: # go through every 10th solution
                try:
                        robot.SetDOFValues(sol, ikmodel.manip.GetArmIndices()) # set the current solution
                        env.UpdatePublishedBodies() # allow viewer to update new robot
                        time.sleep(0.25)
                        #raw_input('press any key')
                except IndexError, e:
                        pass
                



