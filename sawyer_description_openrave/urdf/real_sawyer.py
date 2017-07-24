import os
import openravepy

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Error)
env = openravepy.Environment()
plugin = openravepy.RaveCreateModule(env, "urdf")
env.SetViewer('qtcoin')

with env:
	name = plugin.SendCommand('load real_sawyer_urdf.xml real_sawyer.srdf')
	robot = env.GetRobot(name)

robot.SetActiveManipulator('arm')

ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
	robot, iktype=openravepy.IkParameterization.Type.Transform6D
)

if not ikmodel.load():
	ikmodel.autogenerate()

m = robot.GetActiveManipulator()
robot.SetActiveDOFs(m.GetArmIndices())

home_pose = [
	[0.37, -0.20, 0.91, 0.5],
	[-0.92, -0.16, 0.34, -0.12],
	[0.08, -0.97, -0.25, 0.54],
	[0.00, 0.00, 0.00, 1.00]
]

home_pose_solutions = ikmodel.manip.FindIKSolutions(
	home_pose, openravepy.IkFilterOptions.CheckEnvCollisions
)

robot.SetDOFValues(
        home_pose_solutions[0], m.GetArmIndices()
)

raw_input('press [enter] to continue')
