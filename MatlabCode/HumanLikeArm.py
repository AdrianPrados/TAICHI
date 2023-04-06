''' Code created by Adrian Prados and Blanca Lopez, 
researchers from RoboticsLab, University Carlos III of Madrid, Spain'''

import numpy as np
from adam import Simulation, ConfigurationsManager
from adam.entities import Configuration, Data
import math
import pandas as pd
from UR3_Inverse_Kinematics import *
from scipy.spatial.transform import Rotation
from numpy import linalg as LA


#--------------------------FUNCTIONS ------------------------------------
def get_quaternion_from_euler(roll, pitch, yaw):
	''' Conversion quaternion to euler angles'''
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	return [qw, qz, qy, qx]


def CheckCollision(configs):
	''' Check collision using the ADAM simulator for our robot model based on MujoCo'''
	left_configuration = Configuration(configs[0], configs[1], configs[2], configs[3], configs[4], configs[5])
	#sim.render()
	data: Data = sim.step(left_configuration, initial_data.configuration.right_manipulator)
	col = data.collision.left_manipulator.self_collision
	return col

def DK(configuration):
	''' Estimation of the Direct Kinematics to obtain the values for each joint in cartesian coordinates'''
	SpacialConfigs = []
	a: list[float] = [0.0, 0.24365, 0.21325, 0.0, 0.0, 0.0]
	d: list[float] = [0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819]
	alpha: list[float] = [-np.pi/2, 0.0, 0.0, -np.pi/2, np.pi/2, 0.0]
	theta: np.ndarray = configuration
	system: np.ndarray = np.eye(4)
	for i in range(len(a)):
		step_matrix: np.ndarray = np.array([[np.cos(theta[i]), -np.cos(alpha[i])*np.sin(theta[i]), np.sin(alpha[i])*np.sin(theta[i]), a[i]*np.cos(theta[i])],
    		[np.sin(theta[i]), np.cos(alpha[i])*np.cos(theta[i]), -np.sin(alpha[i])*np.cos(theta[i]), a[i]*np.sin(theta[i])],
    		[0.0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
    		[0.0, 0.0, 0.0, 1.0]])
		system = system @ step_matrix
		SpacialConfigs.append(system)

	return SpacialConfigs

def distPosition(A,B):
	''' Estimate the position between two points'''
	Pos = abs(LA.norm(A-B))
	return Pos

def distOrientation(r,h):
	''' Estimate the orientation between to quaternions'''
	drao_p = np.arccos((r[0]*h[0])+(r[1]*h[1])+(r[2]*h[2])+(r[3]*h[3]))
	drao_n = np.arccos((r[0]*-h[0])+(r[1]*-h[1])+(r[2]*-h[2])+(r[3]*-h[3]))
	DRAO = min(drao_p,drao_n)

	return DRAO

def distanceMetric(ElbowHuman,W2,W1,W3,ElbowRobot,Shoulder):
	''' Estimate the distance respect the human elbow for all of the joints of the robotic arm'''
	ElbowHuman = np.transpose(ElbowHuman)
	C_W2 = abs(LA.norm(ElbowHuman-W2))
	C_W1 = abs(LA.norm(ElbowHuman-W1))
	C_W3 = abs(LA.norm(ElbowHuman-W3))
	C_CR = abs(LA.norm(ElbowHuman-ElbowRobot))
	C_HR = abs(LA.norm(ElbowHuman-Shoulder))

	D = (10*C_W1)+(300*C_CR)+C_HR # Cost function for our model, that function can be adjust for different models
	return D

def variationWrist(W_new,W_old):
	''' Estimate the wirst variation for the robotics arm'''
	WError = 200*abs(W_new - W_old)
	return WError

def VisualizedSimulation(FinalConfigs):
	''' Function to visualize the results'''
	configuration_list: list[Configuration] = FinalConfigs
	for configuration in configuration_list:
		sim.render()
		data: Data = sim.step(configuration, initial_data.configuration.right_manipulator)
		time.sleep(0.1)
		Prueba = sim.controller.data.xpos
		print(Prueba)

	sim.close()

#------------------------------------------------------------------


'''Inicialization of the model simulated''' 
sim: Simulation = Simulation()
initial_data: Data = sim.load_scene()
left_configuration: Configuration = initial_data.configuration.left_manipulator

# Prueba de valores en el simulador
Prueba = sim.controller.data.xpos
print(Prueba)

''' Rotation matrizes for the model and limit values'''
T = np.array([
	[1,0,0,0],
	[0,0.7071,0.7071,0],
	[0,-0.7071,0.7071,0],
	[0,0,0,1]
	], np.float64)

Roz = np.array([
	[math.cos(math.radians(180)), -math.sin(math.radians(180)), 0],
	[math.sin(math.radians(180)), math.cos(math.radians(180)), 0],
	[0, 0, 1]
	], np.float64)

ShoulderLim = [-2,1.5]

''' Read the values from the csv'''
pathArm = pd.read_csv('/home/nox/BrazoCamara/ur_ikfast/DatosHumano/datosprueba11/DatosBrazoHumano.csv',header=None) # Change the path with your own
pathElbow = pd.read_csv('/home/nox/BrazoCamara/ur_ikfast/DatosHumano/datosprueba11/CodoHumano.csv',header=None)
pathEuler = pd.read_csv('/home/nox/BrazoCamara/ur_ikfast/DatosHumano/datosprueba11/EfectorFinal.csv',header=None)
#print(pathArm)

'''Variables of control'''
iterationArm = len(pathArm)
iterationElbow = len(pathElbow)
elbowValues = pathElbow.to_numpy()
armValues = pathArm.to_numpy()
eulerValues = pathEuler.to_numpy()
Elbow = []
FinalConfigs = []
k = 0

'''Weights for the function cost'''
W_rax = 1
W_rao = 10
W_A = 50

''' Elbow values in order'''
for j in range(0,iterationElbow,3):
	ElbowLocal = [elbowValues[j],elbowValues[j+1],elbowValues[j+2]]
	Elbow.append(np.array(ElbowLocal))

''' Arm values'''

for i in range(0,iterationArm,4):
	#print(k)
	
	DistDif = 10000000 # Initial value very high to always used the first value selected
	DisDigOG = DistDif
	iterationControl = 0
	check = True

	''' Matrix end efector'''
	MatrixArm = np.array([
		[armValues[i,0],armValues[i,1],armValues[i,2],armValues[i,3]],
		[armValues[i+1,0],armValues[i+1,1],armValues[i+1,2],armValues[i+1,3]],
		[armValues[i+2,0],armValues[i+2,1],armValues[i+2,2],armValues[i+2,3]],
		[armValues[i+3,0],armValues[i+3,1],armValues[i+3,2],armValues[i+3,3]]
		], np.float64)
	#print(MatrixArm[0:2,0:2])

	MatrixArm[0:3,0:3] =  np.matmul(Roz,MatrixArm[0:3,0:3])
	MatrixArm[0:2,3] = -1*MatrixArm[0:2,3];
	MatrixArm = np.matmul(T,MatrixArm)

	'''Elbow vector'''
	ElbowVector = Elbow[k]
	

	''' Goal point and orientation''' 
	X_RAGoal = np.array([armValues[i,3],armValues[i+1,3],armValues[i+2,3]])
	RotMat = np.array([
		[armValues[i,0],armValues[i,1],armValues[i,2]],
		[armValues[i+1,0],armValues[i+1,1],armValues[i+1,2]],
		[armValues[i+2,0],armValues[i+2,1],armValues[i+2,2]]
		], np.float64)
	quaternion = get_quaternion_from_euler(eulerValues[k,5],eulerValues[k,4],eulerValues[k,3])
	k = k+1;

	try:
		''' Generation of the analytical Inverse Kinematics for the UR3 model'''
		IKSolutions = Cinematica_Inversa(MatrixArm,'ur3') # Send matrix and model in string
		
		for n in range(8):
			collided = CheckCollision(IKSolutions[n]) # Check if any self collision exist
			if collided == False:
				ConfigsJoints = DK(IKSolutions[n]) # Obtain Direct kinematics

				''' End efector data'''
				EndEfectorOrient = ConfigsJoints[5][0:3,0:3]
				EndEfectorOrient[0:3,0:2] = -1*EndEfectorOrient[0:3,0:2] 
				EndEfectorPose = ConfigsJoints[5][0:3,3]  # There is an error in position
				r = Rotation.from_matrix(EndEfectorOrient.tolist())
				EulerEndEfector = r.as_euler("xyz",degrees=False) 
				QuatEndEfector = get_quaternion_from_euler(EulerEndEfector[2],EulerEndEfector[1],EulerEndEfector[0])
				'''Wrist 3 position'''
				Wrist3Pose = ConfigsJoints[4][0:3,3] 

				''' Wrist 2 position'''
				Wrist2Pose = ConfigsJoints[3][0:3,3]  

				''' Wrist1 position'''
				Wrist1Pose = ConfigsJoints[2][0:3,3] 

				''' Elbow position'''
				ElbowPose = ConfigsJoints[1][0:3,3] 

				''' Shoulder position'''
				ShoulderPose = ConfigsJoints[0][0:3,3]

				'''Calculate the cost function'''
				d_RAx = distPosition(EndEfectorPose,X_RAGoal) #position end efector
				d_RAo = distOrientation(QuatEndEfector,quaternion) #orientation end efector
				d_Cost = distanceMetric(ElbowVector,Wrist2Pose,Wrist1Pose,Wrist3Pose,ElbowPose,ShoulderPose)

				if i==0:
					ErrorWrist = 0
				else:
					ErrorWrist = variationWrist(IKSolutions[n][3],WristOld)

				FinalDistance = np.real(((W_rax*d_RAx)+(W_rao*d_RAo)+(W_A*d_Cost)+(ErrorWrist)))

				#print(FinalDistance)

				'''Check if the execution is finished'''

				if FinalDistance < DistDif and IKSolutions[n][1]>= ShoulderLim[0] and IKSolutions[n][1]<= ShoulderLim[1]:
					DistDif = FinalDistance
					BestConfig = IKSolutions[n]
					best_n = n					

		''' Save the best configuration of the analytical values of the IK'''		
		if DistDif != DisDigOG:
			WristOld = BestConfig[3]
			FinalConfigs.append(BestConfig)
			#print(best_n)
		else:
			print("Not valid configuration found")

	except ValueError:
		print("Error, wrong IK value")


'''Visulization of the final result'''
VisualizedSimulation(FinalConfigs)
