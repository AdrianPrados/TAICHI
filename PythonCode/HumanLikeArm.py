''' Code created by Adrian Prados and Blanca Lopez, 
researchers from RoboticsLab, University Carlos III of Madrid, Spain'''

import numpy as np
from adam import Simulation, ConfigurationsManager
from adam.entities import Configuration, AdamInfo
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
	data: AdamInfo = sim.step(left_configuration, initial_data.right_manipulator.configuration)
	col = data.left_manipulator.collision.self_collision
	return col

def DK2(configuration):
	'''Estimation of the Direct Kinematics to obtain the values for each joint in cartesian coordinates'''
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


def DK(configuration):
	''' Obtain the values for each joint in cartesian coordinates right from simulation model'''
	configuration_list: list[Configuration] = configuration
	for configuration in configuration_list:
		data: AdamInfo = sim.step(configuration, initial_data.right_manipulator.configuration)
		# ShoulderXpos = sim.controller.data.xpos[2]
		ShoulderXpos = data.left_manipulator.systems[1].position # using the model value 
		#ElbowXpos = sim.controller.data.xpos[5] # using the ,model directly from Mujoco 
		ElbowXpos = data.left_manipulator.systems[3].position
		#W1Xpos = sim.controller.data.xpos[6]
		W1Xpos = data.left_manipulator.systems[4].position
		#W2Xpos = sim.controller.data.xpos[7]
		W2Xpos = data.left_manipulator.systems[5].position
		#W3Xpos = sim.controller.data.xpos[8]
		W3Xpos = data.left_manipulator.systems[6].position
		Xposes = np.array([ShoulderXpos,ElbowXpos,W1Xpos,W2Xpos,W3Xpos])

	return Xposes


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
	W1 = np.transpose(W1)
	W2 = np.transpose(W2)
	W3 = np.transpose(W3)
	Shoulder = np.transpose(Shoulder)
	ElbowRobot = np.transpose(ElbowRobot)

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
		data: AdamInfo = sim.step(configuration, initial_data.right_manipulator.configuration)
		time.sleep(0.1)

	sim.close()

def checkLimits(End,XRa):
	'''Functions that check if the value generated is in the limit of the robotic arm'''
	End = np.array([*End,1])
	XRa = [XRa[0]+0.07,XRa[1]+0.13,XRa[2]+1.15]
	Pr = T2Correct @ End
	Pr = [Pr[0],Pr[1],Pr[2]]
	dif = LA.norm(np.array(Pr) - np.array(XRa))
	if dif > 0.05:
		return False
	else:
		return True


#------------------------------------------------------------------

'''Inicialization of the model simulated''' 
sim: Simulation = Simulation()
initial_data: AdamInfo = sim.load_scene()
left_configuration: Configuration = initial_data.left_manipulator.configuration
# sim.data_manager.collision_detector.ignore_list.append(("upperarm_left","body"))
# sim.data_manager.collision_detector.ignore_list.append(("forearm_left","body"))


''' Rotation matrizes for the model and limit values'''
T = np.array([
	[1,0,0,0],
	[0,0.7071,0.7071,0],
	[0,-0.7071,0.7071,0],
	[0,0,0,1]
	], np.float64)

T2 = np.array([
	[1,0,0,0.07],
	[0,0.7071,0.7071,0.13],
	[0,-0.7071,0.7071,1.15],
	[0,0,0,1]
	], np.float64)
T2Correct = T2
T2 = LA.inv(T2)


Roz = np.array([
	[math.cos(math.radians(180)), -math.sin(math.radians(180)), 0],
	[math.sin(math.radians(180)), math.cos(math.radians(180)), 0],
	[0, 0, 1]
	], np.float64)

ShoulderLim = [-2,1.5]

''' Read the values from the csv'''

pathArm = pd.read_csv('/home/nox/BrazoCamara/ur_ikfast/DatosHumano/Prueba2/DatosBrazoHumano.csv',header=None) # Change the path with your own
pathElbow = pd.read_csv('/home/nox/BrazoCamara/ur_ikfast/DatosHumano/Prueba2/CodoHumano.csv',header=None)
pathEuler = pd.read_csv('/home/nox/BrazoCamara/ur_ikfast/DatosHumano/Prueba2/EfectorFinal.csv',header=None)


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
				DK_Config = np.array([IKSolutions[n]])
				ConfigsJoints = DK(DK_Config) # Obtain Direct kinematics	
				ConfigsJoints2 = DK2(IKSolutions[n]) # Obtain Direct kinematics	

				''' End efector data'''
				EndEfectorOrient = ConfigsJoints2[5][0:3,0:3]
				EndEfectorOrient[0:3,0:2] = -1*EndEfectorOrient[0:3,0:2] 
				EndEfectorPose = ConfigsJoints2[5][0:3,3]
				EndEfector = EndEfectorPose
				r = Rotation.from_matrix(EndEfectorOrient.tolist())
				EulerEndEfector = r.as_euler("xyz",degrees=False) 
				QuatEndEfector = get_quaternion_from_euler(EulerEndEfector[2],EulerEndEfector[1],EulerEndEfector[0])

				'''Detect if the point is in the limits'''
				correct = checkLimits(EndEfector,X_RAGoal)


				'''Wrist 3 position'''
				Wrist3Pose = ConfigsJoints[4,0:3] #ConfigsJoints[4][0:3,3] 
				Wrist3Pose = np.array([*Wrist3Pose,1])
				Wrist3Pose = T2 @ Wrist3Pose 
				Wrist3Pose = Wrist3Pose[0:3]
							
				''' Wrist 2 position'''
				Wrist2Pose = ConfigsJoints[3,0:3] #ConfigsJoints[3][0:3,3]
				Wrist2Pose = np.array([*Wrist2Pose,1])
				Wrist2Pose = T2 @ Wrist2Pose  
				Wrist2Pose = Wrist2Pose[0:3]

				''' Wrist1 position'''
				Wrist1Pose = ConfigsJoints[2,0:3] #ConfigsJoints[2][0:3,3]
				Wrist1Pose = np.array([*Wrist1Pose,1])
				Wrist1Pose = T2 @ Wrist1Pose 
				Wrist1Pose = Wrist1Pose[0:3]

				''' Elbow position'''
				ElbowPose = ConfigsJoints[1,0:3] #ConfigsJoints[1][0:3,3] 
				ElbowPose = np.array([*ElbowPose,1])
				ElbowPose = T2 @ ElbowPose 
				ElbowPose = ElbowPose[0:3]

				''' Shoulder position'''
				'''ShoulderPose = ConfigsJoints[0,0:3] #ConfigsJoints[0][0:3,3]
				ShoulderPose = np.array([*ShoulderPose,1])
				ShoulderPose = T2 @ ShoulderPose 
				ShoulderPose = ShoulderPose[0:3]''' 
				ShoulderPose = ConfigsJoints2[0][0:3,3]


				'''Calculate the cost function'''
				d_RAx = distPosition(EndEfectorPose,X_RAGoal) #position end efector
				d_RAo = distOrientation(QuatEndEfector,quaternion) #orientation end efector
				d_Cost = distanceMetric(ElbowVector,Wrist2Pose,Wrist1Pose,Wrist3Pose,ElbowPose,ShoulderPose)

				try:
					WristOld
				except:
				    WristOld = None

				if i==0 or WristOld == None:
					ErrorWrist = 0
				else:
					ErrorWrist = variationWrist(IKSolutions[n][3],WristOld)

				FinalDistance = np.real(((W_rax*d_RAx)+(W_rao*d_RAo)+(W_A*d_Cost)+(ErrorWrist)))



				'''Check if the execution is finished'''

				if correct == True and FinalDistance < DistDif and IKSolutions[n][1]>= ShoulderLim[0] and IKSolutions[n][1]<= ShoulderLim[1]:
					DistDif = FinalDistance
					BestConfig = IKSolutions[n]
					best_n = n					

		''' Save the best configuration of the analytical values of the IK'''		
		if DistDif != DisDigOG:
			WristOld = BestConfig[3]
			FinalConfigs.append(BestConfig)
			#print(DistDif)

			#print(BestConfig)
		else:
			print("Not valid configuration found")

	except ValueError:
		print("Error, wrong IK value")


'''Visulization of the final result'''
VisualizedSimulation(FinalConfigs)
