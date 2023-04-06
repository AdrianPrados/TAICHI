''' Code created by Adrian Prados and Blanca Lopez, 
researchers from RoboticsLab, University Carlos III of Madrid, Spain'''


#!/home/nox/anaconda3/envs/ikfastenv/bin/ python3
import sys
import pyrealsense2 as rs
import mediapipe as mp
import cv2
import numpy as np
import datetime as dt
import time
import math as mt
import math as mt
from scipy.spatial.transform import Rotation
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter


################### Functions ##################################


def rotateY(origin, point, angle):
    ''' Function defined to rotate one point respect to naother point'''
    #Angle in radians
    ox,oz = origin
    px,pz = point

    qx = ox + mt.cos(angle) * (px - ox) - mt.sin(angle) * (pz - oz)
    qz = oz + mt.sin(angle) * (px - ox) + mt.cos(angle) * (pz - oz)
    return qx, qz

def calculate_angle(a,b,c):
    ''' Function defined to calculate angle between three points'''
    a = np.array(a) # First
    b = np.array(b) # Mid
    c = np.array(c) # End
    
    radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
    angle = np.abs(radians*180.0/np.pi)
    
    if angle >180.0:
        angle = 360-angle
        
    return angle 

def HandPlaneOrientation(points):
    ''' Obtain the zvector of the final efector as the ortogonal vector of the hand plane'''
    normal_vector = np.cross(points[0] - points[2], points[0] - points[1])
    normal_vector /= np.linalg.norm(normal_vector)
    x_vec = (points[2]-points[1])
    x_vec /= np.linalg.norm(x_vec)
    y_vec = np.cross(normal_vector,x_vec)
    y_vec /= np.linalg.norm(y_vec)

    ''' The -1 correct the orientation of the hand plane respect the image orientation'''
    Mat = np.matrix([
        [-1*x_vec[0],x_vec[1],x_vec[2]],
        [-1*y_vec[0],y_vec[1],y_vec[2]],
        [normal_vector[0],-1*normal_vector[1],-1*normal_vector[2]]
         ])

    angle = 90
    Rox = np.matrix([
        [1, 0, 0],
        [0, mt.cos(mt.radians(angle)), -mt.sin(mt.radians(angle))],
        [0, mt.sin(mt.radians(angle)), mt.cos(mt.radians(angle))]
        ])

    Roz = np.matrix([
        [mt.cos(-mt.radians(angle)), -mt.sin(-mt.radians(angle)), 0],
        [mt.sin(-mt.radians(angle)), mt.cos(-mt.radians(angle)), 0],
        [0, 0, 1]
        ])


    Rotacional = np.matmul(Rox,Roz)
    Rotacional = np.linalg.inv(Rotacional)
    Mat = np.transpose(Mat)
    MatRot = np.matmul(Rotacional,Mat)

    return MatRot


def project_point_on_line(A, B, distance):
    ''' Function that project the elbow in line to rectified the oclusions'''
    # Calculate the direction vector between A and B
    direction = np.array(B) - np.array(A)
    # Normalize the direction vector
    direction = direction / np.linalg.norm(direction)
    # Multiple the vector for the distance
    projection = A + direction * distance
    return projection

def project_end_efector(A,B,distance):
    ''' Function that project the elbow in line to rectified the oclusions'''
    # Calculate the direction vector between A and B
    direction = np.array(B) - np.array(A)
    # Normalize the direction vector
    direction = direction / np.linalg.norm(direction)
    # Multiple the vector for the distance
    projectionEndEfector = A + direction * distance
    return projectionEndEfector

def smooth_rotations(rotations, sigma=1):
    r11 = []
    r12 = []
    r13 = []
    r21 = []
    r22 = []
    r23 = []
    r31 = []
    r32 = []
    r33 = []
    for r in rotations:
        r11.append(r[0,0])
        r12.append(r[0,1])
        r13.append(r[0,2])
        r21.append(r[1,0])
        r22.append(r[1,1])
        r23.append(r[1,2])
        r31.append(r[2,0])
        r32.append(r[2,1])
        r33.append(r[2,2])
    R1 = gaussian_filter(r11, sigma)
    R2 = gaussian_filter(r12, sigma)
    R3 = gaussian_filter(r13, sigma)
    R4 = gaussian_filter(r21, sigma)
    R5 = gaussian_filter(r22, sigma)
    R6 = gaussian_filter(r23, sigma)
    R7 = gaussian_filter(r31, sigma)
    R8 = gaussian_filter(r32, sigma)
    R9 = gaussian_filter(r33, sigma)

    return R1,R2,R3,R4,R5,R6,R7,R8,R9

def smooth_endefector(rotations, sigma=1):
    X = []
    Y= []
    Z = []
    for r in rotations:
        X.append(r[0,3])
        Y.append(r[1,3])
        Z.append(r[2,3])
    XEnd = gaussian_filter(X, sigma)
    YEnd = gaussian_filter(Y, sigma)
    ZEnd = gaussian_filter(Z, sigma)

    return XEnd,YEnd,ZEnd

def smooth_elbow(elbow, sigma=1):
    X = []
    Y= []
    Z = []
    for r in elbow:
        X.append(r[0,0])
        Y.append(r[1,0])
        Z.append(r[2,0])
    XElbow = gaussian_filter(X, sigma)
    YElbow = gaussian_filter(Y, sigma)
    ZElbow = gaussian_filter(Z, sigma)

    return XElbow,YElbow,ZElbow

def plot_smoothed_rotations(rotations, R1,R2,R3,R4,R5,R6,R7,R8,R9):
    rotations = np.array(rotations)  # Convertimos a arreglo NumPy
    fig, axs = plt.subplots(3, 3, figsize=(10,10))
    fig.subplots_adjust(hspace=0.4, wspace=0.4)
    axs = axs.ravel()

    for i in range(9):
        if i < 3:
            axs[i].set_title("Row {}".format(i))
        if i % 3 == 0:
            axs[i].set_ylabel("Axis {}".format(i//3))

        axs[i].plot(rotations[:, i//3, i%3], label='raw')
        axs[i].legend()
    axs[0].plot(R1, label='filter')
    axs[0].legend()
    axs[1].plot(R2, label='filter')
    axs[1].legend()
    axs[2].plot(R3, label='filter')
    axs[2].legend()
    axs[3].plot(R4, label='filter')
    axs[3].legend()
    axs[4].plot(R5, label='filter')
    axs[4].legend()
    axs[5].plot(R6, label='filter')
    axs[5].legend()
    axs[6].plot(R7, label='filter')
    axs[6].legend()
    axs[7].plot(R8, label='filter')
    axs[7].legend()
    axs[8].plot(R9, label='filter')
    axs[8].legend()
    plt.show()

def plot_smoothed_EndEfector(rotations, X,Y,Z):
    rotations = np.array(rotations)  # Convertimos a arreglo NumPy
    fig, axs = plt.subplots(1, 3, figsize=(10,10))
    fig.subplots_adjust(hspace=0.4, wspace=0.4)
    axs = axs.ravel()
    Xr = []
    Yr = []
    Zr = []

    for r in rotations:
        Xr.append(r[0,3])
        Yr.append(r[1,3])
        Zr.append(r[2,3])

    for i in range(3):
        if i < 3:
            axs[i].set_title("Row {}".format(i))

    axs[0].plot(Xr,label='raw')
    axs[0].plot(X, label='filter')
    axs[0].legend()
    axs[1].plot(Yr,label='raw')
    axs[1].plot(Y, label='filter')
    axs[1].legend()
    axs[2].plot(Zr,label='raw')
    axs[2].plot(Z, label='filter')
    axs[2].legend()
    plt.show()

def plot_smoothed_Elbow(elbow, X,Y,Z):
    elbow = np.array(elbow)  # Convertimos a arreglo NumPy
    fig, axs = plt.subplots(1, 3, figsize=(10,10))
    fig.subplots_adjust(hspace=0.4, wspace=0.4)
    axs = axs.ravel()
    Xr = []
    Yr= []
    Zr = []
    for r in elbow:
        Xr.append(r[0,0])
        Yr.append(r[1,0])
        Zr.append(r[2,0])

    for i in range(3):
        if i < 3:
            axs[i].set_title("Row {}".format(i))

    axs[0].plot(Xr,label='raw')
    axs[0].plot(X, label='filter')
    axs[0].legend()
    axs[1].plot(Yr,label='raw')
    axs[1].plot(Y, label='filter')
    axs[1].legend()
    axs[2].plot(Zr,label='raw')
    axs[2].plot(Z, label='filter')
    axs[2].legend()
    plt.show()


################################################################

# ======= VISUAL FONTS =======

font = cv2.FONT_HERSHEY_SIMPLEX
org = (20, 100)
fontScale = .5
color = (0,0,0)
thickness = 2


# ====== DATA ======
global DATOS, CORCODO, EFECTOR, DATOSPRE, CORCODOPRE
DATOS = []
DATOSPRE = []
CORCODO = []
CORCODOPRE = []
EFECTOR = []
datos = 0
h = 0
# ====== Realsense ======
''' Detect the camera RealSense D435i and activate it'''
realsense_ctx = rs.context()
connected_devices = [] # List of serial numbers for present cameras
for i in range(len(realsense_ctx.devices)):
    detected_camera = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
    print("Detected_camera")
    connected_devices.append(detected_camera)
device = connected_devices[0] # For this code only one camera is neccesary
pipeline = rs.pipeline()
config = rs.config()
background_removed_color = 153 # Grey color for the background

# ====== Mediapipe ======
''' Characteristics for hands and pose tracking'''
#------- Hands ---------
''' Link for Hands: https://google.github.io/mediapipe/solutions/hands.html'''
mpHands = mp.solutions.hands
hands = mpHands.Hands(min_detection_confidence=0.3) # The confidence can be change for the specific project
desired_solution = [0, 0, 0, 0, 0, 0] 
#------- Body --------
''' Link for BodyPose : https://google.github.io/mediapipe/solutions/pose.html'''
mpPose = mp.solutions.pose
pose = mpPose.Pose(min_detection_confidence=0.1)

mpDraw = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles


# ====== Enable Streams ======
''' Activate the stream caracteristics for the RealSense D435i'''
config.enable_device(device)

#For worse FPS, but better resolution:

#stream_res_x = 1280
#stream_res_y = 720

#For better FPS. but worse resolution:

stream_res_x = 640
stream_res_y = 480

stream_fps = 30

config.enable_stream(rs.stream.depth, stream_res_x, stream_res_y, rs.format.z16, stream_fps)
config.enable_stream(rs.stream.color, stream_res_x, stream_res_y, rs.format.bgr8, stream_fps)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# ====== Get depth Scale ======
''' Obtain the scale of the depth estimated by the depth sensors of the camara''' 
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale for Camera SN",device,"is: ",depth_scale)

# ====== Set clipping distance ======
''' Generate the maximun distance for the camera range to detect'''
clipping_distance_in_meters = 5
clipping_distance = clipping_distance_in_meters / depth_scale
print("Configuration Successful for SN", device)

# ====== Get and process images ====== 
print("Starting to capture images on SN:",device)


# ======= Algorithm =========
while True:
    start_time = dt.datetime.today().timestamp()

    '''Get and align frames from the camera to the point cloud'''
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    if not aligned_depth_frame or not color_frame:
        print("\tNot aligned")
        continue

    ''' Process images to work with one color image'''
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    depth_image_flipped = cv2.flip(depth_image,1)
    color_image = np.asanyarray(color_frame.get_data())

    depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #Depth image is 1 channel, while color image is 3
    background_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), background_removed_color, color_image)

    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    images = cv2.flip(background_removed,1)
    color_image = cv2.flip(color_image,1)
    color_images_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

    ''' Process hands and pose estimations'''
    results = hands.process(color_images_rgb)
    cuerpo = pose.process(color_images_rgb)

    ''' Load the intrinsics values of the camera RealSense D435i'''
    INTR = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    
    ''' If body is detected check the hands'''
    if cuerpo.pose_landmarks and results.multi_hand_landmarks:
        ''' Draw body lines and save body references'''
        mpDraw.draw_landmarks(images, cuerpo.pose_landmarks, mpPose.POSE_CONNECTIONS)
        codo = cuerpo.pose_landmarks.landmark[mpPose.PoseLandmark.RIGHT_ELBOW]
        muneca = cuerpo.pose_landmarks.landmark[mpPose.PoseLandmark.RIGHT_WRIST]
        hombro = cuerpo.pose_landmarks.landmark[mpPose.PoseLandmark.RIGHT_SHOULDER]
        hombro2 = cuerpo.pose_landmarks.landmark[mpPose.PoseLandmark.LEFT_SHOULDER]

        ''' calculate the angle of the elbow (just to check)'''
        angulo = calculate_angle([hombro2.x,hombro2.y], [codo.x,codo.y], [muneca.x,muneca.y])

        #--------- Human Elbow --------
        codoX = int(codo.x*len(depth_image_flipped[0]))
        codoY = int(codo.y*len(depth_image_flipped))
        if codoX >= len(depth_image_flipped[0]):
            codoX = len(depth_image_flipped[0]) - 1
        if codoY>= len(depth_image_flipped):
            codoY = len(depth_image_flipped) - 1

        # ----------- Human Wrist --------
        munecaX = int(muneca.x*len(depth_image_flipped[0]))
        munecaY = int(muneca.y*len(depth_image_flipped))
        if munecaX >= len(depth_image_flipped[0]):
            munecaX = len(depth_image_flipped[0]) - 1
        if munecaY >= len(depth_image_flipped):
            munecaY = len(depth_image_flipped) - 1

        # ----------- Human Shoulder (Left) ---------    
        hombroX = int(hombro.x*len(depth_image_flipped[0]))
        hombroY = int(hombro.y*len(depth_image_flipped))
        if hombroX  >= len(depth_image_flipped[0]):
            hombroX  = len(depth_image_flipped[0]) - 1
        if hombroY >= len(depth_image_flipped):
            hombroY = len(depth_image_flipped) - 1

        # ----------- Human Shoulder (Rigth) ---------
        ''' The rigth shoulder is used as pivot to the rotation'''
        hombro2X = int(hombro2.x*len(depth_image_flipped[0]))
        hombro2Y = int(hombro2.y*len(depth_image_flipped))
        if hombro2X  >= len(depth_image_flipped[0]):
            hombro2X  = len(depth_image_flipped[0]) - 1
        if hombro2Y >= len(depth_image_flipped):
            hombro2Y = len(depth_image_flipped) - 1

        ''' Z values for the elbow, wrist and shoulder'''
        ZCODO = depth_image_flipped[codoY,codoX] * depth_scale # meters
        ZMUNECA = depth_image_flipped[munecaY,munecaX] * depth_scale # meters
        ZHOMBRO = depth_image_flipped[hombroY,hombroX] * depth_scale # meters
        ZHOMBRO2 = depth_image_flipped[hombro2Y,hombro2X] * depth_scale # meters

        '''Values of the different studied points in meters'''
        CodoBueno = rs.rs2_deproject_pixel_to_point(INTR,[codoX,codoY],ZCODO)
        MunecaBueno = rs.rs2_deproject_pixel_to_point(INTR,[munecaX,munecaY],ZMUNECA)
        HombroBueno = rs.rs2_deproject_pixel_to_point(INTR,[hombroX,hombroY],ZHOMBRO)
        Hombro2Bueno = rs.rs2_deproject_pixel_to_point(INTR,[hombro2X,hombro2Y],ZHOMBRO2)

        #print(f"Codo: {CodoBueno}")
        #print(f"Muneca: {MunecaBueno}")
        #print(f"HombroIzquierda : {HombroBueno}")
        #print(f"HombroDerecha : {Hombro2Bueno}")

        
        ''' Calculate the rotation of the left shoulder to orientate correctly to the camera the body'''
        theta = mt.atan2((HombroBueno[2]-Hombro2Bueno[2]),(HombroBueno[0]-Hombro2Bueno[0]))
        theta = 180 - mt.degrees(theta)# Angle to rotate the left arm
        # As we rotate using the Y axis, if rigth shoulder is the nearest to te camera, the angle is negative
        if theta > 180:
            theta = -1*(360 - theta)
        else:
            theta = theta
        #images = cv2.putText(images, f"THETA: {theta}", org5, font, fontScale, color, thickness, cv2.LINE_AA)

        '''Generates the rotation for all the point od th left arm'''
        Pivote = (Hombro2Bueno[0],Hombro2Bueno[2])
        RotHombro = (HombroBueno[0],HombroBueno[2])
        RotCodo = (CodoBueno[0],CodoBueno[2])
        RotMuneca= (MunecaBueno[0],MunecaBueno[2])
        rotadoH = rotateY(Pivote,RotHombro,mt.radians(theta))
        rotadoC = rotateY(Pivote,RotCodo,mt.radians(theta))
        rotadoM = rotateY(Pivote,RotMuneca,mt.radians(theta))

        #print(f"MunecaBuena2 : ", {rotadoH[1],Hombro2Bueno[2]})

        HombroFinal= [rotadoH[0],HombroBueno[1],rotadoH[1]]
        CodoFinal= [rotadoC[0],CodoBueno[1],rotadoC[1]]
        MunecaFinal= [rotadoM[0],MunecaBueno[1],rotadoM[1]]
        #print("MunecaBuena : ",MunecaFinal)
        #print("HombroFinal : ",HombroFinal)
        #print("CodoFinal : ",CodoFinal)

        ''' Reduction factor for the human arm'''
        HumanHumero = abs(mt.dist(HombroFinal,CodoFinal))
        print(HumanHumero)
        if HumanHumero < 0.26 or HumanHumero > 0.28:
            CodoFinal = project_point_on_line(HombroFinal,CodoFinal,0.27)


        HumanCubito = abs(mt.dist(CodoFinal,MunecaFinal))

        if HumanCubito < 0.23 or HumanCubito > 0.25:
            MunecaFinal = project_end_efector(CodoFinal,MunecaFinal,0.24)


        
        BrazoHuman = HumanHumero + HumanCubito
        if BrazoHuman <=0.8:
            try:
                ''' obtain factor Robot-human for arm length'''
                factorRH = (0.5/BrazoHuman)
            except:
                factorRH = 1

            '''Calculate the translation between points of the human to the robot base reference
            Human to robot reference -> yhuman = zbase; xhuman=ybase; zhuman = xbase;'''
            robotHombro = [0,0,0] #xbase,ybase,zbase can be different in fucntion of the arm used
            Translation = [(robotHombro[0]+HombroFinal[2]),(robotHombro[1]+HombroFinal[0]),(robotHombro[2]+HombroFinal[1])]



            robotHombro = [(Translation[0] - HombroFinal[2]),(Translation[1] - HombroFinal[0]),(Translation[2]- HombroFinal[1])]
            robotCodo = [(Translation[0] - CodoFinal[2]  )*factorRH,(Translation[1] - CodoFinal[0])*factorRH,(Translation[2] - CodoFinal[1])*factorRH]
            robotMuneca = [(Translation[0] - MunecaFinal[2])*factorRH,(Translation[1] - MunecaFinal[0])*factorRH,(Translation[2] - MunecaFinal[1])*factorRH]
            
            #print(Translation)
            #print(f"Hombro en el robot : {robotHombro}")
            #print(f"Codo en el robot : {robotCodo}")
            #print("Muneca en el robot : ",robotMuneca)

            ''' Detection of left hand orientation'''
            if results.multi_handedness[0].classification[0].label == 'Left':    # Only applied when the left hand is in camera, if both hands are showed dont work

                for handLms in results.multi_hand_landmarks:
                    mpDraw.draw_landmarks(images, handLms, mpHands.HAND_CONNECTIONS)
                    Cero = results.multi_hand_landmarks[0].landmark[mpHands.HandLandmark.WRIST]
                    Cinco = results.multi_hand_landmarks[0].landmark[mpHands.HandLandmark.INDEX_FINGER_MCP]
                    Diecisiete = results.multi_hand_landmarks[0].landmark[mpHands.HandLandmark.PINKY_MCP]
                    
                    '''Depth hand values for the 0,5,17 hand reference plane'''
                    CeroX = int(Cero.x*len(depth_image_flipped[0]))
                    CeroY = int(Cero.y*len(depth_image_flipped))
                    if CeroX >= len(depth_image_flipped[0]):
                        CeroX = len(depth_image_flipped[0]) - 1

                    if CeroY>= len(depth_image_flipped):
                        CeroY = len(depth_image_flipped) - 1


                    CincoX = int(Cinco.x*len(depth_image_flipped[0]))
                    CincoY = int(Cinco.y*len(depth_image_flipped))
                    if CincoX >= len(depth_image_flipped[0]):
                        CincoX = len(depth_image_flipped[0]) - 1

                    if CincoY>= len(depth_image_flipped):
                        CincoY = len(depth_image_flipped) - 1

                    DiecisieteX = int(Diecisiete.x*len(depth_image_flipped[0]))
                    DiecisieteY = int(Diecisiete.y*len(depth_image_flipped))
                    if DiecisieteX >= len(depth_image_flipped[0]):
                        DiecisieteX = len(depth_image_flipped[0]) - 1

                    if DiecisieteY>= len(depth_image_flipped):
                        DiecisieteY = len(depth_image_flipped) - 1

                    ''' Z values for the hand (depth)'''
                    ZCERO = depth_image_flipped[CeroY,CeroX] * depth_scale # meters
                    ZCINCO = depth_image_flipped[CincoY,CincoX] * depth_scale # meters
                    ZDIECISITE = depth_image_flipped[DiecisieteY,DiecisieteX] * depth_scale # meters

                    '''Values of the different studied points in meters'''
                    CeroArray = rs.rs2_deproject_pixel_to_point(INTR,[CeroX,CeroY],ZCERO)
                    CincoArray = rs.rs2_deproject_pixel_to_point(INTR,[CincoX,CincoY],ZCINCO)
                    DiecisieteArray = rs.rs2_deproject_pixel_to_point(INTR,[DiecisieteX,DiecisieteY],ZDIECISITE)

                
                points = np.asarray([CeroArray, CincoArray, DiecisieteArray])
                MatRot = HandPlaneOrientation(points)
                #print(MatRot)
             
            ''' Generate the values for the UR3 robot'''
            #m=np.array([[1,0,0,-0.07],[0,0.7072,-0.7072,0.7214],[0,0.7072,0.7072,-0.9052],[0,0,0,1]])
            punto = [robotMuneca[0],robotMuneca[1],robotMuneca[2],1]
            puntoCodo = np.array([[robotCodo[0]],[robotCodo[1]],[robotCodo[2]]])
            #print(punto)
            try:
                MatrizBrazo = np.array([
                    [round(MatRot[0,0],2),round(MatRot[0,1],2),round(MatRot[0,2],2),punto[0]],
                    [round(MatRot[1,0],2),round(MatRot[1,1],2),round(MatRot[1,2],2),punto[1]],
                    [round(MatRot[2,0],2),round(MatRot[2,1],2),round(MatRot[2,2],2),punto[2]],
                    [0,0,0,1]
                    ], np.float64)
                #print(MatrizBrazo)
                if datos > 10:
                    if MatRot[0,0]== "nan" or MatRot[0,1]== "nan" or MatRot[0,2]== "nan":
                        continue
                    else:
                        ''' Correct data is saved'''
                        if h == 1:
                            h = 0
                            DATOSPRE.append(MatrizBrazo)
                            CORCODOPRE.append(puntoCodo)
                            print("Valor de codo",puntoCodo[0,0])
                            r = Rotation.from_matrix(MatRot)
                            angles = r.as_euler("xyz",degrees=False)
                            efectorFinal = [MatrizBrazo[0,3],MatrizBrazo[1,3],MatrizBrazo[2,3],angles[0],angles[1],angles[2]]
                            EFECTOR.append(efectorFinal)
                        h = h + 1


                datos = datos + 1

            except ValueError:
                print("Mathematical inconsistence")
                
        else:
            print("Incorrect value")
        

    else:
        print("No person in front of the camera")


    ''' Visualization of the image'''
    name_of_window = 'SN: ' + str(device)

    '''Display images''' 
    cv2.namedWindow(name_of_window, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(name_of_window, images)
    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        print("User pressed break key for SN:",device)
        break

'''Filter orientation using a Gaussian Filter'''
print(DATOSPRE)
R1,R2,R3,R4,R5,R6,R7,R8,R9 = smooth_rotations(DATOSPRE,1) # Filter values [0.5, 1]
XEnd,YEnd,ZEnd = smooth_endefector(DATOSPRE,1)
#plot_smoothed_EndEfector(DATOSPRE,XEnd,YEnd,ZEnd)
XElbow,YElbow,ZElbow = smooth_elbow(CORCODOPRE,1)
plot_smoothed_Elbow(CORCODOPRE,XElbow,YElbow,ZElbow)
print("**********************")
#print(R1)
plot_smoothed_rotations(DATOSPRE,R1,R2,R3,R4,R5,R6,R7,R8,R9)

'''Save data filtered'''
#print(DATOSPRE[0][0,0])
for n in range(len(R1)):
    
    MatrizFiltered = np.array([
        [R1[n],R2[n],R3[n],XEnd[n]],
        [R4[n],R5[n],R6[n],YEnd[n]],
        [R7[n],R8[n],R9[n],ZEnd[n]],
        [0,0,0,1]
        ], np.float64)
    DATOS.append(MatrizFiltered)


for n in range(len(XElbow)):
    puntoCodoFilter = np.array([[XElbow[n]],[YElbow[n]],[ZElbow[n]]])
    CORCODO.append(puntoCodoFilter)


print("--------------------------")
print(DATOS)
print("--------------------------")
''' Save all the values in .csv'''
variable = np.asarray(DATOS).shape
print("DATOS: ",variable[0])
DATOS= np.reshape(DATOS, (variable[0]*4, -1))
print(np.asarray(DATOS).shape)
Modelo = pd.DataFrame(DATOS)
Modelo.to_csv('/home/nox/BrazoCamara/ur_ikfast/DatosHumano/DatosBrazoHumano.csv',index=False, header=False) # use the path to wherever you want to save the files

variable2 = np.asarray(CORCODO).shape
CORCODO= np.reshape(CORCODO, (variable2[0]*3, -1))
ModeloCodo = pd.DataFrame(CORCODO)
ModeloCodo.to_csv('/home/nox/BrazoCamara/ur_ikfast/DatosHumano/CodoHumano.csv',index=False, header=False)

ModeloEfectorFinal = pd.DataFrame(EFECTOR)
ModeloEfectorFinal.to_csv('/home/nox/BrazoCamara/ur_ikfast/DatosHumano/EfectorFinal.csv',index=False, header=False)

''' Close the application'''
print("Application Closing")
pipeline.stop()
print("Application Closed.")
plot_smoothed_rotations(DATOSPRE,R1,R2,R3,R4,R5,R6,R7,R8,R9)

