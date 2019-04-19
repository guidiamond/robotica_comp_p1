#! /usr/bin/env python2
# -*- coding:utf-8 -*-

from __future__ import print_function

import rospy
import numpy as np
import tf
import math
import cv2
import time
import visao_module
import cormodule
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt8
from math import pi
import argparse, time
import imutils

ap.add_argument("-t", "--tracker", type=str, default="kcf",
    help="OpenCV object tracker type")
ap.add_argument("-c", "--confidence", type=float, default=0.7,
    help="Minimum probability to filter weak detections")
ap.add_argument("-o", "--object", type=str, default='cat',
    help="Object to be tracked")
args = vars(ap.parse_args())


OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.TrackerCSRT_create,
    "kcf": cv2.TrackerKCF_create,
    "boosting": cv2.TrackerBoosting_create,
    "mil": cv2.TrackerMIL_create,
    "tld": cv2.TrackerTLD_create,
    "medianflow": cv2.TrackerMedianFlow_create,
    "mosse": cv2.TrackerMOSSE_create}
tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()

bridge = CvBridge()

cv_image = None

media = []
centro = []
media_cor = []
centro_cor = []

atraso = 1.5E9 # ns (1s30)
viu_obj = False
is_centered = False
objeto = args["object"]

area = 0.0 
check_delay = False 
print_delay = False
coef=1
dist_obj_centro_imagem_x = 0
x_obj = 0
res_x = 640
res_y = 480

dist = 0
dist2 = None

contador_objeto = 0
initBB = None
box = []
detect_counter = 0
detect_mode = True

# Medidor de distancia
def scaneou(dado):
    global dist
    global dist2
    dist = np.array(dado.ranges)[0]
    dist2 = list(dado.ranges)

bump = 0

def laser_scan(distancia):
    if distancia == None:
        return

    for idx, val in distancia:
        if 0 < val < 0.3:
            if idx <= 90:
                return 1 #direita
            elif 270 <= idx <= 360:
                return -1 #esquerda
    return
# Bumper
def bumpeiras(dado):
    global bump
    bump = dado.data
    print('Bump:',bump)

# Roda frame a frame
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
    "sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
net = cv2.dnn.readNetFromCaffe('MobileNetSSD_deploy.prototxt.txt', 'MobileNetSSD_deploy.caffemodel')

def CamView(imagem):
    global cv_image
    global media
    global centro
    
    global media_cor
    global centro_cor
    global area_cor

    global viu_obj
    global is_centered
    global coef
    global objeto
    global x_obj
    global dist_obj_centro_imagem_x
    global contador_objeto
    global detect_mode
    global initBB
    global box

    # Lag detection
    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime
    delay = lag.nsecs
    if print_delay:
        print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay:
        print("Descartando por causa do delay do frame:", delay)
        return None

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        media_cor, centro_cor, area_cor = cormodule.identifica_cor(cv_image)

        if detect_mode:
            centro, imagem, resultados = visao_module.processa(cv_image)

            viu_obj = False
            resultados = [i for i in resultados if i[0]==objeto]
            # Procura pelo objeto
            for r in resultados:
                # if r[0] == objeto:
                viu_obj = True
                # Dimensoes do objeto
                x0_obj, y0_obj = r[2]
                x1_obj, y1_obj = r[3]
                posicao_obj = (int((x0_obj+x1_obj)/2.0), int((y0_obj+y1_obj)/2.0))
                # Dimensoes da imagem
                x_obj, y_obj = posicao_obj
                centro_imagem_x = res_x / 2.0
                dist_obj_centro_imagem_x = x_obj - centro_imagem_x

            if viu_obj:
                contador_objeto += 1
                if contador_objeto >= 5:
                    detect_mode = False
                    contador_objeto = 0
                    (x1, y1), (x2, y2) = resultados[0][2:]
                    initBB = (x1, y1, x2, y2)
                    tracker.init(cv_image, initBB)
                    print("\n\nTracking\n\n")
            else:
                contador_objeto = 0

        if not detect_mode:
            # TRACKER AQUI
            if initBB is not None: # TODO
                # grab the new bounding box coordinates of the object
                (success, box_) = tracker.update(cv_image)

                if success: # check to see if the tracking was a success
                    viu_obj = True
                    # print("B", box_)
                    box = [int(v) for v in box_]
                    (x, y, w, h) = box
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 255, 0), 2)

                else:
                    detect_mode = True
            else:
                detect_mode = True

        cv2.imshow("Camera", cv_image)

    except CvBridgeError as e:
        print('ex', e)


def detect(frame):
    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and predictions

    print("[INFO] computing object detections...")
    net.setInput(blob)
    detections = net.forward()

    results = []

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with the prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is greater than the minimum confidence
        if confidence > args["confidence"]:
            # extract the index of the class label from the `detections`, then
            # compute the (x, y)-coordinates of the bounding box for the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            startX, startY, endX, endY = box.astype("int")

            # display the prediction
            label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
            print("[INFO] {}".format(label))
            cv2.rectangle(image, (startX, startY), (endX, endY), COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(image, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
            
            # ("CLASS", confidence, (x1, y1, x2, y2))
            results.append((CLASSES[idx], confidence*100, (startX, startY), (endX, endY)))

    return image, results

if __name__ == "__main__":
    rospy.init_node("cor")

    # Subscribers
    recebedor = rospy.Subscriber("/kamera", CompressedImage, CamView, queue_size=4, buff_size = 2**24)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    bumper = rospy.Subscriber('/bumper', UInt8, bumpeiras)

    try:
        while not rospy.is_shutdown():
            sensor_laser_scan = laser_scan(dist2)
            if sensor_proximidade == None:
                if bump:
                    if bump == 1:
                        vel = Twist(Vector3(-1.5, 0, 0), Vector3(0, 0, -1))
                    elif bump == 2:
                        vel = Twist(Vector3(-1.5, 0, 0), Vector3(0, 0, 1))
                    elif bump == 3:
                        vel = Twist(Vector3(0.4, 0, 0), Vector3(0, 0, pi/4))
                    else:
                        vel = Twist(Vector3(0.4, 0, 0), Vector3(0, 0, -pi/4))
                    bump = 0
                    velocidade_saida.publish(vel)
                    rospy.sleep(3)
                    continue

                # Tracking
                if viu_obj and not detect_mode and box:
                    print("Tracking objeto")
                    x_obj = box[0]
                    x_obj_width = box[2]
                    media_x = x_obj + x_obj_width / 2
                    centrox_x = centro[0]
                    if media_x - centro_x > 20:
                        vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,-0.6))
                    elif media_x - centro_x < 20:
                        vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,0.6))
                    elif media_x - centro_x < 0 or abs(media_x-centro_x) < 20:
                        vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.1)
                    continue

                if not viu_obj: # and not detect_mode:
                    # Resetar
                    detect_mode = False
                    contador_objeto = 0

                # Segue COR
                if type(media_cor) != tuple and len(centro_cor) != 0:
                    if media_cor[0] > centro_cor[0]:
                        vel = Twist(Vector3(0.2, 0, 0), Vector3(0,0,-0.3))
                    elif media_cor[0] < centro_cor[0]:
                        vel = Twist(Vector3(0.2, 0, 0), Vector3(0,0,0.3))
                    print("Girando em direção a cor")
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.2)
                else:
                    print("Girando sem rumo")
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0.4))
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.2)
            elif sensor_proximidade == -1:
                print("Girando Esquerda")
                vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0.6))
                velocidade_saida.publish(vel)
                rospy.sleep(1)
            elif sensor_proximidade == 1:
                print("Girando Direita")
                vel = Twist(Vector3(0.2,0,0), Vector3(0,0,-0.6))
                velocidade_saida.publish(vel)
                rospy.sleep(1)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
