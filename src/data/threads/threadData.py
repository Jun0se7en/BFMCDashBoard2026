import cv2
import threading
import socket
import base64
import time
import numpy as np
import os
import io
import csv
import json
import random
import pandas as pd
from scipy.spatial.transform import Rotation as R

from multiprocessing import Pipe
from src.templates.threadwithstop import ThreadWithStop
import struct
import pickle
from PyQt5.QtWidgets import QMainWindow, QApplication, QGraphicsColorizeEffect
from PyQt5.QtGui import QFontDatabase, QPixmap, QImage
from PyQt5.QtCore import QThread, QEvent, pyqtSignal

import folium
from ui_interface import *

def procrustes_transform(P, Q):
    # P, Q shape = (N,2)
    # Center data
    P_centered = P - P.mean(axis=0)
    Q_centered = Q - Q.mean(axis=0)

    # SVD
    U, _, Vt = np.linalg.svd(P_centered.T @ Q_centered)
    R = U @ Vt

    # Scale
    scale = np.trace(Q_centered.T @ P_centered @ R) / np.trace(P_centered.T @ P_centered)

    # R_fix = np.eye(2)

    # Translation
    t = Q.mean(axis=0) - scale * (R @ P.mean(axis=0))

    return scale, R, t

# Ánh xạ điểm mới p:
def map_point(p, scale, R, t):
    return scale * (p @ R.T) + t


class threadData(QThread):
    DataUpdate = pyqtSignal(int, int, list, list)

    def __init__(self, ip_address, port):
        super().__init__()
        self.ThreadActive = True
        self.PORT = port
        self.SERVER_ADDRESS = ip_address
        self.connect_flag = False
        self.prev_state = ""
        self.state = ""
        self.fps = 0
        self.speed = 0
        self.angle = 0
        
        
        ##### Map Anchor #####
        real_anchor = []
        
        # Load the CSV file
        for i in range(1, 9):
            df = pd.read_csv(f'./points/point{i}.csv')
            real_anchor.append((np.mean(df['x']), np.mean(df['y']), np.mean(df['z'])))

        real_anchor = np.array(real_anchor, dtype=float)

        print(real_anchor)

        ### Rotation and Scaling ###
        rotation_degrees = 153
        rotation_radians = np.radians(rotation_degrees)
        rotation_axis = np.array([0, 0, 1])

        rotation_vector = rotation_radians * rotation_axis
        self.rotation = R.from_rotvec(rotation_vector)

        rotation_anchor = self.rotation.apply(real_anchor)

        map_anchor = [(21, 380), (212, 244), (244, 185), (186, 154), (154, 212), (378, 20), (154, 365), (154, 59)]
        map_anchor = np.array(map_anchor, dtype=float)  # (N,2)
        rotation_anchor = rotation_anchor[:, :2]  # Chỉ lấy 2D (bỏ z)
        rotation_anchor = np.array(rotation_anchor, dtype=float)  # (N,2)
        
        self.scale, self.R, self.t = procrustes_transform(rotation_anchor, map_anchor)

        self.x_raw = 0
        self.y_raw = 0

    def run(self):
        self.ThreadActive = True
        # self.client_socket = socket.socket()  # instantiate
        # print(f'Socker with {self.PORT} created')
        # while not self.connect_flag:
        #     try:
        #         self.client_socket.connect((self.SERVER_ADDRESS, self.PORT))  # connect to the server
        #         self.connect_flag = True
        #     except:
        #         print('Connecting Failed!!! Retrying....')
        #         pass

        # data = b""
        # payload_size = struct.calcsize("Q")
        while self.ThreadActive:
            # while len(data) < payload_size:
            #     packet = self.client_socket.recv(4*1024)
            #     if not packet: break
            #     data+=packet
            # packed_msg_size = data[:payload_size]
            # data = data[payload_size:]
            # msg_size = struct.unpack("Q",packed_msg_size)[0]
            
            # while len(data) < msg_size:
            #     data += self.client_socket.recv(4*1024)
            # frame_data = data[:msg_size]
            # data  = data[msg_size:]
            # frame = pickle.loads(frame_data)
            # print(frame)
            # # print("Data:",frame)
            # position = frame["Position"]
            # if position:
            #     x = int(position[0] + 0.5)
            #     y = int(position[1] + 0.5)
            # else:
                # x = 0
                # y = 0
            
            x = 0
            y = 0

            stats = [[random.randint(89, 100), random.randint(89, 100), random.randint(89, 100), random.randint(89, 100), random.randint(89, 100), random.randint(89, 100), random.randint(89, 100), random.randint(85, 90), random.randint(35,40)], random.randint(30,40)]

            # frame["Stats"][-3] = int(frame["Stats"][-3]*100)
            # stats = [list(map(int,frame["Stats"][:len(frame["Stats"])-2])), int(frame["Stats"][-1])]

            # if frame["CarStats"] is not None:
            #     self.fps = frame["CarStats"][0]
            #     self.state = frame["CarStats"][1]
            #     self.speed = int(frame["CarStats"][2]/2)
            #     self.angle = int(frame["CarStats"][3])

            self.speed = int(random.uniform(28,30))
            self.angle = 0
            self.fps = random.randint(5, 10)
            self.state = None
            
            label = [x, y, self.state, self.prev_state, random.randint(4,5), int(self.fps)]

            self.DataUpdate.emit(self.speed, self.angle, label, stats)

            if self.prev_state != self.state:
                self.prev_state = self.state

            time.sleep(1)

            # phát tín hiệu về Main Thread
            
        # When everything done, release the socket
        self.client_socket.close()
        self.terminate()

    def stop(self):
        self.ThreadActive = False
        self.terminate()
