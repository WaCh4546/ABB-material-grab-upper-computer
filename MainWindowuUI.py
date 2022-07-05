
from PyQt5 import uic
#from PyQt5.QtGui import *
#from PyQt5.QtWidgets import *
#from PyQt5.QtCore import *
from cv2 import resize,imread
import configparser
from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image

from PyQt5.QtWidgets import QMessageBox,QApplication
from PyQt5.QtGui import QIcon,QPixmap,QImage
from PyQt5.QtCore import QSize,QTimer
import os
import socket
import time

class MainWindowUI:
    def __init__(self,robot,camera,TransParams):
        self.ui = uic.loadUi("ui/main.ui")
        self.robot=robot
    def __del_(self):
        self.robot=None
