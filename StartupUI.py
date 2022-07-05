from PyQt5 import uic
#from PyQt5.QtGui import *
#from PyQt5.QtWidgets import *
#from PyQt5.QtCore import *
from cv2 import resize,imread
import configparser
#from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image
from Robot import Robot,Order
from PyQt5.QtWidgets import QMessageBox,QApplication
from PyQt5.QtGui import QIcon,QPixmap,QImage
from PyQt5.QtCore import QSize,QTimer
import os

import time
from calibrateUI import CalibrateUI
from MainWindowuUI import MainWindowUI
from ZEDCamera import ZEDCamera
#os.system('chcp 65001')
#os.system("pyuic5 main.ui > ui.py")
IP="127.0.0.1"
IP="192.168.1.104"
#IP="192.168.1.2"
PORT=8002
class StartupUI:
    def __init__(self):
        # 从文件中加载UI定义
        # 从 UI 定义中动态 创建一个相应的窗口对象
        # 注意：里面的控件对象也成为窗口对象的属性了
        # 比如 self.ui.button , self.ui.textEdit
        self.ui = uic.loadUi("ui/Startup.ui")
        self.ui.Background.setPixmap(QPixmap("icon/机械臂.png"))
        self.ui.Background.setScaledContents(True)#图片大小与label适应
        self.ui.ConnectRobotButton.setIcon(QIcon("icon/机器人黑.png"))
        self.ui.ConnectCameraButton.setIcon(QIcon("icon/相机黑.png"))
        self.ui.ConnectRobotButton.setIconSize(QSize(64, 64))
        self.ui.ConnectCameraButton.setIconSize(QSize(64, 64))

        self.ui.robotflag.setPixmap(QPixmap("icon/未连接.png"))
        self.ui.robotflag.setScaledContents(True)#图片大小与label适应
        self.ui.cameraflag.setPixmap(QPixmap("icon/未连接.png"))
        self.ui.cameraflag.setScaledContents(True)#图片大小与label适应
        
        self.ui.ConnectRobotButton.clicked.connect(self.ConnectRobot)
        self.ui.ConnectCameraButton.clicked.connect(self.ConnectCamera)
        self.ui.LoadParameter.clicked.connect(self.LoadParameter)
        self.ui.Run.clicked.connect(self.Run)
        self.ui.calibrate.clicked.connect(self.Calibrate)
        
 
        self.camera=[]
        self.Robot=Robot()
        self.camerastatus=False
    def ConnectRobot(self):
        if self.Robot.RunStatus==False:
            if  self.Robot.connect(IP,PORT):
                self.ui.robotflag.setPixmap(QPixmap("icon/连接成功.png"))
                self.ui.robotflag.setScaledContents(True)#图片大小与label适应
            else:
                self.ui.robotflag.setPixmap(QPixmap("icon/连接失败.png"))
                self.ui.robotflag.setScaledContents(True)#图片大小与label适应
        else:
            self.Robot.close()
            self.Robot.RunStatus=False
    def ConnectCamera(self):
        cameras = ZEDCamera.enum_cameras()
        self.camerastatus=True if len(cameras)>0 else False
        self.ui.Cameralist.clear()
        self.ui.Cameralist_2.clear()
        self.ui.Cameralist.addItem("None")
        self.ui.Cameralist_2.addItem("None")
        for camera in cameras:
            self.ui.Cameralist.addItem(str(camera))
            self.ui.Cameralist_2.addItem(str(camera))
        
        if len(cameras)!=0:
            self.ui.cameraflag.setPixmap(QPixmap("icon/连接成功.png"))
            self.ui.cameraflag.setScaledContents(True)#图片大小与label适应
        else:
            self.ui.cameraflag.setPixmap(QPixmap("icon/连接失败.png"))
            self.ui.cameraflag.setScaledContents(True)#图片大小与label适应
    def opencamera(self):
        cout=0
        
        if self.ui.Cameralist.currentText()!="None":
            cout+=1
            self.camera.append(ZEDCamera(int(self.ui.Cameralist.currentText()), resolution=720, camera_fps=15))
        else:
            self.camera.append(None)
        if self.ui.Cameralist_2.currentText()!="None":
            cout+=1
            self.camera.append(ZEDCamera(int(self.ui.Cameralist_2.currentText()), resolution=720, camera_fps=15))
        else:
            self.camera.append(None)
        return cout
    def Calibrate(self):
        if self.Robot.RunStatus==False:
            QMessageBox.critical(self.ui, "警告", "机器人连接失败!")
            return
        if self.camerastatus == False:
            QMessageBox.critical(self.ui, "警告", "摄像机未连接!")
            return
        if self.ui.Cameralist.currentText() != 'None' and self.ui.Cameralist.currentText()==self.ui.Cameralist_2.currentText():
            QMessageBox.critical(self.ui, "警告", "无法同时加载两个相同的相机!")
            return
        if self.opencamera()>0:
            self.c=CalibrateUI(self.Robot,self.camera)
            self.c.ui.show()
            self.ui.close()
        else:
            QMessageBox.critical(self.ui, "警告", "至少需要加载一个相机!")
            return
        conf = configparser.ConfigParser()
        conf.read(r'ini/Calibrate.ini')
        filename='cameralist'
        if conf.has_section(filename):
            conf.remove_section(filename)
        conf.add_section(filename) #添加section
        conf.set(filename, '全局相机', str(self.ui.Cameralist.currentText()))
        conf.set(filename, '局部相机 ', str(self.ui.Cameralist_2.currentText()))
        with open("ini/"+'Calibrate'+".ini", 'w') as fw:
            conf.write(fw) 
             
    def LoadParameter(self):
        dir_files = os.listdir("ini")  # 得到该文件夹下所有的文件
        self.ui.ParameterList.clear()
        for file in dir_files:
            if file[-4:]==".ini":
                self.ui.ParameterList.addItem(file)
    def Run(self):
        
        if self.Robot.RunStatus==False:
            QMessageBox.critical(self.ui, "警告", "机器人连接失败!")
            return False
        filename=self.ui.ParameterList.currentText()
        if filename=='':
            QMessageBox.critical(self.ui, "警告", "全局相机未加载有效标定数据!")
            return False
        conf = configparser.ConfigParser()
        conf.read("ini/"+filename)
        #conf.read("ini/"+filename,encoding="utf-8")
        TransParams=[]
        self.camera.clear()
        try:
            TransParams.append((conf.getfloat('EyeoutofHand', 'trans_x'),
                              conf.getfloat('EyeoutofHand', 'trans_y'),
                              conf.getfloat('EyeoutofHand', 'trans_z'),
                              conf.getfloat('EyeoutofHand', 'trans_yaw'),
                              conf.getfloat('EyeoutofHand', 'trans_pitch'),
                              conf.getfloat('EyeoutofHand', 'trans_roll')))
            TransParams.append((conf.getfloat('EyeinHand', 'trans_x'),
                              conf.getfloat('EyeinHand', 'trans_y'),
                              conf.getfloat('EyeinHand', 'trans_z'),
                              conf.getfloat('EyeinHand', 'trans_yaw'),
                              conf.getfloat('EyeinHand', 'trans_pitch'),
                              conf.getfloat('EyeinHand', 'trans_roll')))
        except:
            QMessageBox.critical(self.ui, "警告", "相机未加载有效标定数据!")
            self.TransParams=[]
            return False
        try:
            self.camera.append(ZEDCamera(conf.getint('cameralist', '全局相机'), resolution=720, camera_fps=15))
            self.camera.append(ZEDCamera(conf.getint('cameralist', '局部相机'), resolution=720, camera_fps=15))
        except:
            QMessageBox.critical(self.ui, "警告", "摄像机连接失败!")
            return False

        self.m=MainWindowUI(self.Robot,self.camera,TransParams)
        self.m.ui.show()
        self.ui.close()





if __name__=='__main__':
    app = QApplication([])
    stats = StartupUI()
    stats.ui.show()
    app.exec_()
