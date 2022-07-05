

from PyQt5 import uic
#from PyQt5.QtGui import *
#from PyQt5.QtWidgets import *
#from PyQt5.QtCore import *

import configparser
from PyQt5 import QtCore, QtGui, QtWidgets

import cv2
import numpy as np
import math
from math import degrees as dg
from PyQt5.QtWidgets import QMessageBox,QApplication
from PyQt5.QtGui import QIcon, QImageWriter,QPixmap,QImage
from PyQt5.QtCore import QSize,QTimer
from ZEDCamera import ZEDCamera
import time

from Robot import Robot,Order
from HandEyeCaliFunc import PSO_Solver
class CalibrateUI:
    def __init__(self,robot,camera):
        # 从文件中加载UI定义
        self.ui = uic.loadUi("ui/calibrate.ui")
        self.ui.setWindowIcon(QIcon("icon/机械臂.png"))
        self.ui.setWindowTitle("机器人标定软件")
        self.robot=robot
        self.cameras=camera
        
        self.orderRCP=Order("RCP")
        self.order=self.orderRCP
        self.Data_Robot=[]
        self.Data_Camera=[]
        self.camdatatemp=None

        self.ui.sampling_robot.clicked.connect(self.sampling_robot)
        self.ui.sampling_camera.clicked.connect(self.sampling_camera)
        self.ui.deletelastdata.clicked.connect(self.deletelastdata)
        self.ui.saveresult.clicked.connect(self.saveresult)

        self.ui.robotstatus.clicked.connect(self.openorcloserobot)
        self.ui.robotstatus.setIcon(QIcon("icon/机器人黑.png"))
        self.ui.robotstatus.setIconSize(QSize(60, 60))
        self.ui.camerastatus.clicked.connect(self.openorclosecamera)
        self.ui.camerastatus.setIcon(QIcon("icon/相机黑.png"))
        self.ui.camerastatus.setIconSize(QSize(60, 60))
        self.ui.e_stop.clicked.connect(self.e_stop)
        self.ui.e_stop.setIcon(QIcon("icon/急停.png"))
        self.ui.e_stop.setIconSize(QSize(90, 90))
        self.ui.movingmode.clicked.connect(self.movingmode)
        self.ui.movingmode.setIcon(QIcon("icon/示教器.png"))
        self.ui.movingmode.setIconSize(QSize(90, 90))
        self.ui.calibratemode.currentIndexChanged.connect(self.calibratemode)
        self.movingmode="示教器控制"
        self.ui.test.clicked.connect(self.testmove)
        #运动相关
        self.ui.step.valueChanged.connect(self.movestep)
        self.ui.xup.clicked.connect(self.x_up)
        self.ui.xdown.clicked.connect(self.x_down)
        self.ui.yup.clicked.connect(self.y_up)
        self.ui.ydown.clicked.connect(self.y_down)
        self.ui.zup.clicked.connect(self.z_up)
        self.ui.zdown.clicked.connect(self.z_down)
        self.ui.yawup.clicked.connect(self.yaw_up)
        self.ui.yawdown.clicked.connect(self.yaw_down)
        self.ui.pitchup.clicked.connect(self.pitch_up)
        self.ui.pitchdown.clicked.connect(self.pitch_down)
        self.ui.rollup.clicked.connect(self.roll_up)
        self.ui.rolldown.clicked.connect(self.roll_down)

        #坐标显示
        self.PositionDisplayCount=0
        self.timer_refresh = QTimer()
        self.timer_refresh.timeout.connect(self.PositionDisplay)
        if self.robot.RunStatus:
            self.timer_refresh.start(100)

        #图像显示
        self.ImageDisplayCount=0
        self.timer_img = QTimer()
        self.timer_img.timeout.connect(self.ImageDisplay)
        if camera[0] is not None:
            self.camera=camera[0]
            self.timer_img.start(100)
        self.ui.IMG.mousePressEvent=self.mousePressEvent
        self.camerastatus=False

    def __del__(self):
        self.robot=None
        self.camera=None
    def e_stop(self):
        if self.movingmode=="上位机控制" and self.robot is not None :
            self.order=Order("PAU")
            QMessageBox.critical(self.ui, "急停", "已停止移动!")
        else:
            self.printmessage("未有执行移动任务,或者当前为示教器控制")
    def mousePressEvent(self,event):
        '''
        n = event.button() # 用来判断是哪个鼠标健触发了事件【返回值：0 1 2 4】
        QtCore.Qt.NoButton - 0 - 没有按下鼠标键
        QtCore.Qt.LeftButton - 1 -按下鼠标左键
        QtCore.Qt.RightButton - 2 -按下鼠标右键
        QtCore.Qt.Mion 或 QtCore.Qt.MiddleButton -4 -按下鼠标中键
        '''
        if event.button()==QtCore.Qt.LeftButton and self.camera is not None:
            img1=self.camera.RGBimage.copy()
            x=img1.shape[1]*event.x()/self.ui.IMG.width()
            y=img1.shape[0]*event.y()/self.ui.IMG.height()
            img1=cv2.circle(img1,center =(int(x),int(y)),radius = 10,color = (0,0,255),thickness = 3)
            if x<self.ui.IMG_2.width()/2:
                x_min=0
                x_max=self.ui.IMG_2.width()
            elif x>img1.shape[1]-self.ui.IMG_2.width()/2:
                x_min=img1.shape[1]-self.ui.IMG_2.width()
                x_max=img1.shape[1]
            else:
                x_min=int(x-self.ui.IMG_2.width()/2)
                x_max=int(x+self.ui.IMG_2.width()/2)
            if y<self.ui.IMG_2.height()/2:
                y_min=0
                y_max=self.ui.IMG_2.height()
            elif y>img1.shape[0]-self.ui.IMG_2.height()/2:
                y_min=img1.shape[0]-self.ui.IMG_2.height()
                y_max=img1.shape[0]
            else:
                y_min=int(y-self.ui.IMG_2.height()/2)
                y_max=int(y+self.ui.IMG_2.height()/2)
            img_=img1[y_min:y_max,x_min:x_max].copy()

            frame = cv2.cvtColor(img_, cv2.COLOR_RGB2BGR)
            img = QImage(frame.data, frame.shape[1], frame.shape[0],frame.shape[1]*3, QImage.Format_RGB888)#第四个参数设置通道数对齐,不然图片可能会变形
            self.ui.IMG_2.setPixmap(QPixmap.fromImage(img))
            self.ui.IMG_2.setScaledContents(True)#图片大小与label适应
           

            X=self.camera.Xmap[int(y),int(x)]
            Y=self.camera.Ymap[int(y),int(x)]
            Z=self.camera.Zmap[int(y),int(x)]
            #self.camdatatemp=[round(X,2),round(Y,2),round(Z,2)]
            #p3=Z*np.linalg.inv(K)@np.array([[int(x)],[int(y)],[1]])
            self.camdatatemp=[round(X,2),round(Y,2),round(Z,2)]
            self.printmessage("相机坐标中的位置为："+str(round(X,2))+","+str(round(Y,2))+","+str(round(Z,2)))
            #self.printmessage("图像坐标中的位置为："+str(round(p3[0,0],2))+","+str(round(p3[1,0],2))+","+str(round(p3[2,0],2)))
    def testmove(self):
        if self.ui.calibratemode.currentText()=="全局相机":
            filename='EyeoutofHand'
        else:
            filename='EyeinHand'
        conf = configparser.ConfigParser()
        conf.read("ini/"+'Calibrate'+".ini")
        try:
            dx=conf.getfloat('EyeoutofHand', 'trans_x')
            dy=conf.getfloat('EyeoutofHand', 'trans_y')
            dz=conf.getfloat('EyeoutofHand', 'trans_z')
            rz=conf.getfloat('EyeoutofHand', 'trans_yaw')
            ry=conf.getfloat('EyeoutofHand', 'trans_pitch')
            rx=conf.getfloat('EyeoutofHand', 'trans_roll')
        except:
            QMessageBox.critical(self.ui, "警告", "未找到有效标定数据!")
            return
        if self.camdatatemp is None:
            QMessageBox.critical(self.ui, "警告", "请点击目标点!")
            return 
        CameraXYZ=np.array(self.camdatatemp)
        pos_euler=np.array([dx,dy,dz,rz,ry,rx])
        RobotXYZ = PSO_Solver._PSO_Solver__calc_hand(pos_euler, CameraXYZ)
        self.camdatatemp=None
        self.printmessage(str(RobotXYZ))
    def ImageDisplay(self):
        if self.camera.refresh():
            self.ImageDisplayCount+=1
            self.ui.sampling_camera.setEnabled(True)
            self.camerastatus=True
            img1=self.camera.get_RGBimage(ROIonly=False, width=None, mark_infeasible=True)
            #img1=self.camera.RGBimage
            frame = cv2.cvtColor(img1, cv2.COLOR_RGB2BGR)
            img = QImage(frame.data, frame.shape[1], frame.shape[0], frame.shape[1]*3,QImage.Format_RGB888)
            self.ui.IMG.setPixmap(QPixmap.fromImage(img))
            self.ui.IMG.setScaledContents(True)#图片大小与label适应
            if self.ImageDisplayCount%15>7:
                self.ui.camerastatus.setIcon(QIcon("icon/相机黑.png"))
            else:
                self.ui.camerastatus.setIcon(QIcon("icon/相机绿.png"))
        else:
            self.camerastatus=False
            QMessageBox.critical(self.ui, "警告", "相机连接中断!")
            self.ui.camerastatus.setIcon(QIcon("icon/相机红.png"))
            self.timer_img.stop()
            self.ui.sampling_camera.setEnabled(False)
    def saveresult(self): 
        hand_points=np.array(self.Data_Robot)[:,:3]
        eye_points=np.array(self.Data_Camera)
        init_pos_euler=np.array([float(self.ui.initX.value()),float(self.ui.initY.value()),\
            float(self.ui.initZ.value()),float(self.ui.initYAW.value()),\
            float(self.ui.initPITCH.value()),float(self.ui.initROLL.value())])
        cali = PSO_Solver(init_pos_euler, eye_points, hand_points)
        error=cali._PSO_Solver__calc_cost(cali.pos_euler,eye_points,hand_points)
        self.printmessage("当前标定平均误差为："+str(round(error,2)))
        if error>20:
            QMessageBox.critical(self.ui, "警告", "误差过大!")
            return
        if self.ui.calibratemode.currentText()=="全局相机":
            filename='EyeoutofHand'

        else:
            filename='EyeinHand'

        conf = configparser.ConfigParser()
        conf.read("ini/"+'Calibrate'+".ini")
        conf.defaults()
        if conf.has_section(filename):
            conf.remove_section(filename)
        conf.add_section(filename) #添加section
        conf.set(filename, 'trans_x ', str(cali.pos_euler[0]))
        conf.set(filename, 'trans_y ', str(cali.pos_euler[1]))
        conf.set(filename, 'trans_z ', str(cali.pos_euler[2]))
        conf.set(filename, 'trans_yaw ', str(cali.pos_euler[3]))
        conf.set(filename, 'trans_pitch ', str(cali.pos_euler[4]))
        conf.set(filename, 'trans_roll ', str(cali.pos_euler[5]))
        with open("ini/"+'Calibrate'+".ini", 'w') as fw:
            conf.write(fw)

    def deletelastdata(self):
        if len(self.Data_Robot)>0:
            if self.ui.calibratemode.currentText()=="全局相机":
                self.Data_Robot.pop()
        if len(self.Data_Camera)>0:
            self.Data_Camera.pop()
        self.printmessage("当前机器人坐标采样点"+str(len(self.Data_Robot))+"个;"+"相机坐标采样点"+str(len(self.Data_Camera))+"个")

    def sampling_camera(self):
        self.Data_Camera.append(self.camdatatemp.copy())
        self.camdatatemp.clear()
        self.printmessage("当前机器人坐标采样点"+str(len(self.Data_Robot))+"个;"+"相机坐标采样点"+str(len(self.Data_Camera))+"个")
    def sampling_robot(self):
        if self.ui.calibratemode.currentText()=="局部相机":
            self.Data_Robot.clear()
        self.Data_Robot.append(self.robot.CurPos)
        pos="("+str(self.robot.CurPos[0])+","+str(self.robot.CurPos[1])+","+str(self.robot.CurPos[2])+")"
        self.printmessage("机器人坐标采样"+pos)
        self.printmessage("当前机器人坐标采样点"+str(len(self.Data_Robot))+"个;"+"相机坐标采样点"+str(len(self.Data_Camera))+"个")
    def openorclosecamera(self):
        if self.camera is not None:
            self.camera.camera.close()
        if self.camerastatus==False:
            conf = configparser.ConfigParser()
            conf.read("ini/"+'Calibrate'+".ini")
            try:
                if self.ui.calibratemode.currentText()=="全局相机":
                    if conf.get('cameralist', '全局相机')!='None':
                        self.cameras[0]=ZEDCamera(conf.getint('cameralist', '全局相机'), resolution=720, camera_fps=15)
                        self.camera=self.cameras[0]
                    else:
                        QMessageBox.critical(self.ui, "警告", "未发现相机!")
                        return
                else:
                    if conf.get('cameralist', '局部相机')!='None':
                        self.cameras[1]=ZEDCamera(conf.getint('cameralist', '局部相机'), resolution=720, camera_fps=15)
                        self.camera=self.cameras[1]
                    else:
                        QMessageBox.critical(self.ui, "警告", "未发现相机!")
                        return
                self.timer_img.start(100)
                QMessageBox.information(self.ui, "成功", "相机连接成功!")
            except:
                QMessageBox.critical(self.ui, "警告", "相机连接失败!")

    def openorcloserobot(self):
        if self.robot.RunStatus==False:
            if self.robot.reconnect():
                if self.movingmode=="上位机控制":
                    self.order=Order("STA")
                self.timer_refresh.start(100)
                QMessageBox.information(self.ui, "成功", "已连接到机器人!")
            else:
                QMessageBox.critical(self.ui, "失败", "连接超时!")
        else:
            self.robot.close()
            self.robot.RunStatus=False
    def movingmode(self):
        if self.movingmode=="示教器控制":
            self.movingmode="上位机控制"
            self.ui.movingmode.setIcon(QIcon("icon/上位机.png"))
            self.ui.movingmode.setIconSize(QSize(90, 90))
            self.printmessage("当前切换到<上位机控制>模式")
            self.order=Order("STA")
        elif self.movingmode=="上位机控制":
            self.movingmode="示教器控制"
            self.ui.movingmode.setIcon(QIcon("icon/示教器.png"))
            self.ui.movingmode.setIconSize(QSize(90, 90))
            self.printmessage("当前切换到<示教器控制>模式")
            self.order=Order("STO")
    def calibratemode(self):
        if self.ui.calibratemode.currentText()=="全局相机":
            if self.cameras[0] is not None:
                self.timer_img.start(100)
            else:
                QMessageBox.critical(self.ui, "失败", "全局相机不在线!")
                self.timer_img.stop()
                self.ui.camerastatus.setIcon(QIcon("icon/相机红.png"))
                self.camerastatus=False
            self.camera=self.cameras[0]
        elif self.ui.calibratemode.currentText()=="局部相机":
            if self.cameras[1] is not None:
                self.timer_img.start(100)
            else:
                QMessageBox.critical(self.ui, "失败", "局部相机不在线!")
                self.timer_img.stop()
                self.ui.camerastatus.setIcon(QIcon("icon/相机红.png"))
                self.camerastatus=False
            self.camera=self.cameras[1]
    def x_up(self):
        offset=[int(self.ui.step.value()),0,0,0,0,0]
        self.Moving(offset)
    def x_down(self):
        offset=[-int(self.ui.step.value()),0,0,0,0,0]
        self.Moving(offset)
    def y_up(self):
        offset=[0,int(self.ui.step.value()),0,0,0,0]
        self.Moving(offset)
    def y_down(self):
        offset=[0,-int(self.ui.step.value()),0,0,0,0,0]
        self.Moving(offset)
    def z_up(self):
        offset=[0,0,int(self.ui.step.value()),0,0,0]
        self.Moving(offset)
    def z_down(self):
        offset=[0,0,-int(self.ui.step.value()),0,0,0]
        self.Moving(offset)
    def yaw_up(self):
        offset=[0,0,0,round(float(self.ui.step.value())/10,2),0,0]
        self.Moving(offset)
    def yaw_down(self):
        offset=[0,0,0,round(-float(self.ui.step.value())/10,2),0,0]
        self.Moving(offset)
    def pitch_up(self):
        offset=[0,0,0,0,round(float(self.ui.step.value())/10,2),0]
        self.Moving(offset)
    def pitch_down(self):
        offset=[0,0,0,0,round(-float(self.ui.step.value())/10,2),0]
        self.Moving(offset)
    def roll_up(self):
        offset=[0,0,0,0,0,round(float(self.ui.step.value())/10,2)]
        self.Moving(offset)
    def roll_down(self):
        offset=[0,0,0,0,0,round(-float(self.ui.step.value())/10,2)]
        self.Moving(offset)
    def movestep(self):
        self.ui.stepnumber.display(self.ui.step.value())
    def Moving(self,offset):
        pos=self.robot.CurPos.copy()
        if self.movingmode=="上位机控制":
            Coordinate=self.ui.Coordinate.currentText()
            if Coordinate=="基坐标系":
                order=Order("MEC")
                order.AddPos([pos[0]+offset[0],pos[1]+offset[1],pos[2]+offset[2],pos[3]+offset[3],pos[4]+offset[4],pos[5]+offset[5]])
            elif Coordinate=="工具坐标系":
                order=Order("MIC")
                order.AddPos([offset[0],offset[1],offset[2],offset[3],offset[4],offset[5]])
            else:
                order=Order("MJO")
                order.AddPos([offset[0]/10,offset[1]/10,offset[2]/10,offset[3],offset[4],offset[5]])
            v=self.ui.stepv.value()
            t=self.ui.stept.value()
            if int(t)!=0:
                order.AddTimeorV(1,t)
            else:
                order.AddTimeorV(0,v)
            order.AddPath()
            self.order=order
        else:
            QMessageBox.information(self.ui, "提示", "当前为示教器控制!")
    def PositionDisplay(self):
        if self.PositionDisplayCount%2==0:
            self.robot.send(self.order)
            self.order=self.orderRCP
            if self.PositionDisplayCount%10<5:
                self.ui.robotstatus.setIcon(QIcon("icon/机器人黑.png"))
        else:
            self.robot.receive()
            if self.PositionDisplayCount%10>=5:
                self.ui.robotstatus.setIcon(QIcon("icon/机器人绿.png"))
            if self.robot.Complete:
                pos="("+str(self.robot.CurPos[0])+","+str(self.robot.CurPos[1])+","+str(self.robot.CurPos[2])+")"
                self.printmessage("移动到"+pos)
                self.order=Order("ROK")
            if self.robot.RunStatus==False:
                self.timer_refresh.stop()
                QMessageBox.critical(self.ui, "警告", "机器人连接中断!")
                self.ui.robotstatus.setIcon(QIcon("icon/机器人红.png"))
            
            self.ui.transX.display(self.robot.CurPos[0])
            self.ui.transY.display(self.robot.CurPos[1])
            self.ui.transZ.display(self.robot.CurPos[2])
            self.ui.YAW.display(self.robot.CurPos[3])
            self.ui.PITCH.display(self.robot.CurPos[4])
            self.ui.ROLL.display(self.robot.CurPos[5])   
        
            
            
        self.PositionDisplayCount+=1

    def printmessage(self,message):
        textCursor = self.ui.message.textCursor()
        textCursor.movePosition(textCursor.End)
        self.ui.message.setTextCursor(textCursor)
        prostr='<'+str(time.strftime("%H:%M:%S", time.localtime()))+'> :'
        self.ui.message.insertPlainText(prostr+message+"\r\n")
        textCursor.movePosition(textCursor.End)
        self.ui.message.setTextCursor(textCursor)
