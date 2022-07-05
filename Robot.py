import socket
class Order():
    def __init__(self,order):
        
        if  order=="MEC" or order=="MIC" or order=="MJO":
            self.moveing=True 
        else:
            self.moveing=False
        self.order=order
        self.path=None
        self.Paths=[]
    def AddPos(self,pos=[]):
        if len(pos)==6:
            self.path=pos #x,y,z,yaw,pitch,roll
            self.path.append(250) #v
            self.path.append(3) #time
    def AddTimeorV(self,mode,torv): #mode 1 时间 0 速度
        if mode==1:
            self.path[7]=1 if torv<1 else torv
        else:
            self.path[7]=0
            self.path[6] = torv if torv<250 else 250
    def AddPath(self):
        if len(self.path)==8:
            self.Paths.append(self.path.copy())
            self.path=None
    def GenerateOrder(self):
        if self.moveing:
            order=self.order+":"+str(len(self.Paths))+"\r\n"
            for p in self.Paths:
                order+=str(p[0])+","+str(p[1])+","+str(p[2])+","+str(p[3])+","+str(p[4])+","+str(p[5])+","+str(p[6])+","+str(p[7])+"\r\n"
            return order
        else:
            return self.order+"\r\n"
class Robot(object):
    """description of class"""
    def __init__(self):
        self.robot= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot.settimeout(1)
        self.CurPos=None
        self.RunStatus=False
        self.movepointlast=0
        self.Enable=False
        self.Complete=False
        self.IP=None
        self.PORT=None
    def __del__(self):
        self.robot.close()
    def connect(self,IP,PORT):
        self.IP=IP
        self.PORT=PORT
        try:
            self.robot.connect((IP, PORT))
            self.RunStatus=True
        except socket.error as socketerror:
            self.RunStatus=False
        return self.RunStatus
    def close(self):
        self.robot.close()
    def reconnect(self):
        self.robot= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot.settimeout(1)
        return self.connect(self.IP,self.PORT)
    def send(self,order):
        if self.RunStatus:
            try:
                if order.moveing:
                    self.movepointlast=len(order.Paths)
                    self.robot.send(order.GenerateOrder().encode('utf-8'))
                else:
                    self.robot.send(order.order.encode('utf-8'))
            except:
                self.RunStatus=False

    def receive(self):
        try:
            rec=self.robot.recv(100).decode()
        except:
            self.RunStatus=False
            return
        rec=rec.split('\r\n')
        if rec[0]=="RCP:":
            self.Enable=True if rec[2]=='1' else False
            self.Complete=True if int(rec[1])>self.movepointlast else False
            self.Ready=True if self.Enable and rec[1]=='0' else False
            self.Error=True if int(rec[1])<0 else False
            self.CurPos=rec[3].split(',')
            #self.CurPos=rec[4].split(',')
            CurPos=[]
            for i in self.CurPos:
                CurPos.append(round(float(i),2))
            self.CurPos=CurPos
        elif rec[0]=="ROK":
            self.movepointlast=0
            self.Complete=False
if __name__=='__main__':
    IP="127.0.0.1"
    PORT=8002
    r=Robot()
    rcp=Order("RCP")
    r.connect(IP,PORT)
    r.send(rcp)
    r.receive()