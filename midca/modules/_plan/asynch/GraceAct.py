class GraceMidcaAct():
    def __init__(self):
        gracePath = '/home/pi/Desktop/Grace_Control'
        self.bottomDepth = 0
        self.gracePath = gracePath
        import sys
        sys.path.insert(0, gracePath)
        import GliderFunIPC
        self.interface = GliderFunIPC.GraceFun()
        self.Regulating=False
        self.targetDepth = 0
        import threading
        self.regulateDepthTread = threading.Thread(target=self.regulateDepth)

    def communicateDepth(self, depth):  # returns boolean telling if depth was sent out over xbee
        grace = self.interface
        return grace.sendXbeeMsg("Depth:" + str(depth))

    def dive(self):  # fills robot tank with water to make it sink
        grace = self.interface
        # grace.movePump(4)
        grace.moveMass(4)

    def rise(self):  # expells water from robot to make it float
        grace = self.interface
        # grace.movePump(98)
        grace.moveMass(98)
        
    def gotToDepth(self,target):#target in meters
        self.targetDepth=target
        self.regulateDepthTread.start()
        
    def stopRegulation(self):
        self.Regulating=False
        
    def regulateDepth(self,kp=5,kd=0,ki=1,hz=1):  # regulates depth to target in meters
        from time import sleep
        grace = self.interface
        self.Regulating=True
        target=self.targetDepth
        err = target - grace.readDepth()
        last_err = err
        total_err=0
        center=50
        while self.Regulating:
            u = min(max(center+kp*err+kd*(err-last_err)*hz+ki*total_err/hz,0),99)
            # grace.movePump(u)
            print u
            grace.moveMass(u)
            sleep(1/hz)
            last_err = err
            total_err+=err
            total_err= max(min(100-center,total_err),-center)
            err = target - grace.readDepth()
            print u
        
    def senseDepth(self):  # reads the pressure sensor and converts it to depth in meters
        grace = self.interface
        return grace.readDepth()

    def checkCommunicationAck(self):  # read a file output by program chechinkg for surface and return true or false
        # sending $%GO%$ over xbee will cause a file "Next_Dive_GO" to be produce with 1 in line one
        Acknowleged = False
        gracePath = self.gracePath + "/"
        try:
            f = open(gracePath + "Next_Dive_GO", 'r')
            Acknowleged = (1 == int(f.readline()))
            f.close()
            if Acknowleged:
                f = open(gracePath + "Next_Dive_GO", 'w')
                f.write("0")
                f.close()
        except:
            return False
        return Acknowleged
