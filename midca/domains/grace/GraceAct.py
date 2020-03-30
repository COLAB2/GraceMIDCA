from time import sleep
import random
import threading
from scipy.integrate import odeint
import numpy as np
from numpy import cross, sin, cos

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
        self.regulateDepthTread = threading.Thread(target=self.regulateDepth)
        self.simulating = False
        self.simulationThread = threading.Thread(target=self._SIMULATE)
        self.rp1 = 0
        self.delta = 0
        self.m0 = 0
        self.currentState = np.array([0,0,0,.0001,0,.0001,0,0,0,1,0,0,0,1,0,0,0,1])
        self.currentState.shape=(18,1)
        
    def communicateDepth(self, depth):  # returns boolean telling if depth was sent out over xbee
        grace = self.interface
        if self.simulating:
            if self.senseDepth() > .5:
                return 0
            else:
                if random.random() > 0.8:
                    f = open(self.gracePath + "Next_Dive_GO", 'w')
                    f.write("1")
                    f.close()
                return 1
        return grace.sendXbeeMsg("Depth:" + str(depth))

    def dive(self):  # fills robot tank with water to make it sink
        if self.simulating:
            self.m0 = -.1
        else:
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
        self.regulateDepthTread = threading.Thread(target=self.regulateDepth)
        
    def regulateDepth(self,kp=10,kd=1,ki=.1,hz=1):  # regulates depth to target in meters
        grace = self.interface
        self.Regulating=True
        target=self.targetDepth
        err = target - self.senseDepth()
        last_err = err
        total_err=0
        center=50.0
        while self.Regulating:
            u = min(max(center+kp*err+kd*(err-last_err)*hz+ki*total_err/hz,0),99)
            # grace.movePump(u)
            if self.simulating:
                self.m0 = (u-center)/center*.1
                #print self.m0,u
            else:
                grace.moveMass(u)
            sleep(1/hz)
            last_err = err
            total_err+=err
            total_err= max(min(100-center,total_err),-center)
            err = target - self.senseDepth()
            #print 'error: ',err
            #print 'Depth: ',self.currentState[2,0]
            
        
    def senseDepth(self):  # reads the pressure sensor and converts it to depth in meters
        if self.simulating:
            return np.copy(self.currentState).flatten()[2]
        grace = self.interface
        return grace.readDepth()
        
    def estimateDepthRate(self,dt=0.5):  # estimates depth by taking two depth measuremeants over time (dt)
        depth1=self.senseDepth()
        sleep(dt)
        return (self.senseDepth()-depth1)/dt
        
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
    
    def startSimulation(self):
        self.simulationThread.start()
        
    def stopSimulation(self):
        self.simulating=False
        
    def _SIMULATE(self,tsim=.05):
        self.simulating=True
        i=0
        while self.simulating:
            x=np.copy(self.currentState).flatten()
            #sol = solve_ivp(lambda t,y: grace_dynam(t,y,self.rp1,self.m0,self.delta,[0,tsim],x))
            #sol = odeint(grace_dynam2,x,[0,tsim],args=(self.rp1,self.m0,self.delta))
            #self.currentState = sol[1]
            #print sol
            self.currentState=self.currentState+grace_dynam(x,self.rp1,self.m0,self.delta)*tsim
            self.currentState[2,0]=max(self.currentState[2,0],0)
            if np.isnan(grace_dynam(x,self.rp1,self.m0,self.delta)).any() or np.isnan(self.currentState).any(): 
                print 'state: ',self.currentState[:,0]
                print 'dx:',self.currentState+grace_dynam(x,self.rp1,self.m0,self.delta)
                self.simulating=False
                self.Regulating=False
            sleep(tsim)
            i+=1
        print tsim*i
            
            
def grace_dynam(x,rp1,m0,delta):
    x_pos=x[0]
    y_pos=x[1]
    z_pos=x[2]
    v1=x[3]
    v2=x[4]
    v3=x[5]
    omega1=x[6]
    omega2=x[7]
    omega3=x[8]
    r11=x[9]
    r12=x[10]
    r13=x[11]
    r21=x[12]
    r22=x[13]
    r23=x[14]
    r31=x[15]
    r32=x[16]
    r33=x[17]
    V=np.sqrt(v1 ** 2 + v2 ** 2 + v3 ** 2)
    alpha=np.arctan2(v3 ,v1)
    beta=np.arcsin(v2 / V)
    R=np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])
    v_b=np.array([[v1],[v2],[v3]])
    omega_b=np.array([[omega1],[omega2],[omega3]])
    omega_b_hat=np.array([[0,- omega3,omega2],[omega3,0,- omega1],[- omega2,omega1,0]])
    cvx=0
    cvy=0
    cvz=0
    rp=np.array([[rp1],[0],[0]])
    g=9.8
    k=np.array([[0],[0],[1]])
    m1=3.88
    m2=9.9
    m3=5.32
    m_bar=0.8
    C_D0=0.45
    C_alpha_D=17.59
    C_delta_D=1
    C_beta_FS=- 2
    C_delta_FS=1.5
    C_L0=0.075
    C_alpha_L=19.58
    J1=0.8
    J2=0.05
    J3=0.08
    C_M0=0.0076
    C_beta_MR=- 0.3
    C_alpha_MP=0.57
    C_beta_MY=5
    C_delta_MY=- 0.2
    K_q1=- 0.1
    K_q2=- 0.5
    K_q3=- 0.1
    S=0.012
    m_w=3.7
    r_w3=0.0015
    rho=1000
    #print "z =",z_pos,", alpha = ", alpha
    #print 'lin vel:',v1,v2,v3,' ang vel:',omega_b
    M=np.diag([m1,m2,m3])
    J=np.diag([J1,J2,J3])
    r_w=np.array([[0],[0],[r_w3]])
    D=0.5 * rho * V ** 2 * S * (C_D0 + C_alpha_D * alpha ** 2 + C_delta_D * delta ** 2)
    FS=0#0.5 * rho * V ** 2 * S * (C_beta_FS * beta + C_delta_FS * delta)
    L=0.5 * rho * V ** 2 * S * (C_L0 + C_alpha_L * alpha)
    M1=0.5 * rho * V ** 2 * S * (C_beta_MR * beta + K_q1 * omega1)
    M2=0.5 * rho * V ** 2 * S * (C_M0 + C_alpha_MP * alpha + K_q2 * omega2)
    M3=0.5 * rho * V ** 2 * S * (C_beta_MY * beta + K_q3 * omega3 + C_delta_MY * delta)
    R_bv=np.array([(cos(alpha)*cos(beta),-cos(alpha)*sin(beta),-sin(alpha)),(sin(beta),cos(beta),0),(sin(alpha)*cos(beta),-sin(alpha)*sin(beta),cos(beta))])	
    F_ext = np.matmul(R_bv, np.asmatrix(np.array([(-D),(FS),(L)])).transpose())
    T_ext=np.matmul(R_bv,np.asmatrix(np.array([(M1),(M2),(M3)])).transpose())
    b_i_dot=np.matmul(R,v_b)
    R_dot=np.matmul(R,omega_b_hat)
    v_b_dot=np.matmul(np.linalg.inv(M),cross(np.matmul(M,v_b).transpose(), omega_b.transpose()).transpose() + m0*g*np.matmul(R.transpose(),k) + F_ext)
    #omega_b_dot=np.linalg.solve(J,(cross(np.matmul(J,omega_b).transpose(),omega_b.transpose()).transpose() + cross(np.matmul(M,v_b).transpose(),v_b.transpose()).transpose() + T_ext + cross(m_w * g * r_w.transpose(),np.matmul(R.transpose(),k).transpose()).transpose() + cross(m_bar * g * rp.transpose(),np.matmul(R.transpose(),k).transpose()).transpose()))
    omega_b_dot=np.matmul(np.linalg.inv(J),(cross(np.matmul(J,omega_b).transpose(),omega_b.transpose()).transpose() + cross(np.matmul(M,v_b).transpose(),v_b.transpose()).transpose() + T_ext + cross(m_w * g * r_w.transpose(),np.matmul(R.transpose(),k).transpose()).transpose() + cross(m_bar * g * rp.transpose(),np.matmul(R.transpose(),k).transpose()).transpose()))
    Rflat = R_dot.flatten()
    Rflat.shape=(9,1)
    dx=np.concatenate((b_i_dot,v_b_dot,omega_b_dot,Rflat))
    #print dx
    return dx

def grace_dynam2(x,t,rp1,m0,delta):
    x_pos=x[0]
    y_pos=x[1]
    z_pos=x[2]
    v1=x[3]
    v2=x[4]
    v3=x[5]
    omega1=x[6]
    omega2=x[7]
    omega3=x[8]
    r11=x[9]
    r12=x[10]
    r13=x[11]
    r21=x[12]
    r22=x[13]
    r23=x[14]
    r31=x[15]
    r32=x[16]
    r33=x[17]
    V=np.sqrt(v1 ** 2 + v2 ** 2 + v3 ** 2)
    alpha=np.arctan2(v3 ,v1)
    beta=np.arcsin(v2 / V)
    R=np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])
    v_b=np.array([[v1],[v2],[v3]])
    omega_b=np.array([[omega1],[omega2],[omega3]])
    omega_b_hat=np.array([[0,- omega3,omega2],[omega3,0,- omega1],[- omega2,omega1,0]])
    cvx=0
    cvy=0
    cvz=0
    rp=np.array([[rp1],[0],[0]])
    g=9.8
    k=np.array([[0],[0],[1]])
    m1=3.88
    m2=9.9
    m3=5.32
    m_bar=0.8
    C_D0=0.45
    C_alpha_D=17.59
    C_delta_D=1
    C_beta_FS=- 2
    C_delta_FS=1.5
    C_L0=0.075
    C_alpha_L=19.58
    J1=0.8
    J2=0.05
    J3=0.08
    C_M0=0.0076
    C_beta_MR=- 0.3
    C_alpha_MP=0.57
    C_beta_MY=5
    C_delta_MY=- 0.2
    K_q1=- 0.1
    K_q2=- 0.5
    K_q3=- 0.1
    S=0.012
    m_w=3.7
    r_w3=0.0015
    rho=1000
    #print "z =",z_pos,", alpha = ", alpha
    #print 'lin vel:',v1,v2,v3,' ang vel:',omega_b
    M=np.diag([m1,m2,m3])
    J=np.diag([J1,J2,J3])
    r_w=np.array([[0],[0],[r_w3]])
    D=0.5 * rho * V ** 2 * S * (C_D0 + C_alpha_D * alpha ** 2 + C_delta_D * delta ** 2)
    FS=0#0.5 * rho * V ** 2 * S * (C_beta_FS * beta + C_delta_FS * delta)
    L=0.5 * rho * V ** 2 * S * (C_L0 + C_alpha_L * alpha)
    M1=0.5 * rho * V ** 2 * S * (C_beta_MR * beta + K_q1 * omega1)
    M2=0.5 * rho * V ** 2 * S * (C_M0 + C_alpha_MP * alpha + K_q2 * omega2)
    M3=0.5 * rho * V ** 2 * S * (C_beta_MY * beta + K_q3 * omega3 + C_delta_MY * delta)
    R_bv=np.array([(cos(alpha)*cos(beta),-cos(alpha)*sin(beta),-sin(alpha)),(sin(beta),cos(beta),0),(sin(alpha)*cos(beta),-sin(alpha)*sin(beta),cos(beta))])	
    F_ext = np.matmul(R_bv, np.asmatrix(np.array([(-D),(FS),(L)])).transpose())
    T_ext=np.matmul(R_bv,np.asmatrix(np.array([(M1),(M2),(M3)])).transpose())
    b_i_dot=np.matmul(R,v_b)
    R_dot=np.matmul(R,omega_b_hat)
    v_b_dot=np.matmul(np.linalg.inv(M),cross(np.matmul(M,v_b).transpose(), omega_b.transpose()).transpose() + m0*g*np.matmul(R.transpose(),k) + F_ext)
    #omega_b_dot=np.linalg.solve(J,(cross(np.matmul(J,omega_b).transpose(),omega_b.transpose()).transpose() + cross(np.matmul(M,v_b).transpose(),v_b.transpose()).transpose() + T_ext + cross(m_w * g * r_w.transpose(),np.matmul(R.transpose(),k).transpose()).transpose() + cross(m_bar * g * rp.transpose(),np.matmul(R.transpose(),k).transpose()).transpose()))
    omega_b_dot=np.matmul(np.linalg.inv(J),(cross(np.matmul(J,omega_b).transpose(),omega_b.transpose()).transpose() + cross(np.matmul(M,v_b).transpose(),v_b.transpose()).transpose() + T_ext + cross(m_w * g * r_w.transpose(),np.matmul(R.transpose(),k).transpose()).transpose() + cross(m_bar * g * rp.transpose(),np.matmul(R.transpose(),k).transpose()).transpose()))
    Rflat = R_dot.flatten()
    Rflat.shape=(9,1)
    dx1=np.concatenate((b_i_dot,v_b_dot,omega_b_dot,Rflat))
    dx = []
    for element in dx1:
        dx.append(element[0,0])
    return dx





