

class GraceMidcaPercieve():
	'''
	class that provides high level perception of Grace data for midca example
	'''
	def __init__(self):#initiallizes socket connection library to communicate with and control grace	
            import sys
            gracePath="/home/pi/Desktop/Grace_Control"
            sys.path.insert(0, gracePath)
            import GliderFunIPC
            self.interface = GliderFunIPC.GraceFun()
            self.bottomDepth = -1
            self.gracePath = gracePath
		
	def senseDepth(self): #reads the pressure sensor and converts it to depth in meters
		grace = self.interface
		return grace.readDepth()
		
	def beginBottomCheck(self): #will call the function to check at bottom
		import os
		os.system("./findBottom &")
		
	def checkAtBottom(self): #read a file output by program chechinkg for bottom and return true or false
		atBottom = False
		gracePath=""
		f=open(gracePath+"atBottom",'r')
		atBottom= (1==int(f.readline()))
		f.close()
		if atBottom:
                    f=open(gracePath+"atBottom",'w')
                    f.write("0")
                    f.close()
                    grace = self.interface
		    self.bottomDepth=grace.readDepth()
		return atBottom 
		
	def checkCommunicationAck(self): #read a file output by program chechinkg for surface and return true or false
		                  #sending $%GO%$ over xbee will cause a file "Next_Dive_GO" to be produce with 1 in line one
		Acknowleged = False
		gracePath = self.gracePath+"/"
		f=open(gracePath+"Next_Dive_GO",'r')
		Acknowleged= (1==int(f.readline()))
		f.close()
		if Acknowleged:
                    f=open(gracePath+"Next_Dive_GO",'w')
                    f.write("0")
                    f.close()
		return Acknowleged  
		
	def checkAtSurfaceDepth(self): #chechs if depth is close enough to say we are surfaced
		grace = self.interface
		depth = grace.readDepth()
		return depth <= .2    
		      
