import GraceAct as G
from time import sleep
import numpy as np
#print G.grace_dynam(np.array([0,0,0,.1,.03,0,0,0,0,1,0,0,0,1,0,0,0,1]),0,0,0)
#print G.grace_dynam2(np.array([0,0,0,.1,.03,0,0,0,0,1,0,0,0,1,0,0,0,1]),0,0,0,0)

g=G.GraceMidcaAct()#this is the same class we use in the midca code
g.startSimulation()# i added a simulation thread that will simulate the robot's movement
#call it with this command /|\ and from there we can use the code as usual
target = 5
g.gotToDepth(target)
while g.senseDepth()<target:
    print g.estimateDepthRate(dt=0.5)
target = 3
g.stopRegulation()
print 'Depth: ',g.senseDepth()
sleep(5)
g.gotToDepth(target)
while g.senseDepth()>target:
    pass
print 'Depth: ',g.senseDepth()
sleep(60)
g.stopRegulation()
print 'Depth: ',g.senseDepth()
sleep(5)
g.stopSimulation()
print 'done:',g.senseDepth()

