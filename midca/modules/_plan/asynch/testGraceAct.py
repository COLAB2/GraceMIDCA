import GraceAct as G
from time import sleep
g=G.GraceMidcaAct()
g.gotToDepth(-5)
sleep(5)
g.stopRegulation()
sleep(5)
print 'done'
