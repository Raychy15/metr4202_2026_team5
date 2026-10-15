import os, sys
TOOLS = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, TOOLS)
BASE = os.path.dirname(TOOLS)
from checklib import check
U=1.03
def c(i): return 0.53+U*i
P={'T slalom low (6.71, 8.77)':(c(6),c(8),True),'T slalom high (8.77, 9.8)':(c(8),c(9),True),
   'central door (7.63, 7.74)':(c(7)-0.11,c(7),True),'central room (6.2, 3.4)':(6.2,3.4,True),
   'B squeeze (6.71, 0.43)':(c(6),0.43,True),'B slalom (8.77, 1.56)':(c(8),c(1),True),
   'fence gap 0.85 (11.79, 5.68)':(11.785,c(5),True),'fence gap 0.21 (13.32, 5.68)':(13.315,c(5),False),
   'lower E (12.0, 4.4)':(12.0,4.4,True),'pocket (13.9, 0.5)':(13.9,0.5,False),'pocket slot (14.25, 1.56)':(14.245,c(1),False)}
labels=[(0.15,10.0,'W room'),(4.3,9.95,'T slalom'),(5.2,6.4,'central room\n(dead end, 0.84 m door)'),(4.4,0.15,'B slalom'),
        (10.5,10.0,'E room'),(10.4,6.3,'fence 0.85 | 0.21'),(12.5,0.2,'pocket\n0.41 slot')]
check(os.path.join(BASE,'ring_mix','ring_mix.world'),(1.0,1.0),P,'ring_mix.world (14.5 x 10.3 m)',labels=labels,png=os.path.join(BASE,'ring_mix','ring_mix_layout.png'))
