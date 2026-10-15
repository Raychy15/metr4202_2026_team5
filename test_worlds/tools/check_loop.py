import os, sys
TOOLS = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
from checklib import check
U=1.03
def c(i): return 0.53+U*i
P={'door BL->BM (4.65, 1.65)':(c(4),1.65,True),'door BM->BR (8.77, 0.53)':(c(8),c(0),True),
   'right end (10.3, 4.65)':(10.3,c(4),True),'door TR->TL (6.71, 7.68)':(c(6),7.68,True),
   'left end (2.1, 4.65)':(2.1,c(4),True),'TL (3.5, 8.6)':(3.5,8.6,True),'BM (6.0, 3.2)':(6.0,3.2,True)}
labels=[(0.2,0.2,'BL start'),(5.3,0.2,'BM'),(9.4,0.2,'BR'),(9.0,8.9,'TR'),(0.2,8.9,'TL'),(3.3,4.4,'central wall (1 m thick)'),
        (0.2,4.4,'<- loop ->'),(11.2,4.0,'loop')]
check(os.path.join(BASE, 'loop_test', 'loop_test.world'),(1.2,1.2),P,'loop_test.world (12.4 x 9.3 m)',labels=labels,png=os.path.join(BASE, 'loop_test', 'loop_test_layout.png'))
