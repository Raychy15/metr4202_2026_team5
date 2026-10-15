import os, sys
TOOLS = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, TOOLS)
BASE = os.path.dirname(TOOLS)
from checklib import check
U=1.03
def c(i): return 0.53+U*i
P={'door A->S-bend (3.62, 1.43)':(c(3),1.43,True),'door A->P (1.45, 3.62)':(1.45,c(3),True),
   'S-bend leg 1 (4.65, 1.56)':(c(4),c(1),True),'choke (6.71, 1.22)':(c(6),1.22,False),
   'S-bend leg 2 (5.68, 3.0)':(c(5),3.0,True),'S-bend exit (6.71, 4.65)':(c(6),c(4),True),
   'door P->J (4.65, 7.74)':(c(4),c(7),True),'T alley (2.6, 10.8)':(c(2),c(10),True),
   'fence gap 0.30 (10.83, 1.18)':(c(10),1.18,False),'fence gap 0.85 (10.83, 2.75)':(c(10),2.755,True),
   'SE room (13.5, 3.5)':(13.5,3.5,True),'NE U pocket (12.4, 9.5)':(12.4,9.6,None),'middle (8.0, 6.0)':(8.0,6.0,True)}
labels=[(0.1,0.15,'A start'),(0.1,8.95,'P hall'),(1.1,10.95,'T dead end'),(3.7,0.15,'S-bend 0.8 m'),(6.0,0.6,'0.38\nchoke'),
        (7.2,10.95,'middle strip'),(11.2,10.95,'NE room'),(11.4,0.15,'SE room'),(9.4,5.0,'fence\n0.30 | 0.85')]
check(os.path.join(BASE,'tight_mix','tight_mix.world'),(1.0,1.0),P,'tight_mix.world (15.5 x 11.4 m)',labels=labels,png=os.path.join(BASE,'tight_mix','tight_mix_layout.png'))
