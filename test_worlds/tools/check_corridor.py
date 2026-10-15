import os, sys
TOOLS = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
from checklib import check
U=1.03
def c(i): return 0.53+U*i
P={'C corridor before choke (5.0, 2.5)':(5.0,2.5,True),'C choke (6.7, 2.05)':(6.7,2.05,False),
   'C beyond choke (8.6, 2.5)':(8.6,2.5,True),'D dead end (6.0, 0.43)':(6.0,0.43,True),
   'door S->H (2.69, 3.62)':(2.69,3.62,True),'door H->M (4.65, 6.69)':(4.65,6.69,True),
   'zig-zag M (6.71, 4.65)':(6.71,4.65,True),'door M->E (9.8, 5.68)':(9.8,5.68,True),
   'E room (12.3, 6.0)':(12.3,6.0,True)}
labels=[(0.2,0.2,'S start room'),(0.2,7.6,'H hall (cylinder)'),(4.4,2.95,'C: 1.26 m corridor'),(7.3,2.1,'<- 0.38 m choke'),
        (4.6,0.15,'D: 0.86 m dead end (5 m)'),(7.05,5.85,'M zig-zag'),(10.4,7.6,'E room'),(2.0,3.95,'0.86 door'),(3.6,7.0,'0.86 door')]
check(os.path.join(BASE, 'corridor_trap', 'corridor_trap.world'),(1.2,1.2),P,'corridor_trap.world (13.4 x 8.3 m)',labels=labels,png=os.path.join(BASE, 'corridor_trap', 'corridor_trap_layout.png'))
