import os, sys
TOOLS = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, TOOLS)
BASE = os.path.dirname(TOOLS)
from checklib import check
U=1.03
def c(i): return 0.53+U*i
P={'hub (6.71, 5.68)':(c(6),c(5),True),'N exit':(c(6),c(7),True),'S exit':(c(6),c(3),True),'W exit':(c(4),c(5),True),'E exit':(c(8),c(5),True),
   'S corridor (4.5, 1.56)':(4.5,c(1),True),'S choke (2.59, 1.25)':(c(2),1.24,False),'SW room (0.8, 1.0)':(0.8,1.0,True),
   'W dead end top (1.56, 11.6)':(c(1),11.6,True),'W zig-zag (1.56, 4.65)':(c(1),c(4),True),
   'fence gap 0.38 (10.52, 3.62)':(10.52,c(3),False),'fence gap 0.68 (12.05, 3.62)':(12.05,c(3),True),'lower E (12.6, 1.0)':(12.6,1.0,True),
   'NE squeeze (11.8, 10.4)':(11.75,c(10),True),'N hall (6.7, 9.0)':(6.7,9.0,True)}
labels=[(5.6,6.4,'hub'),(4.6,11.9,'N hall: sphere, cylinder, tilted box'),(0.1,12.0,'W dead end'),(0.05,2.8,'SW room'),(2.1,0.6,'choke 0.40 m'),
        (3.6,1.85,'S corridor'),(9.4,6.3,'E room'),(9.3,3.0,'fence 0.38 | 0.68'),(9.6,0.1,'lower E'),(11.0,11.9,'NE strip')]
check(os.path.join(BASE,'hub_mix','hub_mix.world'),(c(6),c(5)),P,'hub_mix.world (13.4 x 12.4 m)',labels=labels,png=os.path.join(BASE,'hub_mix','hub_mix_layout.png'))
