import os, sys
TOOLS = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
from checklib import check
P={'U pocket (11.6, 9.1)':(11.58,9.65,None),'between L and T (5.0, 9.6)':(5.0,9.6,True),'middle (7.0, 8.0)':(7.0,8.0,True),
   'X centre gaps (11.4, 3.66)':(11.4,3.66,None),'diamond to wall (8.4, 0.3)':(8.4,0.3,None)}
labels=[(3.9,9.6,'L'),(8.8,10.4,'T'),(12.75,9.5,'U pocket'),(7.7,6.5,'S'),(12.3,3.9,'X'),(8.0,0.15,'diamond')]
check(os.path.join(BASE, 'open_test', 'open_test.world'),(1.5,6.0),P,'open_test.world (14 x 12 m)',labels=labels,png=os.path.join(BASE, 'open_test', 'open_test_layout.png'))
