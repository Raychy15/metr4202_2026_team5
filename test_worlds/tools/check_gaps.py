import os, sys
TOOLS = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
from checklib import check
P={'fence gap 0.30 (6.5, 1.18)':(6.5,1.18,False),'fence gap 0.60 (6.5, 2.63)':(6.5,2.63,None),
   'fence gap 0.85 (6.5, 4.36)':(6.5,4.36,True),'fence gap 0.40 (6.5, 5.98)':(6.5,5.98,False),
   'fence gap 0.79 (6.5, 7.58)':(6.5,7.58,True),
   'diamond squeeze (4.3, 0.4)':(4.3,0.4,None),'funnel mouth (8.7, 2.12)':(8.7,2.12,None),'funnel throat (9.6, 2.12)':(9.6,2.12,False),
   'funnel lower squeeze (9.4, 0.4)':(9.4,0.4,None),
   'slot (9.42, 7.9)':(9.42,7.9,True),'right half (10.5, 6.0)':(10.5,6.0,True)}
labels=[(3.2,0.15,'0.8 m squeeze'),(5.05,1.1,'0.30'),(4.6,2.55,'0.60 tight'),(5.05,4.25,'0.85'),(5.05,5.9,'0.40'),(5.05,7.5,'0.79'),
        (7.15,2.15,'funnel\n0.35 m throat'),(9.05,6.55,'0.9 m dead-end\nslot'),(0.2,8.55,'rotated boxes, sphere, cylinder'),(9.5,0.15,'0.77 m squeeze')]
check(os.path.join(BASE, 'gaps_test', 'gaps_test.world'),(1.0,1.0),P,'gaps_test.world (12 x 9 m)',labels=labels,png=os.path.join(BASE, 'gaps_test', 'gaps_test_layout.png'))
