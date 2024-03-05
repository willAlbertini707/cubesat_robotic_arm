import numpy as np
from robot_structure import Link, FourLinkArm

link1 = Link("z", 30)
link2 = Link("z", 20)
link3 = Link("z", 5)
link4 = Link("x", 3, inline=True)

arm = FourLinkArm(link1, link2, link3, link4)

arm.animate_arm()