import math, sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
from agv_drive_controller.forward_kinematics import estimate_twist

class ForwardKinematicsTest(unittest.TestCase):
    def test_recovers_horizontal_velocity(self):
        positions=((0.45,0.325),(0.45,-0.325),(-0.45,-0.325),(-0.45,0.325))
        result=estimate_twist([math.pi/2]*4,[1.0]*4,positions)
        self.assertAlmostEqual(result.vx,0.0,places=8)
        self.assertAlmostEqual(result.vy,1.0,places=8)
        self.assertAlmostEqual(result.wz,0.0,places=8)
if __name__=='__main__': unittest.main()
