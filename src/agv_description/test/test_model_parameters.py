import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


XACRO = Path(__file__).parents[1] / 'urdf' / 'agv.urdf.xacro'


class ModelParametersTest(unittest.TestCase):
    def test_heavy_body_and_passive_steering_parameters(self):
        text = XACRO.read_text()
        self.assertIn('<mass value="1163.5"/>', text)
        self.assertIn('ixx="73.94"', text)
        self.assertIn('iyy="151.52"', text)
        self.assertIn('izz="201.67"', text)
        self.assertIn('damping="1.0" friction="0.05"', text)
        self.assertEqual(
            len(re.findall(r'<xacro:module id="[1-4]"', text)),
            4)

    def test_wheel_inertia_matches_its_cylinder_axis(self):
        root = ET.parse(XACRO).getroot()
        ns = {'xacro': 'http://www.ros.org/wiki/xacro'}
        properties = {e.attrib['name']: float(e.attrib['value'])
                      for e in root.findall('xacro:property', ns)}
        wheel = root.find("xacro:macro[@name='wheel']", ns)
        inertia = wheel.find('link/inertial/inertia').attrib
        mass = float(wheel.find('link/inertial/mass').attrib['value'])
        radius, width = properties['wheel_radius'], properties['wheel_width']
        self.assertAlmostEqual(float(inertia['izz']), mass * radius ** 2 / 2)
        transverse = mass * (3 * radius ** 2 + width ** 2) / 12
        self.assertAlmostEqual(float(inertia['ixx']), transverse)
        self.assertAlmostEqual(float(inertia['iyy']), transverse)
        self.assertEqual(wheel.find('joint/axis').attrib['xyz'], '0 0 1')
        self.assertIsNone(wheel.find('link/collision/origin'))

    def test_chassis_collision_clears_steering_modules(self):
        text = XACRO.read_text()
        self.assertIn(
            '<collision><origin xyz="0 0 0.05"/>'
            '<geometry><box size="1.2 0.8 0.25"/></geometry></collision>',
            text)


if __name__ == '__main__':
    unittest.main()
