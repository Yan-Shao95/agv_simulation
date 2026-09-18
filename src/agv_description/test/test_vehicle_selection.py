import unittest
from pathlib import Path
import importlib.util


ROOT = Path(__file__).parents[2]


class VehicleSelectionTest(unittest.TestCase):
    def test_description_maps_exactly_three_vehicle_types(self):
        text = (ROOT / 'agv_description/launch/description.launch.py').read_text()
        self.assertIn("'differential_swerve': 'agv.urdf.xacro'", text)
        self.assertIn("'mecanum': 'mecanum.urdf.xacro'", text)
        self.assertIn("'ackermann': 'ackermann.urdf.xacro'", text)
        self.assertIn('未知 vehicle_type=', text)

    def test_sim_launch_passes_vehicle_type_and_conditions_swerve_chain(self):
        text = (ROOT / 'agv_bringup/launch/sim.launch.py').read_text()
        self.assertIn("DeclareLaunchArgument(\n            'vehicle_type'", text)
        self.assertIn("{'vehicle_type': vehicle_type}", text)
        self.assertIn("condition=is_differential", text)

    def test_sim_launch_starts_rviz_with_laser_scan_config_by_default(self):
        launch = (ROOT / 'agv_bringup/launch/sim.launch.py').read_text()
        config = ROOT / 'agv_bringup/config/simulation.rviz'
        self.assertIn("'use_rviz', default_value='true'", launch)
        self.assertIn("package='rviz2'", launch)
        self.assertIn("'config', 'simulation.rviz'", launch)
        self.assertTrue(config.exists())
        text = config.read_text()
        self.assertIn('Fixed Frame: base_link', text)
        self.assertIn('Class: rviz_default_plugins/LaserScan', text)
        self.assertIn('Value: /scan', text)
        self.assertIn('Reliability Policy: Best Effort', text)
        bridge = (ROOT / 'agv_gazebo/config/bridge.yaml').read_text()
        self.assertIn('frame_id: laser_frame', bridge)

    def test_gazebo_launch_only_spawns_ros2_control_for_swerve(self):
        text = (ROOT / 'agv_gazebo/launch/simulation.launch.py').read_text()
        self.assertIn("{'vehicle_type': vehicle_type}", text)
        self.assertEqual(text.count('condition=is_differential'), 2)

    def test_mecanum_spawn_sdf_keeps_traction_axes_in_base_frame(self):
        launch_path = ROOT / 'agv_gazebo/launch/simulation.launch.py'
        spec = importlib.util.spec_from_file_location('agv_simulation_launch', launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        model_path = ROOT / 'agv_description/urdf/mecanum.urdf.xacro'
        sdf = module._mecanum_sdf_from_xacro(model_path)
        self.assertEqual(sdf.count("fdir1 gz:expressed_in='base_footprint'"), 4)
        self.assertNotIn('<fdir1>1 ', sdf)

    def test_mapping_and_navigation_forward_vehicle_type(self):
        for name in ('mapping.launch.py', 'navigation.launch.py'):
            text = (ROOT / 'agv_bringup/launch' / name).read_text()
            self.assertIn("'vehicle_type'", text)
            self.assertIn("'vehicle_type': vehicle_type", text)

    def test_navigation_uses_gazebo_map_and_amcl_localization(self):
        bringup = (ROOT / 'agv_bringup/launch/navigation.launch.py').read_text()
        navigation = (ROOT / 'agv_navigation/launch/navigation.launch.py').read_text()
        params = (ROOT / 'agv_navigation/config/nav2.yaml').read_text()
        rviz = (ROOT / 'agv_navigation/config/nav2_localization.rviz').read_text()
        map_yaml = (ROOT / 'agv_navigation/maps/warehouse.yaml').read_text()
        self.assertIn("'use_rviz': 'false'", bringup)
        self.assertIn("name='nav2_rviz'", bringup)
        self.assertIn("FindPackageShare('nav2_bringup')", navigation)
        self.assertIn("'bringup_launch.py'", navigation)
        self.assertIn("'use_localization': 'True'", navigation)
        self.assertIn("'use_composition': 'False'", navigation)
        self.assertIn("'/drive/set_control_mode'", bringup)
        self.assertIn("'{mode: 1}'", bringup)
        self.assertIn('controller_server:', params)
        self.assertIn('planner_server:', params)
        self.assertIn('bt_navigator:', params)
        self.assertIn('nav2_smac_planner::SmacPlanner2D', params)
        self.assertIn('motion_model: "Omni"', params)
        self.assertIn('consider_footprint: true', params)
        self.assertIn('VelocityDeadbandCritic', params)
        self.assertIn('cost_travel_multiplier: 6.0', params)
        self.assertIn('inflation_radius: 7.0', params)
        self.assertIn('robot_model_type: "nav2_amcl::OmniMotionModel"', params)
        self.assertIn('set_initial_pose: true', params)
        self.assertIn('Fixed Frame: map', rviz)
        self.assertIn('Class: rviz_default_plugins/Map', rviz)
        self.assertIn('Class: nav2_rviz_plugins/ParticleCloud', rviz)
        self.assertIn('resolution: 0.05', map_yaml)

    def test_mecanum_model_has_four_wheels_and_drive_plugin(self):
        path = ROOT / 'agv_description/urdf/mecanum.urdf.xacro'
        self.assertTrue(path.exists())
        text = path.read_text()
        for name in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            self.assertIn(f'name="{name}"', text)
        self.assertIn('gz-sim-mecanum-drive-system', text)

    def test_mecanum_wheels_follow_gazebo_contact_and_axis_convention(self):
        text = (ROOT / 'agv_description/urdf/mecanum.urdf.xacro').read_text()
        self.assertIn('<collision><geometry><sphere', text)
        self.assertIn('<axis xyz="0 1 0"/>', text)
        self.assertNotIn('<axis xyz="0 -1 0"/>', text)
        self.assertIn('params="name x y traction"', text)
        self.assertIn('${traction}</fdir1>', text)
        self.assertIn('<mu1>1.2</mu1><mu2>0.0</mu2>', text)

    def test_mecanum_model_and_bridge_publish_gps_measurements(self):
        model = (ROOT / 'agv_description/urdf/mecanum.urdf.xacro').read_text()
        bridge = (ROOT / 'agv_gazebo/config/bridge.yaml').read_text()
        self.assertIn('sensor name="gps" type="navsat"', model)
        self.assertIn('ros_topic_name: /sensors/gps/fix', bridge)
        self.assertIn('ros_type_name: sensor_msgs/msg/NavSatFix', bridge)
        self.assertIn('gz_type_name: gz.msgs.NavSat', bridge)

    def test_world_loads_navsat_system(self):
        world = (ROOT / 'agv_worlds/worlds/warehouse_40x50.sdf').read_text()
        self.assertIn('gz-sim-navsat-system', world)
        self.assertIn('<spherical_coordinates>', world)

    def test_ackermann_has_dedicated_stateful_keyboard_launcher(self):
        project = ROOT.parent
        script = project / 'scripts/ackermann_teleop.sh'
        self.assertTrue(script.exists())
        script_text = script.read_text()
        self.assertIn('agv_drive_controller ackermann_keyboard', script_text)
        readme = (project / 'README.md').read_text()
        self.assertIn('阿克曼状态保持式键盘控制', readme)
        self.assertIn('0.1 rad/s', readme)
        self.assertIn('0.15', readme)

    def test_ackermann_model_has_steering_joints_and_plugin(self):
        path = ROOT / 'agv_description/urdf/ackermann.urdf.xacro'
        self.assertTrue(path.exists())
        text = path.read_text()
        self.assertIn('front_${side}_steering_joint', text)
        self.assertIn('gz-sim-ackermann-steering-system', text)

    def test_bridge_supports_plugin_cmd_vel_and_odometry(self):
        text = (ROOT / 'agv_gazebo/config/bridge.yaml').read_text()
        self.assertIn('ros_topic_name: /cmd_vel', text)
        self.assertIn('gz_topic_name: /model/agv/cmd_vel', text)
        self.assertIn('ros_topic_name: /odometry/wheel', text)


if __name__ == '__main__':
    unittest.main()
