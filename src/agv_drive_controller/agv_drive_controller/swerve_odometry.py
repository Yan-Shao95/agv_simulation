import math
from .nodes import Node, JointState, rclpy
from .forward_kinematics import estimate_twist
try:
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
except ImportError:  # pragma: no cover
    Odometry=TransformStamped=TransformBroadcaster=None

POSITIONS=((0.45,0.325),(0.45,-0.325),(-0.45,-0.325),(-0.45,0.325))
class SwerveOdometry(Node):
    def __init__(self):
        super().__init__('swerve_odometry'); self.x=self.y=self.yaw=0.0; self.last=None
        self.publish_tf=self.declare_parameter('publish_odom_tf',True).value
        self.pub=self.create_publisher(Odometry,'odometry/wheel',10); self.tf=TransformBroadcaster(self)
        self.create_subscription(JointState,'joint_states',self._update,10)
    def _update(self,msg):
        pos=dict(zip(msg.name,msg.position)); vel=dict(zip(msg.name,msg.velocity))
        angles=[pos.get(f'module_{i}_steering_joint',0.0) for i in range(1,5)]
        speeds=[]
        for i in range(1,5):
            left=vel.get(f'module_{i}_left_wheel_joint',0.0)*0.1
            right=-vel.get(f'module_{i}_right_wheel_joint',0.0)*0.1
            speeds.append((left+right)/2.0)
        twist=estimate_twist(angles,speeds,POSITIONS); now=self.get_clock().now()
        if self.last is not None:
            dt=(now-self.last).nanoseconds/1e9; c=math.cos(self.yaw); s=math.sin(self.yaw)
            self.x+=(c*twist.vx-s*twist.vy)*dt; self.y+=(s*twist.vx+c*twist.vy)*dt; self.yaw+=twist.wz*dt
        self.last=now; qz=math.sin(self.yaw/2); qw=math.cos(self.yaw/2)
        out=Odometry(); out.header.stamp=now.to_msg(); out.header.frame_id='odom'; out.child_frame_id='base_link'; out.pose.pose.position.x=self.x; out.pose.pose.position.y=self.y; out.pose.pose.orientation.z=qz; out.pose.pose.orientation.w=qw; out.twist.twist.linear.x=twist.vx; out.twist.twist.linear.y=twist.vy; out.twist.twist.angular.z=twist.wz; self.pub.publish(out)
        if self.publish_tf:
            tf=TransformStamped(); tf.header=out.header; tf.child_frame_id='base_link'; tf.transform.translation.x=self.x; tf.transform.translation.y=self.y; tf.transform.rotation=out.pose.pose.orientation; self.tf.sendTransform(tf)
def main():
    if rclpy is None: raise RuntimeError('需要 ROS 2 Jazzy 环境')
    rclpy.init(); node=SwerveOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
