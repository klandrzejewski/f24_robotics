import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math


class TurtleBotController(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')
        
        # Publisher for controlling TurtleBot's velocity
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriptions for odometry and lidar
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # Initialize variables for odometry data
        self.initial_position = None
        self.current_position = None
        self.initial_orientation = None
        self.current_orientation = None

        self.distance_travelled = 0.0
        self.angle_turned = 0.0

        # Command to control velocity
        self.cmd = Twist()

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.update_callback)

        # Movement parameters
        self.target_distance = 1.0  # Default target distance (in meters)
        self.target_angle = 0.0  # Default target angle (in radians)
        self.speed = 0.075  # Default linear speed (m/s)
        self.angular_speed = 0.0  # Default angular speed (rad/s)

        self.moving_forward = False
        self.turning = False

    def odometry_callback(self, msg):
        # Store the current position and orientation from odometry data
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.current_position = (position.x, position.y)
        self.current_orientation = self.get_yaw_from_quaternion(orientation)

        if self.initial_position is None:
            self.initial_position = self.current_position
        if self.initial_orientation is None:
            self.initial_orientation = self.current_orientation

        self.distance_travelled = self.calculate_distance(self.initial_position, self.current_position)
        self.angle_turned = self.calculate_angle(self.initial_orientation, self.current_orientation)

    def get_yaw_from_quaternion(self, orientation):
        # Convert quaternion to Euler angles and extract yaw
        _, _, yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        return yaw

    def euler_from_quaternion(self, x, y, z, w):
        # Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def calculate_distance(self, initial_position, current_position):
        # Calculate Euclidean distance between initial and current positions
        return math.sqrt((current_position[0] - initial_position[0]) ** 2 + 
                         (current_position[1] - initial_position[1]) ** 2)

    def calculate_angle(self, initial_yaw, current_yaw):
        # Calculate the difference in orientation (yaw)
        return abs(current_yaw - initial_yaw)

    def update_callback(self):
        if self.moving_forward:
            # Check if we have reached the target distance
            if self.distance_travelled >= self.target_distance:
                self.cmd.linear.x = 0.0
                self.publisher_.publish(self.cmd)
                self.get_logger().info(f'Target distance reached: {self.distance_travelled} meters')
                self.moving_forward = False
            else:
                self.cmd.linear.x = self.speed
                self.publisher_.publish(self.cmd)

        elif self.turning:
            # Check if we have reached the target angle
            if self.angle_turned >= self.target_angle:
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                self.get_logger().info(f'Target angle reached: {self.angle_turned} radians')
                self.turning = False
            else:
                self.cmd.angular.z = self.angular_speed
                self.publisher_.publish(self.cmd)

    def move_straight(self, distance, speed):
        # Move the robot in a straight line
        self.target_distance = distance
        self.speed = speed
        self.initial_position = self.current_position
        self.moving_forward = True

    def turn_angle(self, angle, angular_speed):
        # Turn the robot by a certain angle
        self.target_angle = angle
        self.angular_speed = angular_speed
        self.initial_orientation = self.current_orientation
        self.turning = True


def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtleBotController()
    
    # Move straight for different distances at specified speeds
    turtlebot_controller.move_straight(1.0, 0.075)  # Example: Move 1 meter at 75 mm/s
    turtlebot_controller.move_straight(5.0, 0.150)  # Example: Move 5 meters at 150 mm/s

    # Turn by different angles at specified angular speeds
    turtlebot_controller.turn_angle(math.radians(10), math.radians(30))  # Example: Turn 10 degrees at 30 deg/s
    turtlebot_controller.turn_angle(math.radians(180), math.radians(120))  # Example: Turn 180 degrees at 120 deg/s

    rclpy.spin(turtlebot_controller)

    turtlebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
