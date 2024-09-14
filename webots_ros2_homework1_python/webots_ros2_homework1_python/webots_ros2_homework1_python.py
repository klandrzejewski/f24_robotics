import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import time

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90
ALPHA = 0.5  # Weight for utility function (balance between distance and information gain)
TARGET_REACHED_THRESHOLD = 0.2  # Distance threshold to consider target reached
TURNING_SPEED = 0.3  # Angular speed when turning toward a target
MIN_TARGET_DISTANCE = 1.0  # Minimum distance to consider a target
STALL_TIME_THRESHOLD = 5  # Time in seconds before detecting a stall

class WallWalker(Node):

    def __init__(self):
        super().__init__('wall_walker_node')
        self.scan_cleaned = []
        self.target_location = None
        self.stall = False
        self.time_stationary = 0.0  # Time spent stationary
        self.last_move_time = time.time()  # Record the last move time
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        self.candidate_locations = []  # Store candidate locations (frontiers)
        self.current_pos = None  # To store current position
        self.pose_saved = None  # Save last position for stall detection
        self.cmd = Twist()
        self.timer = self.create_timer(0.5, self.timer_callback)

    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = []
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)
        self.identify_frontiers()  # Detect frontiers after lidar scan

    def listener_callback2(self, msg2):
        self.current_pos = msg2.pose.pose.position
        if self.pose_saved is not None:
            self.get_logger().info(f'X: {self.current_pos.x} Y: {self.current_pos.y}')
            diffX = math.fabs(self.pose_saved.x - self.current_pos.x)
            diffY = math.fabs(self.pose_saved.y - self.current_pos.y)
            
            # If robot hasn't moved significantly, update the stationary time
            if diffX < 0.001 and diffY < 0.001:
                current_time = time.time()
                #self.get_logger().info(f'Current Time: {current_time}')
                #self.get_logger().info(f'Last Move Time: {self.last_move_time}')
                self.time_stationary = current_time - self.last_move_time
                #self.last_move_time = current_time
            else:
                # Reset stall timer if the robot has moved
                self.time_stationary = 0.0
                self.last_move_time = time.time()

        self.pose_saved = self.current_pos  # Save current position for next comparison

    def identify_frontiers(self):
        self.candidate_locations = []  # Reset frontiers
        for i in range(len(self.scan_cleaned)):
            if self.scan_cleaned[i] > SAFE_STOP_DISTANCE:  # Identify frontiers based on safe distance
                angle = (i * math.pi / 180.0)  # Convert index to angle
                x = self.current_pos.x + self.scan_cleaned[i] * math.cos(angle)
                y = self.current_pos.y + self.scan_cleaned[i] * math.sin(angle)
                self.candidate_locations.append((x, y))  # Save as a candidate frontier location

    def evaluate_candidates(self):
        best_candidate = None
        best_utility = -float('Inf')
        
        for candidate in self.candidate_locations:
            dist = self.euclidean_distance(self.current_pos, candidate)
            
            # Skip candidates that are too close
            if dist < MIN_TARGET_DISTANCE:
                continue
            
            info_gain = self.estimate_information_gain(candidate)
            utility = ALPHA * (1 - dist / self.max_distance()) + (1 - ALPHA) * info_gain
            
            if utility > best_utility:
                best_utility = utility
                best_candidate = candidate
                
        return best_candidate


    def max_distance(self):
        return max([self.euclidean_distance(self.current_pos, c) for c in self.candidate_locations], default=1)

    def estimate_information_gain(self, candidate):
        return 1.0  # Placeholder for real information gain calculation

    def euclidean_distance(self, p1, p2):
        return math.sqrt((p1.x - p2[0])**2 + (p1.y - p2[1])**2)

    def move_to_target(self, target):
        angle_to_target = math.atan2(target[1] - self.current_pos.y, target[0] - self.current_pos.x)
        distance = self.euclidean_distance(self.current_pos, target)
        
        if distance < TARGET_REACHED_THRESHOLD:
            # Target reached, stop the robot
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            return True  # Indicate that target is reached

        # Linear movement towards target
        self.cmd.linear.x = min(distance, LINEAR_VEL)
        
        # Angular control to face the target
        self.cmd.angular.z = TURNING_SPEED * angle_to_target
        self.publisher_.publish(self.cmd)
        return False  # Target not reached yet

    def timer_callback(self):
        if len(self.scan_cleaned) == 0 or self.current_pos is None:
            self.turtlebot_moving = False
            return
        
        # Get the minimum distances from the LIDAR on the right, front, and left
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
        
        # Wall-following logic
        self.get_logger().info(f'Time stationary: {self.time_stationary}')

        if self.time_stationary >= STALL_TIME_THRESHOLD:
            self.cmd.linear.x = -0.5  # Reverse to recover from stall
            self.cmd.angular.z = 0.5  # Rotate to find a new path
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Stalled, recovering')
            self.time_stationary = 0.0
            self.last_move_time = time.time()
            self.stall = False  # Reset stall flag
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            # If there's an obstacle in front, slow down and turn
            self.cmd.linear.x = 0.07
            if right_lidar_min > left_lidar_min:
                self.cmd.angular.z = -0.3  # Turn right
            else:
                self.cmd.angular.z = 0.3  # Turn left
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Turning to avoid front obstacle')
            self.turtlebot_moving = True
        else:
            # If there's space in front, follow the wall on the right side
            if right_lidar_min < SAFE_STOP_DISTANCE:
                # If the robot is too close to the right wall, turn left slightly
                self.cmd.linear.x = 0.15
                self.cmd.angular.z = 0.1
                self.get_logger().info('Too close to wall, adjusting left')
            elif right_lidar_min > SAFE_STOP_DISTANCE + 0.2:
                # If the robot is too far from the right wall, turn right slightly
                self.cmd.linear.x = 0.15
                self.cmd.angular.z = -0.1
                self.get_logger().info('Too far from wall, adjusting right')
            else:
                # If the distance is optimal, move forward
                self.cmd.linear.x = LINEAR_VEL
                self.cmd.angular.z = 0.0
                self.get_logger().info('Following wall')

            # Publish the movement command
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True


def main(args=None):
    rclpy.init(args=args)
    room_explorer_node = WallWalker()
    rclpy.spin(room_explorer_node)
    room_explorer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()