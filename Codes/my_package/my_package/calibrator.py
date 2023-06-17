import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32
import math
import time

class Calibrator(Node):
    def __init__(self):
        super().__init__('calibrator')
        self.gps = None
        self.cm = None
        self.direction = 0.0
        self.last_turn_time = None
        self.turning_time = 0
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_turn = self.create_publisher(Float32, 'turn', 10)
        self.subscription_gps = self.create_subscription(
            NavSatFix,
            'gps',
            self.listener_callback_gps,
            10)
        self.subscription_cm = self.create_subscription(
            Point,
            'center_of_mass_cm',
            self.listener_callback_cm,
            10)
        self.timer = self.create_timer(0.1, self.compare_values)

    def listener_callback_gps(self, msg):
        self.gps = msg

    def listener_callback_cm(self, msg):
        self.cm = msg

    def calculate_distance(self):
        # Calculate Euclidean distance
        return math.sqrt((self.cm.x - self.gps.longitude)**2 + (self.cm.y - self.gps.latitude)**2)

    def calculate_angle(self):
        # Calculate angle (in radians) for turning
        return math.atan2(self.cm.x - self.gps.longitude, self.cm.y - self.gps.latitude)

    def compare_values(self):
        if self.gps is not None and self.cm is not None:
            distance = self.calculate_distance()
            angle = self.calculate_angle() / math.pi * 180

            self.get_logger().info(f'Distance to target: {distance}, Angle to target: {angle}')

            twist = Twist()
            doruk = Float32()
            # If distance is greater than threshold, move robot
            if distance > 0.1:
                # rotate to face the target
                if abs(self.direction - angle) > 5:
                    self.last_turn_time = self.last_turn_time or time.time()
                    if angle > 0:
                        # turn right (clockwise)
                        twist.angular.z = 0.1
                        doruk.data = 40.125
                    else:
                        # turn left (counterclockwise)
                        twist.angular.z = -0.1
                        doruk.data = 30.125
                    # move forward with constant speed, only if angle is small enough (less than 10 degrees here)
                else:
                    self.last_turn_time = None
                    twist.linear.x = 0.1
                    doruk.data = 10.125
            else:
                # stop if target is close enough
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                doruk.data = 20.125

            if self.last_turn_time:
                now = time.time()
                self.turning_time += now - self.last_turn_time
                self.last_turn_time = now

            self.direction += (twist.angular.z * self.turning_time) / math.pi * 180 / 30
            self.get_logger().info(f'Current direction: {self.direction}')

            self.publisher_.publish(twist)
            self.publisher_turn.publish(doruk)


            if self.direction > 360:
                self.direction -= 360


def main(args=None):
    rclpy.init(args=args)
    calibrator = Calibrator()
    rclpy.spin(calibrator)

    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
