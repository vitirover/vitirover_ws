# to run this script, use "python3 command_robot_gazebo.py"
# and send commands to gazebo in another terminal using "src/mobile_robot/scripts/command_robot_gazebo.py"

import rospy
import numpy as np
from math import tan, sin, cos, radians
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Robot dimensions
e = 0.340  # Track width (m)
r = 0.088  # Wheel radius (m)
x = 0.400  # Distance from the robot center to the wheel

# Initialize velocities
v = 0.0
omega = 0.0

def cmd_vel_callback(data):
    global v, omega
    v = data.linear.x
    omega = data.angular.z
    calculate_wheel_velocities()

def calculate_wheel_velocities():
    global angle_value_rad
    # Convert angle from degrees to radians for calculation
    angle_value_rad = radians(10)  # Example angle, replace with actual telemetry data

    # Matrix A calculation using the current back axle angle
    A = np.array([
        [(x + e * tan(angle_value_rad) / 2) / x * r, 0],
        [(x - e * tan(angle_value_rad) / 2) / x * r, 0],
        [(x + e * sin(angle_value_rad) / 2) / x * r * cos(angle_value_rad), -e / 2 * r],
        [(x - e * sin(angle_value_rad) / 2) / x * r * cos(angle_value_rad), e / 2 * r]
    ])

    # Calculate wheel velocities
    wRoues = np.dot(A, np.array([v, omega]))

    # Publish wheel velocities
    publish_wheel_velocities(wRoues[0], wRoues[1], wRoues[2], wRoues[3])

def publish_wheel_velocities(wG, wH, wI, wJ):
    # Create messages for each wheel velocity
    wheel_msgs = [Float64(data=w) for w in [wG, wH, wI, wJ]]

    # Publish wheel velocities to their respective topics
    front_left_pub.publish(wheel_msgs[0])
    front_right_pub.publish(wheel_msgs[1])
    rear_left_pub.publish(wheel_msgs[2])
    rear_right_pub.publish(wheel_msgs[3])

if __name__ == '__main__':
    rospy.init_node('vitirover_wheel_velocity_controller')
    
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    
    # Initialize publishers for each wheel
    front_left_pub = rospy.Publisher('/vitirover/left_front_wheel_velocity_controller/command', Float64, queue_size=10)
    front_right_pub = rospy.Publisher('/vitirover/right_front_wheel_velocity_controller/command', Float64, queue_size=10)
    rear_left_pub = rospy.Publisher('/vitirover/left_rear_wheel_velocity_controller/command', Float64, queue_size=10)
    rear_right_pub = rospy.Publisher('/vitirover/right_rear_wheel_velocity_controller/command', Float64, queue_size=10)
    
    rospy.spin()
