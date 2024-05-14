import rospy
import pygame
import math
from geometry_msgs.msg import Twist, PoseStamped
import std_msgs.msg
import socket
import telemetry_pb2 as telemetry_pb2
from google.protobuf.text_format import MessageToString
import math

# Linear and angular velocity
v = 0
omega = 0.001

# Constants for the robot
e = 0.340  # Track width (m)
r = 0.088  # Wheel radius (m)
x = 0.400 

# ROS Callback
def cmd_vel_callback(data):
    global v, omega
    v = data.linear.x
    omega = data.angular.z

# Socket setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("YOUR.IP.HERE", 5005))
sock.setblocking(0)

# Initialize ROS
rospy.init_node('vitirover_simulation', anonymous=True)
rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)

# Main loop
rate = rospy.Rate(10)
running = True
factor = 1

while running and not rospy.is_shutdown():
    data, _ = sock.recvfrom(20000)
    telemetry_data = telemetry_pb2.VitiroverTelemetry()
    telemetry_data.ParseFromString(data)

    angle_in_degrees = telemetry_data.back_axle_angle

    # Conversion de degrés en radians
    angle_value = angle_in_degrees * (math.pi / 180)
    print("angle value: ", angle_value)
    
    # Robot motion simulation logic
    # Matrix A calculation using the current back axle angle
    A = np.array([
        [(x + e * tan(angle_value) / 2) / (x * r), 0],
        [(x - e * tan(angle_value) / 2) / (x * r), 0],
        [(x + e * sin(angle_value) / 2) / (x * r * cos(angle_value)), -e / (2 * r)],
        [(x - e * sin(angle_value) / 2) / (x * r * cos(angle_value)), e / (2 * r)]
    ])

    # Calculate wheel velocities
    wRoues = np.dot(A, np.array([v, omega]))

    # Publish wheel velocities
    publish_wheel_velocities(wRoues[0], wRoues[1], wRoues[2], wRoues[3])
    
    wG = wRoues[0]
    wH = wRoues[1]
    wI = wRoues[2]
    wJ = wRoues[3]

    print('wG: ', wG)
    print('wH: ', wH)
    print('wI: ', wI)
    print('wJ: ', wJ)

    # Send to your robot over socket
    order = telemetry_pb2.VitiroverOrder()
    order.low_level_order.front_left_speed = int(wG * factor)
    order.low_level_order.front_right_speed = int(wH * factor)
    order.low_level_order.back_left_speed = int(wI * factor)
    order.low_level_order.back_right_speed = int(wJ * factor)

    data = order.SerializeToString()
    sock.sendto(data, ("192.168.2.42", 5005)) #Robot IP here

    telemetry_data = None
    while True:
        try:
            data, addr = sock.recvfrom(20000)  # Buffer size
            telemetry_data = telemetry_pb2.VitiroverTelemetry()
        except BlockingIOError:
            # Plus de messages dans le buffer, sortir de la boucle
            break
    if telemetry_data != None:
        telemetry_data.ParseFromString(data)
        # Afficher quelques valeurs
        #print("Motor Data:", telemetry_data.front_left_wheel.back_electromotive_force)
        #print("wG/wGBFM: ", wG/(telemetry_data.front_left_wheel.back_electromotive_force+0.0001))
        #print("wH/wHBFM: ", wH/(telemetry_data.front_right_wheel.back_electromotive_force+0.0001))
        #print("wI/wIBFM: ", wI/(telemetry_data.back_left_wheel.back_electromotive_force+0.0001))
        #print("wJ/wJBFM: ", wJ/(telemetry_data.back_right_wheel.back_electromotive_force+0.0001))

    rate.sleep()

pygame.quit(),
