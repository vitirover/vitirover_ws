import rospy
import math

# source devel/setup.bash to be able to import them
from geometry_msgs.msg import Twist, PoseStamped 
from mobile_robot.msg import VitiroverMowerOrder
import std_msgs.msg
import socket
import telemetry_pb2 as telemetry_pb2
from google.protobuf.text_format import MessageToString



# if there is a problem with protobuf's version, you can try : 
# export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python


# Linear and angular velocity
v = 0
omega = 0.001

# BE CAREFUL WITH MOWER COMMANDS
# mower commands
left_mower_speed = 0
right_mower_speed = 0


# Constants for the robot
e = 0.340  # Track width (m)
r = 0.088  # Wheel radius (m)
x = 0.400 
dt = 1/60.0
robot_orientation = 0.0001

# Robot properties
robot_width, robot_height = 35, 18
robot_x, robot_y = 0, 0
robot_orientation = 0.001

# ROS Callback
def cmd_vel_callback(data):
    global v, omega
    v = data.linear.x
    omega = data.angular.z

def mower_order_callback(data):
    global left_mower_speed, right_mower_speed
    left_mower_speed = data.left_mower_speed
    right_mower_speed = data.right_mower_speed
    print("received mower order")


# Socket setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# to communicate with the robot throught ethernet, you must first set up your internet connection, for example 
# sudo ifconfig enp56s0 up 192.168.2.106 netmask 255.255.255.0

# sock.bind(("YOUR IP HERE", 5005))
sock.bind(("192.168.2.106", 5005))
sock.setblocking(0)

# Initialize ROS
rospy.init_node('vitirover_simulation', anonymous=True)
rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
rospy.Subscriber("mower_order", VitiroverMowerOrder, mower_order_callback)
pose_pub = rospy.Publisher("robot_pose", PoseStamped, queue_size=10)
axle_angle_pub = rospy.Publisher("back_axle_angle", std_msgs.msg.Float64, queue_size=10)

# Main loop
rate = rospy.Rate(10)
running = True
count = 0
count1 = 0

angle_value = 0

while running and not rospy.is_shutdown():
    # Robot motion simulation logic
    
    R = x / (math.tan(omega * dt) + 0.00001)

    wG = (v/r) + omega * e/(2*r)
    wH = (v/r) - omega * e/(2*r)
    wI = (v/r) * (1 - e/(2*R))
    wJ = (v/r) * (1 + e/(2*R))
    
    if v == 0:
        wG = wH = wI = wJ = 0

    # if v < 0:
    #     wG = -wG
    #     wH = -wH
    #     wI = -wI
    #     wJ = -wJ

    print("omega : " + str(omega))
    print('wG: ', wG)
    print('wH: ', wH)
    print('wI: ', wI)
    print('wJ: ', wJ)
    # if v < 0:
    #     wG = -wG
    #     wH = -wH
    #     wI = -wI
    #     wJ = -wJ

    print("omega : " + str(omega))
    print('wG: ', wG)
    print('wH: ', wH)
    print('wI: ', wI)
    print('wJ: ', wJ)

    # Send to your robot over socket    # if v < 0:
    #     wG = -wG
    #     wH = -wH
    #     wI = -wI
    #     wJ = -wJ

    robot_wants_to_go_right = angle_value < -15
    robot_wants_to_go_left = angle_value > 15
    if abs(omega) < 0.0001:
        pass
    elif omega < 0:
        print("droite")

        if robot_wants_to_go_right:
            wI =  wI / 2.0
        else:
            wI = 0
    elif omega > 0:
        print("gauche")
        if robot_wants_to_go_left:
            wJ = wJ / 2.0
        else:
            wJ = 0

    order = telemetry_pb2.VitiroverOrder()
    order.low_level_order.front_left_speed = int(wG * 8)
    order.low_level_order.front_right_speed = int(wH * 8)
    order.low_level_order.back_left_speed = int(wI * 8)
    order.low_level_order.back_right_speed = int(wJ * 8)

    print(order)

    print(order)

    data = order.SerializeToString()
    sock.sendto(data, ("192.168.2.42", 5005)) #Robot IP here


    if right_mower_speed != 0 or left_mower_speed != 0:
        mower_order = telemetry_pb2.VitiroverMowerOrder()
        mower_order.left_mower_speed = left_mower_speed
        mower_order.right_mower_speed = right_mower_speed
        order = telemetry_pb2.VitiroverOrder()
        order.mower_order.CopyFrom(mower_order)
        mower_data = order.SerializeToString()
        sock.sendto(mower_data, ("192.168.2.42", 5005)) #Robot IP here

    
    # data, _ = sock.recvfrom(20000)
    # telemetry_data = telemetry_pb2.VitiroverTelemetry()
    # telemetry_data.ParseFromString(data)
    # angle_value = telemetry_data.back_axle_angle
    print("angle value: ", angle_value)
    
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
        print(telemetry_data)
        angle_value = telemetry_data.back_axle_angle

        print(telemetry_data)
        angle_value = telemetry_data.back_axle_angle

        # Afficher quelques valeurs
        #print("Motor Data:", telemetry_data.front_left_wheel.back_electromotive_force)
        #print("wG/wGBFM: ", wG/(telemetry_data.front_left_wheel.back_electromotive_force+0.0001))
        #print("wH/wHBFM: ", wH/(telemetry_data.front_right_wheel.back_electromotive_force+0.0001))
        #print("wI/wIBFM: ", wI/(telemetry_data.back_left_wheel.back_electromotive_force+0.0001))
        #print("wJ/wJBFM: ", wJ/(telemetry_data.back_right_wheel.back_electromotive_force+0.0001))

    rate.sleep()
