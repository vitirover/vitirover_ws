
import rospy
from mobile_robot.msg import VitiroverMowerOrder

def publish_mower_orders():
    # Initialiser le noeud ROS
    rospy.init_node('mower_orders_publisher', anonymous=True)
    
    # Créer le publisher pour le topic "mower_orders"
    pub = rospy.Publisher('mower_order', VitiroverMowerOrder, queue_size=10)
    
    # Définir la fréquence de publication (toutes les 5 secondes)
    rate = rospy.Rate(0.2)  # 0.2 Hz correspond à une publication toutes les 5 secondes
    
    while not rospy.is_shutdown():
        # Créer un message VitiroverMowerOrder
        mower_order = VitiroverMowerOrder()
        mower_order.control_mode = VitiroverMowerOrder.PWM  # ou VitiroverMowerOrder.PWM selon ton besoin
        mower_order.left_mower_speed = 50
        mower_order.right_mower_speed = 50
        
        # Publier le message sur le topic "mower_orders"
        pub.publish(mower_order)
        print("sent")
        
        # Attendre avant de publier le prochain message
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_mower_orders()
    except rospy.ROSInterruptException:
        pass