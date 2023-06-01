#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def publish_helloworld():
    # Inicializar el nodo
    rospy.init_node('helloworld_publisher', anonymous=True)

    # Crear un publicador para el tópico "helloworld"
    pub = rospy.Publisher('helloworld', String, queue_size=10)

    # Frecuencia de publicación (10 Hz)
    rate = rospy.Rate(10)

    # Bucle de publicación
    while not rospy.is_shutdown():
        # Publicar el mensaje "Hello, World!"
        pub.publish("Hello, World!")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_helloworld()
    except rospy.ROSInterruptException:
        pass
