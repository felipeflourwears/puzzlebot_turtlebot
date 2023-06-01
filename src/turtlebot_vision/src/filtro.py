#!/usr/bin/env python3
import rospy
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import PoseWithCovarianceStamped

def fiducial_pose_callback(data):
    # Aquí puedes realizar las operaciones que desees con los datos recibidos
    # Accede a los campos del mensaje data de la siguiente manera:
    header = data.header
    poses = data.transforms

    # Ejemplo de acceso a los campos del primer fiducial en la lista de poses
    if len(poses) > 0:
        first_fiducial_pose = poses[0].fiducial_pose.pose
        position = first_fiducial_pose.position
        orientation = first_fiducial_pose.orientation
        covariance = poses[0].fiducial_pose.covariance

        # Crear un nuevo mensaje para el dato de covariance
        covariance_msg = PoseWithCovarianceStamped()
        covariance_msg.header = header
        covariance_msg.pose.covariance = covariance

        # Publicar el mensaje en el tópico "/fiducial_pose_data"
        pub.publish(covariance_msg)

def listener():
    # Inicializa el nodo
    rospy.init_node('fiducial_pose_subscriber', anonymous=True)

    # Crear un publicador para el tópico "/fiducial_pose_data"
    pub = rospy.Publisher("/fiducial_pose_data", PoseWithCovarianceStamped, queue_size=10)

    # Suscríbete al tópico "/fiducial_pose" y especifica la función de callback
    rospy.Subscriber("/fiducial_pose", FiducialTransformArray, fiducial_pose_callback)

    # Mantén el nodo en funcionamiento
    rospy.spin()

if __name__ == '__main__':
    listener()
