#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose2D, Twist, PoseWithCovarianceStamped
import numpy as np

# ROS Topics for subscribers for input
ODOMETRY_TOPIC = '/odom'
CMD_VEL = '/cmd_vel'
ARUCO_MARKERS = '/fiducial_pose'


# ROS Topics used to publish the desired output messages
KALMAN_OUTPUT = '/super_pose'


class KalmanFilter:

	def __init__(self):

		pub_rate = 30
		
		# Variable initializations for odometry input
		self.position = Pose2D()
		self.position.theta = 0.0
		self.position.x = 0.0
		self.position.y = 0.0

		# Variable initializations for aruco markers
		self.position_aruco = Pose2D()
		self.position_aruco.theta = 0.0
		self.position_aruco.x = 0.0
		self.position_aruco.y = 0.0

		# Variable initializations for output
		self.est_position = Pose2D()
		self.est_position.theta = 0.0
		self.est_position.x = 0.0
		self.est_position.y = 0.0
		# Subscribe to the odometry topic
		rospy.Subscriber(ODOMETRY_TOPIC, Odometry, self.odom_callback)

		# Subscribe to the cmd_vel topic
		rospy.Subscriber(CMD_VEL, Twist, self.cmd_vel_callback)

		# Subscribe to the aruco markers topic
		rospy.Subscriber(ARUCO_MARKERS, PoseWithCovarianceStamped, self.aruco_markers_callback)

		# Initialize Publisher that will send filtered pose
		self.pose_pub = rospy.Publisher(KALMAN_OUTPUT, Pose2D, queue_size = 10)

		# Initialize a rospy node 
		rospy.init_node('kalman_filter')
		print("node_init")

		# Define the ROS node execution rate
		self.rate = rospy.Rate(pub_rate)


		######### Define kalman matrixes and variables

		# k = time

		# System Dyanamic Model, A size is equal to the number of states by the number of states
		self.A_k_min_1 = np.array([[1.0,   0,   0], 
						   		   [0,   1.0,   0], 
						   		   [0,     0, 1.0]])
		
		# Noise vector, V, for each of the states
		self.V_k_min_1 = np.array([0.01, 0.01, 0.003])

		# State model noise covariance, same rows as states. For odometry
		self.Q_k_odom = np.array([[1.0,   0,   0],
                             	  [  0, 1.0,   0],
                             	  [  0,   0, 1.0]])

		# State model noise covariance, same rows as states. For Aruco
		self.Q_k_aruco = np.array([[1.0,   0,   0],
                             	   [  0, 1.0,   0],
                             	   [  0,   0, 1.0]])

		# Observability matrix. Identity matrix because the states are the same as the odometry measurements
		self.H_k = np.array([[1.0,   0,   0], 
						     [0,   1.0,   0],
						     [0,     0, 1.0]])

		# Sensor noise covariance, sensor measurements x sensor measurements. If sensors are good R = 0
		self.R_k = np.array([[1.0,   0,    0],
                             [  0, 1.0,    0],
                             [  0,    0, 1.0]]) 

		# Sensor noise
		self.W_k = np.array([0.07, 0.07, 0.04])


		## Initial values
		# State vector estimation: [x_t-1, y_t-1, theta_t-1]
		self.X_k_min_1_odom = [0.0, 0.0, 0.0]

		# State vector estimation: [x_t-1, y_t-1, theta_t-1]
		self.X_k_min_1_aruco = [0.0, 0.0, 0.0]

		# Control input vector: [v, omega]
		self.U_k_min_1 = np.array([0.0, 0.0])

		# Covariance matrix P_k_minus_1
		self.P_k_min_1_odom = np.array([[0.1,   0,   0],
                                   [  0, 0.1,   0],
                                   [  0,   0, 0.1]])

		# Covariance matrix P_k_minus_1
		self.P_k_min_1_aruco = np.array([[0.1,   0,   0],
                                   [  0, 0.1,   0],
                                   [  0,   0, 0.1]])

		# Variables for B matrix
		self.omega_B = 0.0 # radians
		self.dk = 1 # seconds



	# Calculate B matrix. Number of states x number of control inputs (3x2)
	def getB(self, omega, dt):

		B = np.array([[np.cos(omega)*dt, 0],
               		  [np.sin(omega)*dt, 0],
               		  [0, dt]])

		return B 


	def ekf(self, Z_k, Q_k, X_k_min_1, P_k_min_1):

		## 1- Predict

		# Predict the state estimate at time k
		X_k = self.A_k_min_1 @ (X_k_min_1) + (self.getB(X_k_min_1[2], self.dk)) @ (self.U_k_min_1) + (self.V_k_min_1)

		# Predict state covariance
		P_k = self.A_k_min_1 @ P_k_min_1 @ self.A_k_min_1.T + Q_k


		## 2- Correct

		# Difference between sensor and predicted state
		Y_k = Z_k - (self.H_k @ X_k) + (self.W_k)

		# Residual covariance
		S_k = self.H_k @ P_k @ self.H_k.T + self.R_k

		# Kalman gain
		K_k = P_k @ self.H_k.T @ np.linalg.pinv(S_k)

		# Update state estimate
		X_k = X_k + (K_k @ Y_k)

		# Update state covariance
		P_k = P_k - (K_k @ self.H_k @ P_k)

		return X_k, P_k

	    
	    
	def run(self):
	    
		# Main loop
		while not rospy.is_shutdown():
		    if self.position is None: 
		        self.rate.sleep()
		        continue
		       
		    # Implemet kalman filter from methods

		    odom_input = [self.position.x, self.position.y, self.position.theta]
		    aruco_input = [self.position_aruco.x, self.position_aruco.y, self.position_aruco.theta]

		    X_k_odom, self.P_k_min_1_odom = self.ekf(odom_input, self.Q_k_odom, self.X_k_min_1_odom, self.P_k_min_1_odom)
		    X_k_aruco, self.P_k_min_1_aruco = self.ekf(aruco_input, self.Q_k_aruco, self.X_k_min_1_aruco, self.P_k_min_1_aruco)
		    
		    self.X_k_min_1_odom = X_k_odom
		    self.X_k_min_1_aruco = X_k_aruco

		    #Fuse the outputs of the two kalman filters
		    fusion = X_k_odom + (self.P_k_min_1_odom) @ ((self.P_k_min_1_odom + self.P_k_min_1_aruco).T) @ (X_k_odom - X_k_aruco)


		    self.est_position.x = fusion[0]
		    self.est_position.y = fusion[1]
		    self.est_position.theta = fusion[2]

			# Publish output 
		    output = self.est_position

		    self.pose_pub.publish(output)

		    self.rate.sleep()
		
		    
	def odom_callback(self, msg):
		
		# Obtain theta
		
		x = msg.pose.pose.orientation.x
		y = msg.pose.pose.orientation.y
		z = msg.pose.pose.orientation.z
		w = msg.pose.pose.orientation.w

		cov = msg.pose.covariance
		
		roll, ptch, yaw = euler_from_quaternion([x,y,z,w])
		
		radian = yaw
		
		
		# Obtain x and y 
		
		self.position.x = float(msg.pose.pose.position.x)
		self.position.y = float(msg.pose.pose.position.y)
		
		self.position.theta = float(radian*180/3.1416)

		# Update coviarnce matrix with covariance given by vector

		self.Q_k_odom = np.array([[cov[0],   0,   0],
                             [  0, cov[7],   0],
                             [  0,   0, cov[35]]])


	def aruco_markers_callback(self, msg):
		
		# Obtain theta
		
		x = msg.pose.pose.orientation.x
		y = msg.pose.pose.orientation.y
		z = msg.pose.pose.orientation.z
		w = msg.pose.pose.orientation.w

		cov = msg.pose.covariance
		
		roll, ptch, yaw = euler_from_quaternion([x,y,z,w])
		
		radian = yaw
		
		
		# Obtain x and y 
		
		self.position_aruco.x = float(msg.pose.pose.position.x)
		self.position_aruco.y = float(msg.pose.pose.position.y)
		
		self.position_aruco.theta = float(radian*180/3.1416)

		# Update coviarnce matrix with covariance given by vector

		self.Q_k_aruco = np.array([[cov[0],   0,   0],
                             [  0, cov[7],   0],
                             [  0,   0, cov[35]]])
		print(self.position_aruco)


	def cmd_vel_callback(self, msg):

		self.U_k_min_1[0] = msg.linear.x 
		self.U_k_min_1[1] = msg.angular.z 


		
	
if __name__ == '__main__':
    kalman_filter = KalmanFilter()
    try:
        kalman_filter.run()
    except rospy.ROSInterruptException:
        pass



