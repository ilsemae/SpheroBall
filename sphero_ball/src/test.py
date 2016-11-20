import numpy as np

x = 0
y = 0
theta = 0

# turtle position callback
def pose_callback(data):
	global x
	global y
	global theta
	x = data.x
	y = data.y
	theta = float(np.mod(data.theta,2*np.pi)) # bring angle value to within +2*pi
	#print(data)

# sphero position callback
def odom_callback(data):
	global x
	global y
	global theta
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	theta = float(np.mod(data.pose.pose.orientation.w,2*np.pi)) # bring angle value to within +2*pi
